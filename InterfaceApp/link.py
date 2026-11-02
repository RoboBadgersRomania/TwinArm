"""Link to the LightGateway, which passes text lines to and from the Teensy console, over WiFi or the radio.

connect("192.168.4.1", 4210)   WiFi (UDP) to the follower's gateway
connect("COM13")               USB serial to the master arm's gateway, which relays the same lines over the encrypted,
                               frequency-hopping LoRa radio to the follower's gateway (master_arm.ino, TwinLink.h)

PC -> gateway  "#hello <ms>"             registers this PC; answered with G,<ms>,<uptime ms>,<rssi dBm>
               "#save <id> <4 gains>" / "#load"   gain backup in the gateway's flash; answered with B,id,<4 gains> / B,end,n
PC -> Teensy   "1 a12.50", "t -30", "x"  motorcontroller.ino console commands, several lines per datagram
Teensy -> PC   T,id,ok,deg,target,amps,tempC,volts,busAmps,mode,fault,jammed   one line per motor every 100 ms
               P,id,ok,posKp,posKi,velKp,velKi   answer to "<id> g ..."     J,deg,ms   answer to "j ..."
               S,id,begin,start,target,sampleMs / S,id,i,deg,deg,... / S,id,end   step test curve, answer to "<id> s<deg>"
               R,idle|rec|move|play,take,secondsIn,takeSeconds,sdOk   the Teensy's recorder, with every telemetry sweep
               K,n,seconds ... K,end,count   the takes on the Teensy's SD card, answer to "takes"
               Q,on|off,lead,slowestUs,id:wn:zeta,...   the replay MPC, answer to "mpc ..." (wn 0 = no model)
               Q,test,ok,us   answer to "mpc test"
gateway -> PC  M,turret,shoulder,elbow,wristPitch,rollPot%,gripClosed,okBits   the master arm 20 times a second:
               joints 0..360 deg (-1 no reading), the wrist-roll pot 0..100 %, the gripper button's state (1 closed)
               L,m|f,state,slots/s,ok%,rssi,snr,peerRssi,peerSnr,MHz   the radio once a second, seen from the master (m)
               or the follower (f); state -2 no radio, -1 no key, 0 searching, 1 up
"""
import socket
import time
from collections import deque

MODES = ["off", "volt", "current", "speed", "pos"]
FAULTS = ["voltage", "current", "temp", "encoder", "", "", "hardware", "software"]  # bit order
RADIO_STATES = {-2: "no radio module", -1: "no key set", 0: "searching", 1: "up, encrypted, hopping"}


def now_ms():
    return int(time.monotonic() * 1000) & 0xFFFFFFFF


def is_serial(host):
    return host.upper().startswith("COM") or host.startswith("/dev/")


class Motor:
    def __init__(self, history):
        self.ok = False
        self.deg = self.target = self.amps = self.volts = self.bus_amps = 0.0
        self.temp = self.mode = self.fault = 0
        self.jam = False                     # the Teensy saw it stall short of its target and holds it there
        self.seen = 0.0                      # monotonic time of the last line
        self.hist = deque(maxlen=history)    # (t, error or None when off, amps, temp, deg, target)
        self.gains, self.gains_ok, self.gains_at = None, False, 0.0  # (pos kp, pos ki, vel kp, vel ki) from the driver

    @property
    def watts(self):
        return self.volts * self.bus_amps

    @property
    def faults(self):
        return ", ".join(n for i, n in enumerate(FAULTS) if self.fault >> i & 1 and n) or ("0x%02X" % self.fault if self.fault else "")


class Link:
    def __init__(self, ids, history=1200):  # 2 min at 10 Hz
        self.sock, self.addr = None, None
        self.ser, self.rx = None, b""        # serial port to the master's gateway, and a line not finished yet
        self.master, self.radio = None, None  # last M and L lines, parsed, with their arrival time
        self.take, self.takes = None, {}     # the Teensy's recorder (last R line); takes on its card: number -> s
        self.motors = {i: Motor(history) for i in ids}
        self.gw_seen, self.rtt, self.rssi = 0.0, None, 0
        self.lines = deque(maxlen=200)       # arrival times of T lines, for the rate
        self.jam_cfg, self.jam_at = None, 0.0  # (tolerance deg, time ms) the Teensy uses
        self.captures = {}                   # CAN id -> last step test: start, target, dt, samples {i: deg}, done, at
        self.backup, self.backup_end = {}, None  # CAN id -> gains kept on the gateway; (motor count, time) of the last #load
        self.mpc, self.mpc_test = None, None  # the Teensy's replay MPC (last Q line), its last self-test

    def connect(self, host, port=0):
        """A COM port (or /dev/tty...) is the master's gateway on USB; OSError if it cannot be opened."""
        self.close()
        if is_serial(host):
            import serial  # pyserial: only the radio link needs it
            s = serial.Serial()
            s.port, s.baudrate, s.timeout, s.write_timeout = host, 115200, 0, 0
            s.dtr, s.rts = True, False  # RTS on with DTR off would reset the ESP32-S3
            s.open()
            self.ser, self.rx = s, b""
        else:
            self.addr = (host, int(port))
            self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            self.sock.setblocking(False)
        self.hello()

    def close(self):
        if self.sock:
            self.sock.close()
        if self.ser:
            self.ser.close()
        self.sock = self.ser = None
        self.gw_seen = 0.0

    def send(self, *lines):
        if not lines:
            return
        data = ("\n".join(lines) + "\n").encode()
        try:
            if self.ser:
                self.ser.write(data)
            elif self.sock:
                self.sock.sendto(data, self.addr)
        except OSError:  # unreachable network, bad host, full or unplugged port: the status shows offline
            pass

    def hello(self):
        self.send("#hello %d" % now_ms())

    def poll(self):
        if self.ser:
            try:
                data = self.ser.read(self.ser.in_waiting or 1)
            except OSError:  # unplugged: Disconnect, plug in, Connect
                self.ser.close()
                self.ser = None
                return
            self.feed(data, time.monotonic())
        while self.sock:
            try:
                data = self.sock.recv(2048)
            except (BlockingIOError, ConnectionResetError, OSError):  # Windows reports ICMP port-unreachable as a reset
                return
            for line in data.decode(errors="replace").splitlines():
                self.handle(line.strip(), time.monotonic())

    def feed(self, data, t):
        """Bytes from the serial port: the complete lines are handled, the rest waits for its end."""
        *lines, self.rx = (self.rx + data).split(b"\n")
        for line in lines:
            self.handle(line.decode(errors="replace").strip(), t)

    def handle(self, line, t):
        f = line.split(",")
        try:
            if f[0] == "G" and len(f) >= 4:
                self.gw_seen, self.rssi = t, int(f[3])
                self.rtt = (now_ms() - int(f[1])) & 0xFFFFFFFF
            elif f[0] == "T" and len(f) in (11, 12) and int(f[1]) in self.motors:  # 11 = firmware before jam detection
                m = self.motors[int(f[1])]
                m.ok = f[2] == "1"
                m.deg, m.target, m.amps = float(f[3]), float(f[4]), float(f[5])
                m.temp, m.volts, m.bus_amps = int(f[6]), float(f[7]), float(f[8])
                m.mode, m.fault = int(f[9]), int(f[10])
                m.jam = len(f) == 12 and f[11] == "1"
                m.seen = self.gw_seen = t
                self.lines.append(t)
                if m.ok:
                    m.hist.append((t, m.target - m.deg if m.mode else None, m.amps, m.temp, m.deg, m.target))
            elif f[0] == "P" and len(f) == 7 and int(f[1]) in self.motors:
                m = self.motors[int(f[1])]
                m.gains_ok, m.gains_at = f[2] == "1", t
                if m.gains_ok:
                    m.gains = tuple(float(v) for v in f[3:7])
            elif f[0] == "J" and len(f) == 3:
                self.jam_cfg, self.jam_at = (float(f[1]), int(f[2])), t
            elif f[0] == "B" and len(f) == 3 and f[1] == "end":
                self.backup_end = (int(f[2]), t)
            elif f[0] == "B" and len(f) == 6 and int(f[1]) in self.motors:
                self.backup[int(f[1])] = tuple(float(v) for v in f[2:6])
            elif f[0] == "S" and len(f) >= 3 and int(f[1]) in self.motors:
                cid, cap = int(f[1]), self.captures.get(int(f[1]))
                if f[2] == "begin" and len(f) == 6:
                    self.captures[cid] = {"start": float(f[3]), "target": float(f[4]), "dt": int(f[5]) / 1000,
                                          "samples": {}, "done": False, "fail": False, "at": t, "last": t}
                elif f[2] in ("end", "fail"):
                    cap = cap if cap and f[2] == "end" else {"samples": {}, "fail": True}
                    cap.update(done=True, at=t)
                    self.captures[cid] = cap
                elif cap and not cap["done"] and len(f) >= 4:
                    for k, v in enumerate(f[3:]):  # by index: datagrams may arrive out of order, or not at all
                        cap["samples"][int(f[2]) + k] = float(v)
                    cap["last"] = t
            elif f[0] == "M" and len(f) == 8:
                ok = int(f[7])
                self.master = {"deg": [float(v) for v in f[1:5]], "ok": [bool(ok >> j & 1) for j in range(4)],
                               "roll": float(f[5]), "grip": f[6] == "1", "seen": t}
            elif f[0] == "R" and len(f) == 6:
                self.take = {"state": f[1], "no": int(f[2]), "at": float(f[3]), "len": float(f[4]), "sd": f[5] == "1",
                             "seen": t}
            elif f[0] == "K" and len(f) == 3 and f[1] != "end":
                self.takes[int(f[1])] = float(f[2])
            elif f[0] == "Q" and len(f) == 4 and f[1] == "test":
                self.mpc_test = {"ok": f[2] == "1", "us": int(f[3]), "seen": t}
            elif f[0] == "Q" and len(f) >= 4:
                models = {}
                for part in f[4:]:
                    cid, wn, zeta = part.split(":")
                    if float(wn) > 0:
                        models[int(cid)] = (float(wn), float(zeta))
                self.mpc = {"on": f[1] == "on", "lead": float(f[2]), "us": int(f[3]), "models": models, "seen": t}
            elif f[0] == "L" and len(f) == 10:
                self.radio = {"role": f[1], "state": int(f[2]), "rate": int(f[3]), "ok": int(f[4]), "rssi": float(f[5]),
                              "snr": float(f[6]), "peer_rssi": float(f[7]), "peer_snr": float(f[8]), "mhz": float(f[9]),
                              "seen": t}
        except ValueError:
            pass  # a garbled line is dropped; the next one comes 100 ms later

    def master_online(self):
        return self.master is not None and time.monotonic() - self.master["seen"] < 0.5

    def recorder(self):
        """The last R line while it is fresh (the Teensy sends one every 100 ms), else None."""
        return self.take if self.take and time.monotonic() - self.take["seen"] < 1.5 else None

    def radio_status(self):
        """The last L line while it is fresh, else None."""
        return self.radio if self.radio and time.monotonic() - self.radio["seen"] < 2.5 else None

    def gateway_online(self):
        return time.monotonic() - self.gw_seen < 2.5

    def robot_online(self):
        return any(m.ok and time.monotonic() - m.seen < 1.0 for m in self.motors.values())

    def rate(self):
        """T lines per second over the last second, divided by the motor count = telemetry Hz."""
        t = time.monotonic()
        return sum(1 for s in self.lines if t - s < 1.0) / max(1, len(self.motors))


if __name__ == "__main__":
    gw = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    gw.bind(("127.0.0.1", 0))
    link = Link([1, 2])
    link.connect(*gw.getsockname())
    msg, pc = gw.recvfrom(100)
    assert msg.startswith(b"#hello "), msg
    t = int(msg.split()[1])
    gw.sendto(b"G,%d,1234,-48\nT,1,1,12.50,15.00,0.520,38,24.10,0.31,4,5\nT,2,0,0,0,0,0,0,0,0,0,1\nT,9,1\n"
              b"P,1,1,20.0000,0.5000,3.0000,0.1000\nP,2,0,nan,nan,nan,nan\nJ,4.00,400\n" % t, pc)
    time.sleep(0.05)
    link.poll()
    m = link.motors[1]
    assert link.gateway_online() and link.rssi == -48 and link.rtt is not None
    assert m.ok and m.deg == 12.5 and m.target == 15 and m.temp == 38 and abs(m.watts - 7.471) < 1e-9
    assert m.hist[-1][1] == 2.5 and m.faults == "voltage, temp" and not link.motors[2].ok
    assert not m.jam and link.motors[2].jam  # old 11-field lines parse as not jammed
    assert m.gains == (20, 0.5, 3, 0.1) and m.gains_ok and m.hist[-1][4:] == (12.5, 15)
    assert not link.motors[2].gains_ok and link.motors[2].gains is None and link.jam_cfg == (4.0, 400)
    gw.sendto(b"S,1,begin,2.000,7.000,10\nS,1,10,5.0,6.0\nS,1,0,2.1,3.5,nan\nS,1,end\nS,2,fail\n", pc)
    time.sleep(0.05)
    link.poll()
    cap = link.captures[1]
    assert cap["done"] and not cap["fail"] and cap["dt"] == 0.01 and cap["target"] == 7 and cap["samples"][0] == 2.1
    assert cap["samples"][1] == 3.5 and cap["samples"][11] == 6.0 and 5 not in cap["samples"]
    assert link.captures[2]["fail"] and link.captures[2]["done"]
    gw.sendto(b"B,1,20.0000,0.0000,8.0000,0.1000\nB,end,1\n", pc)
    time.sleep(0.05)
    link.poll()
    assert link.backup == {1: (20, 0, 8, 0.1)} and link.backup_end[0] == 1
    link.feed(b"M,12.50,-1,300.25,90.00,48.2,1,13\r\nL,m,1,31,97,-42,9.5,-45,8.8,433.70\nT,1,1,1", time.monotonic())
    assert link.master_online() and link.master["ok"] == [True, False, True, True] and link.master["deg"][2] == 300.25
    assert link.master["roll"] == 48.2 and link.master["grip"] and link.radio_status()["state"] == 1
    assert link.radio["mhz"] == 433.7
    assert link.rx == b"T,1,1,1"  # the unfinished line waits for the rest
    link.feed(b",12.50,15.00,0.520,38,24.10,0.31,4,5,0\n", time.monotonic())
    assert link.rx == b"" and link.motors[1].deg == 12.5 and is_serial("COM13") and not is_serial("192.168.4.1")
    link.feed(b"R,play,3,4.20,12.30,1\nK,1,5.00\nK,3,12.30\nK,end,2\n", time.monotonic())
    assert link.recorder() == dict(link.take, state="play", no=3, at=4.2, len=12.3, sd=True)
    assert link.takes == {1: 5.0, 3: 12.3}
    link.feed(b"Q,on,25.0,1480,1:0.00:0.00,2:6.20:0.77,3:5.10:0.90,4:0.00:0.00\r\nQ,test,1,365\n", time.monotonic())
    assert link.mpc["on"] and link.mpc["lead"] == 25 and link.mpc["us"] == 1480
    assert link.mpc["models"] == {2: (6.2, 0.77), 3: (5.1, 0.9)} and link.mpc_test["ok"] and link.mpc_test["us"] == 365
    print("link ok")

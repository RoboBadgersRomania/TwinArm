"""UDP link to the LightGateway, which passes text lines to and from the Teensy console.

PC -> gateway  "#hello <ms>"             registers this PC; answered with G,<ms>,<uptime ms>,<rssi dBm>
PC -> Teensy   "1 a12.50", "t -30", "x"  motorcontroller.ino console commands, several lines per datagram
Teensy -> PC   T,id,ok,deg,target,amps,tempC,volts,busAmps,mode,fault,jammed   one line per motor every 100 ms
"""
import socket
import time
from collections import deque

MODES = ["off", "volt", "current", "speed", "pos"]
FAULTS = ["voltage", "current", "temp", "encoder", "", "", "hardware", "software"]  # bit order


def now_ms():
    return int(time.monotonic() * 1000) & 0xFFFFFFFF


class Motor:
    def __init__(self, history):
        self.ok = False
        self.deg = self.target = self.amps = self.volts = self.bus_amps = 0.0
        self.temp = self.mode = self.fault = 0
        self.jam = False                     # the Teensy saw it stall short of its target and holds it there
        self.seen = 0.0                      # monotonic time of the last line
        self.hist = deque(maxlen=history)    # (t, error or None when off, amps, temp)

    @property
    def watts(self):
        return self.volts * self.bus_amps

    @property
    def faults(self):
        return ", ".join(n for i, n in enumerate(FAULTS) if self.fault >> i & 1 and n) or ("0x%02X" % self.fault if self.fault else "")


class Link:
    def __init__(self, ids, history=1200):  # 2 min at 10 Hz
        self.sock, self.addr = None, None
        self.motors = {i: Motor(history) for i in ids}
        self.gw_seen, self.rtt, self.rssi = 0.0, None, 0
        self.lines = deque(maxlen=200)       # arrival times of T lines, for the rate

    def connect(self, host, port):
        self.close()
        self.addr = (host, int(port))
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.setblocking(False)
        self.hello()

    def close(self):
        if self.sock:
            self.sock.close()
        self.sock = None
        self.gw_seen = 0.0

    def send(self, *lines):
        if self.sock and lines:
            try:
                self.sock.sendto(("\n".join(lines) + "\n").encode(), self.addr)
            except OSError:  # unreachable network, bad host: the status shows offline
                pass

    def hello(self):
        self.send("#hello %d" % now_ms())

    def poll(self):
        while self.sock:
            try:
                data = self.sock.recv(2048)
            except (BlockingIOError, ConnectionResetError, OSError):  # Windows reports ICMP port-unreachable as a reset
                return
            for line in data.decode(errors="replace").splitlines():
                self.handle(line.strip(), time.monotonic())

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
                    m.hist.append((t, m.target - m.deg if m.mode else None, m.amps, m.temp))
        except ValueError:
            pass  # a garbled line is dropped; the next one comes 100 ms later

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
    gw.sendto(b"G,%d,1234,-48\nT,1,1,12.50,15.00,0.520,38,24.10,0.31,4,5\nT,2,0,0,0,0,0,0,0,0,0,1\nT,9,1\n" % t, pc)
    time.sleep(0.05)
    link.poll()
    m = link.motors[1]
    assert link.gateway_online() and link.rssi == -48 and link.rtt is not None
    assert m.ok and m.deg == 12.5 and m.target == 15 and m.temp == 38 and abs(m.watts - 7.471) < 1e-9
    assert m.hist[-1][1] == 2.5 and m.faults == "voltage, temp" and not link.motors[2].ok
    assert not m.jam and link.motors[2].jam  # old 11-field lines parse as not jammed
    print("link ok")

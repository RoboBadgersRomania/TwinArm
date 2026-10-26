"""Fake LightGateway + Teensy for trying the app without hardware.

python sim.py, then python main.py 127.0.0.1. Speaks the same lines as the real pair: answers #hello,
follows the console commands and sends T lines at 10 Hz. Each joint is a toy spring-damper driven by its
position Kp and velocity Kp (g command), capped at its rpm, so step tests react to tuning.
The elbow (#2) hits a fake obstacle at +70 deg and jams, with the same rule as checkJam on the Teensy.
"""
import math
import random
import socket
import sys
import time

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.bind(("127.0.0.1", 4210))
sock.setblocking(False)
motors = {i: {"deg": 0.0, "target": 0.0, "rpm": 10.0, "mode": 4, "temp": 31.0, "amps": 0.0, "fault": 0,
              "last": 0.0, "stuck": 0.0, "jam": 0, "vel": 0.0, "gains": [20.0, 0.0, 8.0, 0.1]} for i in (1, 2, 3, 4)}  # made-up gains, not SteadyWin defaults
OBSTACLE = {2: 70.0}
jam_cfg = [4.0, 400]  # tolerance deg, time ms (0 = off)
backup = {}           # the gateway's flash gain backup: id -> 4 gains
SIM_DROP_END = "--drop-end" in sys.argv  # lose every step test's end line, as a lossy WiFi might
HIDDEN_CAP = {4: 15.0} if "--slow-roll" in sys.argv else {}  # deg/s the driver itself allows, whatever 'v' says
pc, last, next_tx = None, time.monotonic(), 0.0
print("fake gateway on 127.0.0.1:4210, run: python main.py 127.0.0.1")


def move(i, m, dt):
    """Toy spring-damper joint from position Kp and velocity Kp, capped at the rpm, stopped by the obstacle."""
    if m["mode"]:
        kp, _, kv, _ = m["gains"]
        vmax = min(m["rpm"] * 6, HIDDEN_CAP.get(i, 1e9))  # rpm -> deg/s
        m["vel"] = max(-vmax, min(vmax, m["vel"] + (kp * (m["target"] - m["deg"]) - kv * m["vel"]) * dt))
        m["deg"] += m["vel"] * dt
        if m["deg"] >= OBSTACLE.get(i, 1e9):
            m["deg"], m["vel"] = OBSTACLE[i], 0.0


def command(line):
    global pc
    if line.startswith("#hello"):
        sock.sendto(("G,%s,%d,-42\n" % (line.split()[1], time.monotonic() * 1000)).encode(), pc)
    elif line.startswith("#save "):
        p = line.split()
        backup[int(p[1])] = [float(v) for v in p[2:6]]
        sock.sendto(("B,%s,%s\n" % (p[1], ",".join("%.4f" % v for v in backup[int(p[1])]))).encode(), pc)
    elif line.startswith("#load"):
        for i, g in sorted(backup.items()):
            sock.sendto(("B,%d,%s\n" % (i, ",".join("%.4f" % v for v in g))).encode(), pc)
        sock.sendto(("B,end,%d\n" % len(backup)).encode(), pc)
    elif line in ("x", "h"):
        for m in motors.values():
            m.update(mode=0) if line == "x" else m.update(target=0.0, mode=4, jam=0, stuck=0.0)
    elif line.startswith("t "):
        print("turret", line[2:])
    elif line.startswith("p "):
        print("gripper %s %% open" % line[2:])
    elif line == "j" or line.startswith("j "):
        p = line.split()
        if len(p) == 3 and float(p[1]) > 0 and int(p[2]) >= 0:
            jam_cfg[:] = [float(p[1]), int(p[2])]
        sock.sendto(("J,%.2f,%d\n" % tuple(jam_cfg)).encode(), pc)
    else:
        i, op = line.split(maxsplit=1)
        m = motors[int(i)]
        if op[0] == "g":
            vals = op[1:].split()
            if len(vals) == 4:
                m["gains"] = [float(v) for v in vals]
            sock.sendto(("P,%s,1,%s\n" % (i, ",".join("%.4f" % v for v in m["gains"]))).encode(), pc)
            return
        c, v = op[0], float(op[1:] or 0)
        if c in "ars" and m["jam"]:
            return
        if c == "s":  # step test like stepTest on the Teensy: 2 s every 10 ms, simulated ahead in 1 ms steps
            start = m["deg"]
            m["target"] = start + v
            send = lambda s: sock.sendto((s + "\n").encode(), pc)
            ys = []
            for n in range(200):
                for _ in range(10):
                    move(int(i), m, 0.001)
                ys.append(m["deg"] + random.gauss(0, 0.01))
            send("S,%s,begin,%.3f,%.3f,10" % (i, start, m["target"]))
            for n in range(0, 200, 10):  # 10 samples a line, like the Teensy
                send("S,%s,%d,%s" % (i, n, ",".join("%.2f" % y for y in ys[n:n + 10])))
            if not SIM_DROP_END:
                send("S,%s,end" % i)
            m["last"] = m["deg"]
            return
        if c == "a":
            m.update(target=v, mode=4)
        elif c == "r":
            m["target"] += v
        elif c == "v":
            m["rpm"] = v
        elif c == "m":
            print("motor %s max current %.1f A" % (i, v))
        elif c == "o":
            m["mode"] = 0
        elif c == "c":
            m.update(fault=0, jam=0, stuck=0.0)


while True:
    try:
        data, pc = sock.recvfrom(2048)
        for line in data.decode().splitlines():
            try:
                command(line.strip())
            except (ValueError, KeyError, IndexError):
                print("? " + line)
    except (BlockingIOError, ConnectionResetError):
        pass
    now = time.monotonic()
    dt, last = now - last, now
    for i, m in motors.items():
        move(i, m, dt)
        moving =m["mode"] and abs(m["target"] - m["deg"]) > 0.01
        pushing = moving and m["deg"] >= OBSTACLE.get(i, 1e9)
        gravity = 2.2 * math.cos(math.radians(m["deg"] + 40 * i)) if i < 4 else 0.2
        m["amps"] = (gravity + (1.5 if moving else 0)) * (m["mode"] > 0) + 4.5 * pushing + random.gauss(0, 0.04)
        m["temp"] += (31 + 9 * abs(m["amps"]) - m["temp"]) * dt / 40
    if pc and now >= next_tx:
        next_tx = now + 0.1
        for i, m in motors.items():  # one line per datagram, like the gateway
            stuck = (m["mode"] and abs(m["target"] - m["deg"]) > jam_cfg[0]
                     and abs(m["deg"] - m["last"]) < min(0.25 * m["rpm"] * 6 * 0.1, 1.0))  # as checkJam on the Teensy
            m["stuck"], m["last"] = (m["stuck"] + 0.1 if stuck else 0.0), m["deg"]
            if jam_cfg[1] and m["stuck"] >= jam_cfg[1] / 1000 and not m["jam"]:
                m.update(jam=1, target=m["deg"])
                print("motor %d jammed at %.1f" % (i, m["deg"]))
            line = "T,%d,1,%.2f,%.2f,%.3f,%d,%.2f,%.2f,%d,%d,%d\n" % (
                i, m["deg"] + random.gauss(0, 0.03), m["target"], m["amps"], m["temp"], 24.3 + random.gauss(0, 0.05),
                abs(m["amps"]) * 0.32, m["mode"], m["fault"], m["jam"])
            sock.sendto(line.encode(), pc)
    time.sleep(0.002)

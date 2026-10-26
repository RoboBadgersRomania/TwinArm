"""Fake LightGateway + Teensy for trying the app without hardware.

python sim.py, then python main.py 127.0.0.1. Speaks the same lines as the real pair: answers #hello,
follows the console commands with rate-limited joints and sends T lines at 10 Hz.
The elbow (#2) hits a fake obstacle at +70 deg and jams, with the same rule as checkJam on the Teensy.
"""
import math
import random
import socket
import time

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.bind(("127.0.0.1", 4210))
sock.setblocking(False)
motors = {i: {"deg": 0.0, "target": 0.0, "rpm": 10.0, "mode": 4, "temp": 31.0, "amps": 0.0, "fault": 0,
              "last": 0.0, "stuck": 0.0, "jam": 0} for i in (1, 2, 3, 4)}
OBSTACLE = {2: 70.0}
pc, last, next_tx = None, time.monotonic(), 0.0
print("fake gateway on 127.0.0.1:4210, run: python main.py 127.0.0.1")


def command(line):
    global pc
    if line.startswith("#hello"):
        sock.sendto(("G,%s,%d,-42\n" % (line.split()[1], time.monotonic() * 1000)).encode(), pc)
    elif line in ("x", "h"):
        for m in motors.values():
            m.update(mode=0) if line == "x" else m.update(target=0.0, mode=4, jam=0, stuck=0.0)
    elif line.startswith("t "):
        print("turret", line[2:])
    else:
        i, op = line.split(maxsplit=1)
        m, c, v = motors[int(i)], op[0], float(op[1:] or 0)
        if c in "ar" and m["jam"]:
            return
        if c == "a":
            m.update(target=v, mode=4)
        elif c == "r":
            m["target"] += v
        elif c == "v":
            m["rpm"] = v
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
        if m["mode"]:
            step = m["rpm"] * 6 * dt  # rpm -> deg/s
            m["deg"] = min(m["deg"] + max(-step, min(step, m["target"] - m["deg"])), OBSTACLE.get(i, 1e9))
        moving = m["mode"] and abs(m["target"] - m["deg"]) > 0.01
        pushing = moving and m["deg"] >= OBSTACLE.get(i, 1e9)
        gravity = 2.2 * math.cos(math.radians(m["deg"] + 40 * i)) if i < 4 else 0.2
        m["amps"] = (gravity + (1.5 if moving else 0)) * (m["mode"] > 0) + 4.5 * pushing + random.gauss(0, 0.04)
        m["temp"] += (31 + 9 * abs(m["amps"]) - m["temp"]) * dt / 40
    if pc and now >= next_tx:
        next_tx = now + 0.1
        for i, m in motors.items():  # one line per datagram, like the gateway
            stuck = m["mode"] and abs(m["target"] - m["deg"]) > 4 and abs(m["deg"] - m["last"]) < 0.25 * m["rpm"] * 6 * 0.1
            m["stuck"], m["last"] = (m["stuck"] + 0.1 if stuck else 0.0), m["deg"]
            if m["stuck"] >= 0.4 and not m["jam"]:
                m.update(jam=1, target=m["deg"])
                print("motor %d jammed at %.1f" % (i, m["deg"]))
            line = "T,%d,1,%.2f,%.2f,%.3f,%d,%.2f,%.2f,%d,%d,%d\n" % (
                i, m["deg"] + random.gauss(0, 0.03), m["target"], m["amps"], m["temp"], 24.3 + random.gauss(0, 0.05),
                abs(m["amps"]) * 0.32, m["mode"], m["fault"], m["jam"])
            sock.sendto(line.encode(), pc)
    time.sleep(0.002)

"""Fake LightGateway + Teensy for trying the app without hardware.

python sim.py, then python main.py 127.0.0.1. Speaks the same lines as the real pair: answers #hello,
follows the console commands and sends T lines at 10 Hz. Each joint is a toy spring-damper driven by its
position Kp and velocity Kp (g command), capped at its rpm, so step tests react to tuning.
The elbow (#2) hits a fake obstacle at +70 deg and jams, with the same rule as checkJam on the Teensy.
It also passes on a fake master arm (M lines, the wrist sensor without a reading) and a radio link that is up (L lines),
and has the Teensy's recorder: rec / stop / play / load / takes, with R and K lines (at 10 Hz, no move-in).

python sim.py --model step_tests/model_fit.json   joints move like the real ones fitted by compare.py / the app's
whole-arm test, starting with the gains they were measured with; changing gains scales the fit like a cascaded
P (position) -> P (velocity) loop: kp ~ pos Kp * vel Kp, kv ~ vel Kp. An approximation, good for trying directions.
"""
import json
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
rec = {"state": "idle", "no": 0, "take": [], "t0": 0.0, "speed": 1.0}  # the Teensy's recorder, at the 10 Hz tick
takes = {}            # its SD card: number -> samples ({CAN id: target} every 0.1 s)
mpc_state = {"models": {}}  # the Teensy's replay MPC (always on), state only: the sim's replay ignores it
SIM_DROP_END = "--drop-end" in sys.argv  # lose every step test's end line, as a lossy WiFi might
HIDDEN_CAP = {4: 15.0} if "--slow-roll" in sys.argv else {}  # deg/s the driver itself allows, whatever 'v' says
MODEL = {}  # CAN id -> fitted real joint: kp, kv, vmax and the gains / speed limit it was measured with
if "--model" in sys.argv:
    with open(sys.argv[sys.argv.index("--model") + 1]) as f:
        MODEL = {int(k): v for k, v in json.load(f).items()}
    for i, fit in MODEL.items():
        if fit.get("gains"):
            motors[i]["gains"] = list(fit["gains"])  # Read in the app shows what the real driver had
        print("motor %d moves like the fitted %s: wn %.1f rad/s, zeta %.2f, top %s"
              % (i, fit["motor"], fit["wn"], fit["zeta"], "%.0f deg/s" % fit["vmax"] if fit["vmax"] else "its speed limit"),
              flush=True)
pc, last, next_tx = None, time.monotonic(), 0.0
print("fake gateway on 127.0.0.1:4210, run: python main.py 127.0.0.1")


def move(i, m, dt):
    """Toy spring-damper joint from position Kp and velocity Kp, capped at the rpm, stopped by the obstacle."""
    if m["mode"]:
        kp, _, kv, _ = m["gains"]
        vmax = min(m["rpm"] * 6, HIDDEN_CAP.get(i, 1e9))  # rpm -> deg/s
        fit = MODEL.get(i)
        if fit:  # the fitted real joint, scaled by how far the gains and speed moved since the test
            g0 = fit.get("gains") or m["gains"]
            pk = m["gains"][0] * m["gains"][2] / (g0[0] * g0[2]) if g0[0] * g0[2] else 1.0
            vk = m["gains"][2] / g0[2] if g0[2] else 1.0
            kp, kv = fit["kp"] * pk, fit["kv"] * vk
            if fit["vmax"]:  # a cap the real joint hit; None = it never did, keep the commanded limit
                vmax = fit["vmax"] * (m["rpm"] * 6 / fit["limit_dps"] if fit.get("limit_dps") else 1.0)
        m["vel"] = max(-vmax, min(vmax, m["vel"] + (kp * (m["target"] - m["deg"]) - kv * m["vel"]) * dt))
        m["deg"] += m["vel"] * dt
        if m["deg"] >= OBSTACLE.get(i, 1e9):
            m["deg"], m["vel"] = OBSTACLE[i], 0.0


def command(line):
    global pc
    f = line.split()
    if rec["state"] == "play" and f and (f[0] in "xhtp" or len(f) > 1 and f[1][:1] in "arsho"):
        rec["state"] = "idle"  # a manual move takes over from the replay, as on the Teensy
    if line == "rec":
        rec.update(state="rec", no=0, take=[], t0=time.monotonic())
    elif line == "stop":
        if rec["state"] == "rec":
            rec["no"] = max(takes, default=0) + 1
            takes[rec["no"]] = rec["take"]
        rec["state"] = "idle"
    elif line.startswith("play"):
        if rec["take"]:
            rec.update(state="play", t0=time.monotonic(), speed=float(line[4:] or 100) / 100)
    elif line.startswith("load "):
        if int(line[5:]) in takes:
            rec.update(no=int(line[5:]), take=takes[int(line[5:])])
    elif line == "takes":
        for n, samples in sorted(takes.items()):
            sock.sendto(("K,%d,%.2f\n" % (n, len(samples) / 10)).encode(), pc)
        sock.sendto(("K,end,%d\n" % len(takes)).encode(), pc)
    elif line.startswith("#hello"):
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
    elif line == "mpc" or line.startswith("mpc "):  # the Teensy's replay MPC: only its state here, not its effect
        p = line.split()[1:]
        if len(p) == 3 and 0.5 <= float(p[1]) <= 150 and 0.1 <= float(p[2]) <= 4:  # the Teensy's bounds
            mpc_state["models"][int(p[0])] = (float(p[1]), float(p[2]))
        sock.sendto(("Q,on,25.0,0,%s\n" % (",".join(
            "%d:%.2f:%.2f" % (i, *mpc_state["models"].get(i, (0, 0))) for i in (1, 2, 3, 4)))).encode(), pc)
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
            for n in range(200):  # sample, then move 10 ms: the first sample is the moment of the step, as on the Teensy
                ys.append(m["deg"] + random.gauss(0, 0.01))
                for _ in range(10):
                    move(int(i), m, 0.001)
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
        if rec["state"] == "rec":
            rec["take"].append({i: m["target"] for i, m in motors.items()})
        elif rec["state"] == "play":
            k = int((now - rec["t0"]) * rec["speed"] * 10)
            if k >= len(rec["take"]):
                rec["state"] = "idle"
            else:
                for i, deg in rec["take"][k].items():
                    motors[i].update(target=deg, mode=4)
        at = len(rec["take"]) / 10 if rec["state"] == "rec" else (now - rec["t0"]) * rec["speed"] if rec["state"] == "play" else 0
        sock.sendto(("R,%s,%d,%.2f,%.2f,1\n" % (rec["state"], rec["no"], at, len(rec["take"]) / 10)).encode(), pc)
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
        # the master arm over the radio, as the follower's gateway passes it on: slow sweeps, the wrist sensor
        # unplugged, the gripper button pressed every 3 s
        sock.sendto(("M,%.2f,%.2f,%.2f,-1,%.1f,%d,7\n" % (
            180 + 60 * math.sin(now * 0.5), 90 + 40 * math.sin(now * 0.7), 200 + 30 * math.sin(now * 0.4),
            50 + 45 * math.sin(now * 0.3), int(now / 3) % 2)).encode(), pc)
        if int(now) != int(now - 0.1):
            sock.sendto(b"L,f,1,31,%d,-48,9.0,-51,8.5,433.70\n" % random.randint(94, 100), pc)
    time.sleep(0.002)

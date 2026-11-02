"""Unattended tuning session for one joint over WiFi, logged to files to analyse later (e.g. back on the internet).

Run it on the PC joined to the follower gateway's WiFi (TwinArm), with the app closed: the gateway sends the
telemetry to the last PC that talked to it. Everything lands in InterfaceApp/sessions/<time>_id<N>/:
  raw.log                     every line sent (>) and received (<), seconds since the start; notes (#)
  hold.csv                    the joint holding still before the steps: angle, target, current, temperature, 10 Hz
  setNN_steptest.csv          one row per step, in the app's whole-arm test format, so
  setNN_steptest_curves.csv     python compare.py sessions/.../set01_steptest.csv sessions/.../set02_steptest.csv
  summary.json                gain sets, metrics, model fits, verdicts, and why the session ended

python bench.py --id 2                          hold 10 s, then steps +5 -5 +10 -10 at the gains the driver holds
python bench.py --id 2 --try velkp=1.2 --try velkp=1.44,poskp=0.9   then the same steps with each of these sets
python bench.py --id 2 --auto 4                 then up to 4 rounds of tune.py's suggestions, one gain set per round
python bench.py --log-only --minutes 20         only record what the gateway sends, while you drive the arm yourself
python bench.py --host 127.0.0.1 --id 2 --yes   against python sim.py, to try it without the arm
Factors (poskp poski velkp velki) scale the gains the driver held at the start. At the end the driver gets those
starting gains back (--keep leaves the last set on it) and the speed limit the app uses, and the joint goes back to
where it was holding.

SAFETY: this moves the arm. Keep the E-stop at hand and the workspace clear. It stops, puts the starting gains
back and holds when a step overshoots more than --max-overshoot, keeps ringing, ends far off its target, fails, or
the joint jams or faults, and on Ctrl+C. A gain set more than 2x or under 0.5x the starting gains, or one with a
Position Ki above 0 (SteadyWin warns against it), is refused.
"""
import argparse
import csv
import json
import math
import os
import sys
import time

import compare
import tune
from link import Link

HERE = os.path.dirname(os.path.abspath(__file__))
IDS = (1, 2, 3, 4)
JOINT = {1: "Shoulder", 2: "Elbow", 3: "Wrist pitch", 4: "Wrist roll"}
NAMES = ("poskp", "poski", "velkp", "velki")  # tune.py's gain index order
APP = {"rpm_max": 30, "speed": 30, "roll_dps": 60, "roll_units": 8}  # main.py's defaults, settings.json wins


class Abort(Exception):
    pass


def app_settings():
    cfg = dict(APP)
    try:
        with open(os.path.join(HERE, "settings.json")) as f:
            cfg.update({k: v for k, v in json.load(f).items() if k in APP})
    except (OSError, ValueError):
        pass
    return cfg


def parse_try(s):
    """'velkp=1.2,poskp=0.9' -> {2: 1.2, 0: 0.9}"""
    out = {}
    for part in s.split(","):
        name, _, v = part.partition("=")
        if name.strip().lower() not in NAMES:
            raise argparse.ArgumentTypeError("%r: use %s" % (name, " ".join(NAMES)))
        out[NAMES.index(name.strip().lower())] = float(v)
    return out


class Bench:
    def __init__(self, a, folder):
        self.a, self.folder = a, folder
        self.t0 = time.monotonic()
        self.raw = open(os.path.join(folder, "raw.log"), "w", buffering=1, encoding="utf-8")
        self.link = Link(list(IDS), history=6000)
        handle, send = self.link.handle, self.link.send

        def logged_handle(line, t):
            self.raw.write("%.3f < %s\n" % (t - self.t0, line))
            handle(line, t)

        def logged_send(*lines):
            for ln in lines:
                self.raw.write("%.3f > %s\n" % (time.monotonic() - self.t0, ln))
            send(*lines)

        self.link.handle, self.link.send = logged_handle, logged_send
        self.hello_at = 0.0
        self.cfg = app_settings()
        self.orig = None        # gains on the driver at the start
        self.changed = False    # the driver holds other gains than orig
        self.summary = {"started": time.strftime("%Y-%m-%d %H:%M:%S"), "args": vars(a), "sets": []}

    # ---------- plumbing ----------

    def note(self, msg):
        print(msg, flush=True)
        self.raw.write("%.3f # %s\n" % (time.monotonic() - self.t0, msg))

    def pump(self, seconds):
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            self.tick()

    def tick(self):
        if time.monotonic() - self.hello_at >= 1.0:
            self.hello_at = time.monotonic()
            self.link.hello()
        self.link.poll()
        time.sleep(0.005)

    def wait(self, cond, timeout):
        end = time.monotonic() + timeout
        while time.monotonic() < end:
            self.tick()
            if cond():
                return True
        return False

    def save_summary(self):
        with open(os.path.join(self.folder, "summary.json"), "w", encoding="utf-8") as f:
            json.dump(self.summary, f, indent=2, ensure_ascii=False)

    # ---------- the joint ----------

    @property
    def m(self):
        return self.link.motors[self.a.id]

    def driver_rpm(self, rpm):
        """The wrist roll's driver counts motor rpm, before its 8:1 gear."""
        return rpm * self.cfg["roll_units"] if self.a.id == 4 else rpm

    def app_rpm(self):
        """The speed limit the app gives this driver (main.py App.rpm), to leave it as the app expects."""
        if self.a.id == 4:
            return max(0.1, self.cfg["roll_dps"] / 6 * self.cfg["roll_units"])
        return max(0.1, self.cfg["rpm_max"] * self.cfg["speed"] / 100)

    def check(self, when):
        m = self.m
        if not (m.ok and time.monotonic() - m.seen < 1.5):
            raise Abort("%s: no telemetry from motor %d" % (when, self.a.id))
        if m.jam:
            raise Abort("%s: motor %d is JAMMED at %+.2f deg" % (when, self.a.id, m.deg))
        if m.fault:
            raise Abort("%s: motor %d reports fault %s" % (when, self.a.id, m.faults))
        if m.mode != 4:
            raise Abort("%s: motor %d is not in position mode (mode %d): home it first" % (when, self.a.id, m.mode))

    def read_gains(self):
        t = time.monotonic()
        self.link.send("%d g" % self.a.id)
        if not self.wait(lambda: self.m.gains_at > t, 3) or not self.m.gains_ok:
            raise Abort("motor %d did not answer the gain read" % self.a.id)
        return list(self.m.gains)

    def write_gains(self, g):
        for attempt in range(3):
            t = time.monotonic()
            self.link.send("%d g %s" % (self.a.id, " ".join("%g" % v for v in g)))
            if self.wait(lambda: self.m.gains_at > t, 3) and self.m.gains_ok and \
                    all(abs(x - y) <= 1e-3 + 1e-3 * abs(y) for x, y in zip(self.m.gains, g)):
                self.changed = list(g) != list(self.orig)
                return
        raise Abort("motor %d did not take the gains %s" % (self.a.id, g))

    def allowed(self, g):
        """None if the set is inside the guard rails, else why not."""
        for k, (new, old) in enumerate(zip(g, self.orig)):
            if new < 0:
                return "%s below 0" % NAMES[k]
            if k == 1 and new > 0 and not self.a.allow_poski:
                return "Position Ki above 0 (SteadyWin: keep it 0; --allow-poski to try anyway)"
            if old > 0 and not 0.5 <= new / old <= 2.0:
                return "%s %.4g is more than 2x / under 0.5x the starting %.4g" % (NAMES[k], new, old)
            if old == 0 and new > 0 and k == 3 and new > 0.2 * g[2]:
                return "Velocity Ki %.4g from 0: at most 20 %% of Velocity Kp" % new
        return None

    # ---------- the session ----------

    def preflight(self):
        a = self.a
        self.note("gateway %s:%d, motor %d (%s)" % (a.host, a.port, a.id, JOINT[a.id]))
        self.link.connect(a.host, a.port)
        if not self.wait(self.link.gateway_online, 5):
            raise Abort("no answer from the gateway at %s: joined its WiFi? app closed?" % a.host)
        if not self.wait(lambda: self.m.ok and time.monotonic() - self.m.seen < 1, 5):
            raise Abort("gateway online but no telemetry from motor %d: Teensy on, homed, supply on?" % a.id)
        self.pump(1.0)
        self.check("before the tests")
        self.orig = self.read_gains()
        t = time.monotonic()
        self.link.send("j")
        self.wait(lambda: self.link.jam_at > t, 2)
        jc = self.link.jam_cfg
        m = self.m
        self.start_target = m.target
        self.summary.update(motor=JOINT[a.id], id=a.id, start_deg=m.deg, start_target=m.target, volts=m.volts,
                            temp_c=m.temp, gains_start=self.orig, jam=jc, test_rpm=a.rpm,
                            app_driver_rpm=self.app_rpm())
        self.note("motor %d at %+.2f deg (target %+.2f), %.1f V, %d C, gains pos kp %.4g ki %.4g, vel kp %.4g ki %.4g"
                  % (a.id, m.deg, m.target, m.volts, m.temp, *self.orig))
        if not jc or not jc[1]:
            self.note("WARNING: jam detection is off on the Teensy")

    def hold(self):
        if self.a.hold <= 0:
            return
        self.note("holding still for %g s" % self.a.hold)
        t = time.monotonic()
        self.pump(self.a.hold)
        rows = [h for h in self.m.hist if h[0] >= t]
        with open(os.path.join(self.folder, "hold.csv"), "w", newline="") as f:
            w = csv.writer(f)
            w.writerow(["t_s", "deg", "target_deg", "amps", "temp_c"])
            for h in rows:
                w.writerow(["%.3f" % (h[0] - t), "%.3f" % h[4], "%.3f" % h[5], "%.3f" % h[2], h[3]])
        if rows:
            degs, amps = [h[4] for h in rows], [h[2] for h in rows]
            mean = sum(degs) / len(degs)
            st = {"samples": len(rows), "deg_mean": mean, "deg_pp": max(degs) - min(degs),
                  "deg_std": math.sqrt(sum((d - mean) ** 2 for d in degs) / len(degs)),
                  "amps_mean": sum(amps) / len(amps), "amps_pp": max(amps) - min(amps),
                  "off_target": mean - rows[-1][5]}
            self.summary["hold"] = st
            self.note("hold: %d samples, angle wanders %.3f deg p-p (std %.3f), %.2f A mean, %.2f A p-p, "
                      "%.3f deg off target" % (st["samples"], st["deg_pp"], st["deg_std"], st["amps_mean"],
                                               st["amps_pp"], st["off_target"]))
        self.check("after holding")

    def step(self, deg):
        cid = self.a.id
        self.check("before a step")
        self.link.captures.pop(cid, None)
        self.link.send("%d v%.2f" % (cid, self.driver_rpm(self.a.rpm)), "%d s%g" % (cid, deg))

        def done():
            cap = self.link.captures.get(cid)
            if cap and not cap["done"] and cap["samples"] and time.monotonic() - cap["last"] > 0.6:
                cap.update(done=True)  # its end line got lost: use what arrived
            return cap and cap["done"]

        if not self.wait(done, 6):
            raise Abort("no step data came back for %+g deg" % deg)
        cap = self.link.captures[cid]
        if cap["fail"] or not cap["samples"]:
            raise Abort("the Teensy did not take the %+g deg step" % deg)
        y = [cap["samples"].get(i, float("nan")) for i in range(max(cap["samples"]) + 1)]
        met = tune.metrics(y, cap["dt"], cap["start"], cap["target"], self.a.rpm)
        hs = tune.hints(met)
        fit = compare.fit(y, cap["start"], cap["target"])
        last = next((v for v in reversed(y) if not math.isnan(v)), float("nan"))
        self.pump(self.a.pause)  # telemetry is back, the joint settles before the next step
        return {"step": deg, "start": cap["start"], "target": cap["target"], "dt": cap["dt"], "curve": y,
                "end_error": last - cap["target"], "metrics": met, "hints": hs, "fit": fit,
                "missing": sum(1 for v in y if math.isnan(v))}

    def unsafe(self, r):
        met, deg = r["metrics"], r["step"]
        if met is None:
            return "too little data in the %+g deg step" % deg
        if met["overshoot"] * 100 > self.a.max_overshoot:
            return "%+g deg step overshot %.0f %% (limit %g %%)" % (deg, 100 * met["overshoot"], self.a.max_overshoot)
        if met["swings"] >= 3:
            return "%+g deg step rang: %d swings past the target" % (deg, met["swings"])
        if abs(r["end_error"]) > max(2.0, 0.3 * abs(deg)):
            return "%+g deg step ended %.2f deg off its target" % (deg, r["end_error"])
        return None

    def run_set(self, n, gains, label):
        why = self.allowed(gains)
        entry = {"set": n, "label": label, "gains": gains, "steps": []}
        self.summary["sets"].append(entry)
        if why:
            entry["refused"] = why
            self.note("set %d (%s) refused: %s" % (n, label, why))
            self.save_summary()
            return None
        self.note("set %d (%s): pos kp %.4g ki %.4g, vel kp %.4g ki %.4g" % (n, label, *gains))
        if list(gains) != list(self.m.gains or ()):
            self.write_gains(gains)
            self.pump(0.5)
        results = []
        try:
            for deg in self.a.steps:
                r = self.step(deg)
                results.append(r)
                met = r["metrics"]
                self.note("  %+g deg: %s  ->  %s" % (deg, describe(met, r["fit"]), r["hints"][0][0]))
                bad = self.unsafe(r)
                if bad:
                    raise Abort(bad)
        finally:
            entry["steps"] = [{k: v for k, v in r.items() if k != "curve"} for r in results]
            self.write_csv(n, gains, results)
            self.save_summary()
        return results

    def write_csv(self, n, gains, results):
        if not results:
            return
        path = os.path.join(self.folder, "set%02d_steptest.csv" % n)
        fmt = lambda v, f="%.3f": "" if v is None else f % v
        motor = JOINT[self.a.id]
        with open(path, "w", newline="", encoding="utf-8") as f:
            w = csv.writer(f)
            w.writerow(["motor", "id", "step_deg", "start_deg", "target_deg", "speed_pct", "overshoot_pct", "rise_s",
                        "settle_s", "steady_error_deg", "peak_speed_dps", "speed_limit_dps", "pos_kp", "pos_ki",
                        "vel_kp", "vel_ki", "fit_wn", "fit_zeta", "fit_vmax_dps", "fit_rms_deg", "verdict"])
            for r in results:
                m, fi = r["metrics"] or {}, r["fit"] or (None,) * 4
                w.writerow([motor, self.a.id, r["step"], fmt(r["start"]), fmt(r["target"]), "",
                            fmt(100 * m["overshoot"], "%.1f") if m else "", fmt(m.get("rise")), fmt(m.get("settle")),
                            fmt(m.get("sse")), fmt(m.get("speed"), "%.1f"), fmt(m.get("limit"), "%.1f")] +
                           ["%g" % v for v in gains] +
                           [fmt(fi[0], "%.2f"), fmt(fi[1]), fmt(fi[2], "%.1f"), fmt(fi[3])] + [r["hints"][0][0]])
        with open(path.replace(".csv", "_curves.csv"), "w", newline="") as f:
            w = csv.writer(f)
            w.writerow(["motor", "id", "step_deg", "t_s", "deg"])
            for r in results:
                for i, y in enumerate(r["curve"]):
                    w.writerow([motor, self.a.id, r["step"], "%.2f" % (i * r["dt"]), "%.3f" % y])

    def next_auto(self, gains, results):
        """tune.py's suggestion for the next round, from the step that behaved worst; None when nothing to change."""
        worst = max((r for r in results if r["metrics"]), key=lambda r: r["metrics"]["overshoot"], default=None)
        changes = next((ch for r in ([worst] if worst else []) + results for _, ch in r["hints"] if ch), None)
        if not changes:
            return None
        g = list(gains)
        for k, factor in changes.items():
            g[k] = 0.1 * g[2] if k == 3 and g[k] == 0 else g[k] * factor  # no velocity Ki yet: 10 % of Kp
        return [round(v, 5) for v in g]

    def tests(self):
        sets = [(self.orig, "starting gains")]
        sets += [([g * f.get(k, 1.0) for k, g in enumerate(self.orig)], "try " + ",".join(
            "%s=%g" % (NAMES[k], v) for k, v in f.items())) for f in self.a.tries]
        n, gains, results = 0, None, None
        for gains, label in sets:
            n += 1
            results = self.run_set(n, gains, label)
        for r in range(self.a.auto):
            if results is None:
                break
            nxt = self.next_auto(gains, results)
            if nxt is None:
                self.note("auto: tune.py has nothing more to change")
                break
            n += 1
            gains, results = nxt, self.run_set(n, nxt, "auto round %d" % (r + 1))

    def finish(self, ok):
        """Starting gains, the app's speed limit, back to where it was holding. Runs on every ending."""
        cid = self.a.id
        try:
            if self.changed and not (ok and self.a.keep):
                self.write_gains(self.orig)
                self.note("starting gains back on motor %d" % cid)
            self.link.send("%d v%.2f" % (cid, self.app_rpm()))
            self.pump(1.0)
            m = self.m
            if ok and m.ok and not m.jam and abs(m.deg - self.start_target) > 0.5:
                self.link.send("%d a%.2f" % (cid, self.start_target))
                self.note("back to %+.2f deg, where it was holding" % self.start_target)
                self.pump(2.0)
        except Abort as e:
            self.note("COULD NOT RESTORE: %s. Set the gains back by hand: %s" % (e, self.orig))
            self.summary["restore_failed"] = str(e)

    def log_only(self):
        self.note("gateway %s:%d, recording for %g min (Ctrl+C ends it)" % (self.a.host, self.a.port, self.a.minutes))
        self.link.connect(self.a.host, self.a.port)
        end, shown = time.monotonic() + 60 * self.a.minutes, 0.0
        while time.monotonic() < end:
            self.tick()
            if time.monotonic() - shown >= 10:
                shown = time.monotonic()
                online = [i for i in IDS if self.link.motors[i].ok and time.monotonic() - self.link.motors[i].seen < 1]
                self.note("gateway %s, motors online %s, %.0f Hz telemetry" % (
                    "online" if self.link.gateway_online() else "OFFLINE", online or "none", self.link.rate()))


def describe(m, fit):
    if not m:
        return "no metrics"
    num = lambda v, f: f % v if v is not None else "-"
    s = "overshoot %.0f %%, rise %s, settles %s, off %.2f deg, peak %.0f/%.0f deg/s" % (
        100 * m["overshoot"], num(m["rise"], "%.2f s"), num(m["settle"], "%.2f s"), m["sse"], m["speed"],
        m["limit"] or 0)
    if fit:
        s += ", model wn %.1f zeta %.2f" % fit[:2]
    return s


def main(argv):
    if hasattr(sys.stdout, "reconfigure"):
        sys.stdout.reconfigure(errors="replace")  # an old Windows console cannot print every hint
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--host", default="192.168.4.1")
    p.add_argument("--port", type=int, default=4210)
    p.add_argument("--id", type=int, choices=IDS, help="CAN id of the joint to test")
    p.add_argument("--steps", default="5,-5,10,-10", help="step sizes, deg, in order (each +d is best followed by -d)")
    p.add_argument("--rpm", type=float, default=10, help="output rpm limit during the steps (default 10 = 60 deg/s)")
    p.add_argument("--hold", type=float, default=10, help="seconds of holding still before the steps")
    p.add_argument("--pause", type=float, default=1.5, help="seconds between steps")
    p.add_argument("--try", dest="tries", type=parse_try, action="append", default=[], help="gain factors, e.g. velkp=1.2")
    p.add_argument("--auto", type=int, default=0, help="rounds of tune.py suggestions after the sets")
    p.add_argument("--max-overshoot", type=float, default=40, help="%% overshoot that stops the session")
    p.add_argument("--allow-poski", action="store_true", help="allow Position Ki above 0")
    p.add_argument("--keep", action="store_true", help="leave the last gain set on the driver")
    p.add_argument("--log-only", action="store_true", help="only record, no commands except #hello")
    p.add_argument("--minutes", type=float, default=15, help="with --log-only")
    p.add_argument("--yes", action="store_true", help="start without asking")
    a = p.parse_args(argv)
    try:
        a.steps = [float(s) for s in a.steps.split(",") if s.strip()]
    except ValueError:
        p.error("--steps: numbers separated by commas")
    if not a.log_only and a.id is None:
        p.error("--id is needed (or --log-only)")
    if not a.log_only and any(abs(s) > 30 for s in a.steps):
        p.error("--steps: at most 30 deg each")

    stamp = time.strftime("%Y%m%d_%H%M%S")
    folder = os.path.join(HERE, "sessions", "%s_%s" % (stamp, "log" if a.log_only else "id%d" % a.id))
    if not a.log_only and not a.yes:
        print("Motor %d (%s): hold %g s, steps %s at %g rpm, %d extra gain set(s), %d auto round(s).\n"
              "THE ARM WILL MOVE. E-stop at hand, workspace clear, app closed." % (
                  a.id, JOINT[a.id], a.hold, " ".join("%+g" % s for s in a.steps), a.rpm, len(a.tries), a.auto))
        if input("Type yes to start: ").strip().lower() != "yes":
            print("Not started")
            return 1
    os.makedirs(folder, exist_ok=True)
    b = Bench(a, folder)
    ok, reason = False, "done"
    try:
        if a.log_only:
            b.log_only()
        else:
            b.preflight()
            b.hold()
            b.tests()
        ok = True
    except Abort as e:
        reason = str(e)
        b.note("STOPPED: %s" % reason)
    except KeyboardInterrupt:
        reason = "Ctrl+C"
        b.note("STOPPED: Ctrl+C")
    finally:
        if not a.log_only and b.orig is not None:
            b.finish(ok)
        b.summary.update(ended=time.strftime("%Y-%m-%d %H:%M:%S"), result="ok" if ok else "stopped", reason=reason)
        b.save_summary()
        b.link.close()
        b.raw.close()
        print("files in %s" % folder)
    return 0 if ok else 2


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))

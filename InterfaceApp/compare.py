"""Read whole-arm step tests, fit the simulator's joint model to them, and compare two tests side by side.

python compare.py step_tests/steptest_A.csv                   metrics + model fit per step, plot, model_fit.json
python compare.py step_tests/steptest_A.csv step_tests/steptest_B.csv   B next to A (e.g. the simulator vs the arm)
python sim.py --model step_tests/model_fit.json               the simulator then moves like the fitted arm

The model is the simulator's joint: acceleration = kp * error - kv * speed, speed capped at vmax. Reported as
natural frequency wn = sqrt(kp) (rad/s), damping zeta = kv / (2 wn) and top speed vmax (deg/s).
"""
import csv
import json
import math
import os
import sys

import tune

DT = 0.01  # the Teensy samples a step every 10 ms


def load(path):
    """Rows of a steptest_*.csv, each with its curve from the matching _curves.csv."""
    with open(path, newline="") as f:
        rows = list(csv.DictReader(f))
    curves = {}
    with open(path.replace(".csv", "_curves.csv"), newline="") as f:
        for r in csv.DictReader(f):
            curves.setdefault((r["id"], r["step_deg"]), []).append(float(r["deg"]))
    for r in rows:
        r["curve"] = curves.get((r["id"], r["step_deg"]), [])
        r["start"] = float(r["start_deg"]) if r.get("start_deg") else (r["curve"][0] if r["curve"] else 0.0)
        r["target"] = float(r["target_deg"]) if r.get("target_deg") else r["start"] + float(r["step_deg"])
    return rows


def simulate(kp, kv, vmax, start, target, n, sub=5):
    """The simulator's joint, sampled like the Teensy: first sample at the moment of the step."""
    y, v, out, h = start, 0.0, [], DT / sub
    for _ in range(n):
        out.append(y)
        for _ in range(sub):
            v = max(-vmax, min(vmax, v + (kp * (target - y) - kv * v) * h))
            y += v * h
    return out


def fit(curve, start, target):
    """(wn, zeta, vmax, rms deg) of the model closest to the curve: a coarse grid, then a shrinking local search.
    vmax is None when the curve never reached a speed cap (so no cap can be read from it)."""
    ys = [y for y in curve if not math.isnan(y)]
    if len(ys) < 20 or abs(target - start) < 0.5:
        return None
    peak = max(abs(b - a) for a, b in zip(ys, ys[3:])) / (3 * DT)  # deg/s over 30 ms

    def err(wn, zeta, vmax):
        sim = simulate(wn * wn, 2 * zeta * wn, vmax, start, target, len(curve))
        return sum((s - y) ** 2 for s, y in zip(sim, curve) if not math.isnan(y))

    def search(caps):
        best = min(((err(wn, z, vm), wn, z, vm)
                    for wn in (1.5 * 1.35 ** k for k in range(12))
                    for z in (0.15, 0.3, 0.45, 0.6, 0.75, 0.9, 1.1, 1.4, 1.8)
                    for vm in caps), key=lambda b: b[0])
        step = 0.3
        params = (1, 2, 3) if caps[0] < 1e9 else (1, 2)
        moves = [((i,), (s,)) for i in params for s in (1, -1)] + [((1, 2), s) for s in ((1, 1), (1, -1), (-1, 1), (-1, -1))]
        while step > 0.002:  # multiplicative steps, also on wn and zeta together (they trade off), keep any that helps
            moved = False
            for idx, signs in moves:
                cand = list(best)
                for i, s in zip(idx, signs):
                    cand[i] *= (1 + step) ** s
                e = err(*cand[1:])
                if e < best[0]:
                    best, moved = [e] + cand[1:], True
            if not moved:
                step /= 2
        return best

    n = sum(1 for y in curve if not math.isnan(y))
    free = search([1e9])                                       # no speed cap at all
    capped = search([peak * f for f in (0.9, 1.1, 1.5, 3.0)])  # with a cap near the peak speed
    if capped[0] < 0.8 * free[0] - n * 0.01 ** 2:               # the cap explains clearly more than encoder noise
        return capped[1], capped[2], capped[3], math.sqrt(capped[0] / n)
    return free[1], free[2], None, math.sqrt(free[0] / n)


def describe(r):
    m = tune.metrics(r["curve"], DT, r["start"], r["target"])
    f = fit(r["curve"], r["start"], r["target"])
    return m, f


def table(a, b=None):
    """One line per step: metrics and fitted model of a (and of b, matched by motor and step)."""
    by_key = {(r["id"], r["step_deg"]): r for r in b or []}
    head = "%-12s %5s | %6s %6s %7s %6s | %6s %5s %6s %5s" % (
        "motor", "step", "over%", "rise", "settle", "off°", "wn", "zeta", "vmax", "rms°")
    print(head + ("   ||  %6s %6s %7s %6s | %6s %5s %6s" % ("over%", "rise", "settle", "off°", "wn", "zeta", "vmax")
                  if b else ""))
    print("-" * len(head) * (2 if b else 1))
    fits = {}
    for r in a:
        m, f = describe(r)
        fits.setdefault(r["id"], []).append((r, f))
        line = "%-12s %+5g | %s" % (r["motor"], float(r["step_deg"]), fmt(m, f))
        other = by_key.get((r["id"], r["step_deg"]))
        if other:
            line += "   ||  " + fmt(*describe(other), short=True)
        print(line)
    return fits


def fmt(m, f, short=False):
    num = lambda v, p="%.2f": p % v if v is not None else "  –  "
    s = "%6s %6s %7s %6s" % (num(m and 100 * m["overshoot"], "%.0f"), num(m and m["rise"]), num(m and m["settle"]),
                             num(m and m["sse"])) if m else "%6s %6s %7s %6s" % (("–",) * 4)
    s += " | %6s %5s %6s" % ((num(f[0], "%.1f"), num(f[1]), num(f[2], "%.0f") if f[2] else "free") if f else ("–",) * 3)
    if not short:
        s += " %5s" % (num(f[3]) if f else "–")
    return s


def save_model(fits, rows, path):
    """Per motor: the mean fitted model, and the driver gains and speed limit it was measured with."""
    model = {}
    for cid, items in fits.items():
        good = [f for _, f in items if f]
        if not good:
            continue
        r = items[0][0]
        wn = sum(f[0] for f in good) / len(good)
        zeta = sum(f[1] for f in good) / len(good)
        gains = [float(r[k]) for k in ("pos_kp", "pos_ki", "vel_kp", "vel_ki")] if r.get("pos_kp") else None
        caps = [f[2] for f in good if f[2]]
        model[cid] = {"motor": r["motor"], "kp": wn * wn, "kv": 2 * zeta * wn, "wn": wn, "zeta": zeta,
                      "vmax": sum(caps) / len(caps) if caps else None, "gains": gains,  # None: use the speed limit
                      "limit_dps": float(r["speed_limit_dps"]) if r.get("speed_limit_dps") else None}
    with open(path, "w") as f:
        json.dump(model, f, indent=2)
    print("\nmodel saved to %s: python sim.py --model %s" % (path, path))


def plot(a, b, path, la, lb):
    try:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
    except ImportError:
        print("(no matplotlib: pip install matplotlib for the plot)")
        return
    ids = list(dict.fromkeys(r["id"] for r in a))
    fig, axes = plt.subplots(len(ids), 2, figsize=(12, 2.6 * len(ids)), squeeze=False)
    by_key = {(r["id"], r["step_deg"]): r for r in b or []}
    for r in a:
        ax = axes[ids.index(r["id"])][0 if float(r["step_deg"]) > 0 else 1]
        t = [i * DT for i in range(len(r["curve"]))]
        ax.plot(t, r["curve"], color="#A4161A", lw=2, label=la)
        f = fit(r["curve"], r["start"], r["target"])
        if f:
            ax.plot(t, simulate(f[0] ** 2, 2 * f[1] * f[0], f[2] or 1e9, r["start"], r["target"], len(t)),
                    "--", color="#555555", lw=1.2, label="fitted model")
        other = by_key.get((r["id"], r["step_deg"]))
        if other:
            ax.plot([i * DT for i in range(len(other["curve"]))],
                    [y - other["start"] + r["start"] for y in other["curve"]], ":", color="#1D6FA5", lw=2, label=lb)
        ax.axhline(r["target"], color="#999999", lw=0.8)
        ax.set_title("%s  %+g°" % (r["motor"], float(r["step_deg"])), fontsize=10)
        ax.set_xlabel("s")
        ax.set_ylabel("°")
        ax.legend(fontsize=8, loc="lower right" if float(r["step_deg"]) > 0 else "upper right")
    fig.tight_layout()
    fig.savefig(path, dpi=110)
    print("plot saved to %s" % path)


def main(argv):
    if len(argv) < 2:
        print(__doc__)
        return
    a = load(argv[1])
    b = load(argv[2]) if len(argv) > 2 else None
    print("A = %s%s\n" % (argv[1], "\nB = %s" % argv[2] if b else ""))
    fits = table(a, b)
    base = os.path.splitext(argv[1])[0]
    save_model(fits, a, os.path.join(os.path.dirname(argv[1]) or ".", "model_fit.json"))
    plot(a, b, base + "_compare.png", os.path.basename(argv[1]), os.path.basename(argv[2]) if b else "")


if __name__ == "__main__":
    if sys.argv[1:] == ["--selftest"]:
        for wn, zeta, vmax in ((6.0, 0.3, 200.0), (12.0, 0.8, 30.0), (3.0, 1.2, 500.0)):
            curve = simulate(wn * wn, 2 * zeta * wn, vmax, 10.0, 25.0, 200)
            fw, fz, fv, rms = fit(curve, 10.0, 25.0)
            assert rms < 0.05, (wn, zeta, vmax, rms)
            if vmax < 100:  # mostly at its cap: the cap is measured, wn and zeta barely show in the curve
                assert fv and abs(fv - vmax) / vmax < 0.05, (vmax, fv)
            else:  # never near its cap: no cap to report
                assert fv is None and abs(fw - wn) / wn < 0.05 and abs(fz - zeta) / zeta < 0.08, (wn, zeta, fw, fz, fv)
        print("compare ok")
    else:
        main(sys.argv)

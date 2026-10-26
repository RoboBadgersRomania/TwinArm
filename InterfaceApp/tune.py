"""Step-response metrics and tuning hints for one driver (position P/PI loop feeding a velocity PI loop).

The hints are rules of thumb that move one or two gains by 20-50 % per round: apply, step again, repeat.
Gain index: 0 position Kp, 1 position Ki, 2 velocity Kp, 3 velocity Ki.
"""
import math


def metrics(samples, dt, start, target, rpm=None):
    """samples: joint angle (deg) every dt s after the step, NaN where a read failed. None if too little to judge."""
    step = target - start
    pts = [(i * dt, (y - start) / step) for i, y in enumerate(samples) if not math.isnan(y)] if abs(step) >= 0.5 else []
    if len(pts) < 20:
        return None
    t10 = next((t for t, r in pts if r >= 0.1), None)
    t90 = next((t for t, r in pts if r >= 0.9), None)
    band = max(0.05, 0.2 / abs(step))  # settled = inside 5 % of the step, or 0.2 deg on small steps
    last_out = max((t for t, r in pts if abs(1 - r) > band), default=None)
    tail = [r for t, r in pts if t >= 0.8 * pts[-1][0]]
    runs, sign = [], 0  # largest error of each swing past the target, after first reaching 90 %
    for e in (1 - r for t, r in pts if t90 is not None and t >= t90):
        s = (e > 0) - (e < 0)
        if s and s != sign:
            runs.append(0.0)
            sign = s
        if runs:
            runs[-1] = max(runs[-1], abs(e))
    k = 3  # speed over 3 samples (30 ms at 100 Hz): single-sample differences mostly show encoder noise
    vel = [((pts[i + k][1] - pts[i][1]) / (pts[i + k][0] - pts[i][0]), pts[i][0]) for i in range(len(pts) - k)]
    speed = max(abs(v) for v, _ in vel) * abs(step)  # deg/s
    plateau = None  # share of the rise spent near the peak speed: >= 0.7 when capped, <= 0.53 for a loop under 5 % overshoot
    if t10 is not None:
        end = t90 if t90 is not None else pts[-1][0]
        inside = [abs(v) * abs(step) for v, t in vel if t10 <= t <= end]
        if end - t10 >= 0.1 and inside:
            plateau = sum(1 for v in inside if v >= 0.85 * speed) / len(inside)
    return {
        "step": step,
        "overshoot": max(0.0, max(r for _, r in pts) - 1),
        "rise": t90 - t10 if t90 is not None and t10 is not None else None,
        "settle": None if last_out is not None and last_out >= pts[-1][0] else (last_out or -dt) + dt,
        "sse": abs(1 - sum(tail) / len(tail)) * abs(step),
        "swings": sum(1 for a in runs[1:] if a > band),  # runs[0] is the approach from 90 %, not a swing
        "speed": speed,
        "limit": rpm * 6 if rpm is not None else None,  # deg/s the drivers were told
        "plateau": plateau,
        "limited": rpm is not None and speed >= 0.8 * rpm * 6,
    }


def hints(m):
    """[(text, {gain index: factor})]; an empty dict is advice only."""
    if m is None:
        return [("Not enough data: run a step test of at least 1°", {})]
    out = []
    os_, rise = m["overshoot"], m["rise"]
    capped = (not m["limited"] and m["limit"] and m["speed"] < 0.7 * m["limit"] and m["plateau"] is not None
              and m["plateau"] > 0.6 and os_ < 0.05)  # cruising well under the limit: not the gains
    if capped:
        out.append(("Moves at a steady %.0f°/s, far under its %.0f°/s limit: something else caps the speed "
                    "(the driver's own max speed or current limit), not these gains" % (m["speed"], m["limit"]), {}))
    elif rise is None:
        out.append(("Never reached 90 % of the step: raise Speed % or take a smaller step", {}) if m["limited"]
                   else ("Never reached 90 % of the step: Position Kp ×1.5", {0: 1.5}))
    elif os_ > 0.25 or m["swings"] >= 3:
        out.append(("Overshoots %.0f %% and rings: Position Kp ×0.8, Velocity Kp ×1.3" % (100 * os_), {0: 0.8, 2: 1.3}))
    elif os_ > 0.10:
        out.append(("Overshoots %.0f %%: Velocity Kp ×1.2 for more damping" % (100 * os_), {2: 1.2}))
    elif os_ < 0.02 and not m["limited"] and rise > 0.4:
        out.append(("Sluggish, %.2f s to rise: Position Kp ×1.25" % rise, {0: 1.25}))
    if m["sse"] > max(0.3, 0.03 * abs(m["step"])):
        out.append(("Stays %.2f° off the target: Velocity Ki ×1.5 (10 %% of Velocity Kp if it is 0)" % m["sse"], {3: 1.5}))
    if not capped and not any(changes for _, changes in out) and rise is not None and m["settle"] is not None:
        out.insert(0, ("Looks well tuned: %.0f %% overshoot, settles in %.2f s" % (100 * os_, m["settle"]), {}))
    if m["limited"] and rise is not None:
        out.append(("The rise is set by the speed limit (Speed %), not by the gains", {}))
    return out


if __name__ == "__main__":
    def second_order(zeta, wn, step=10.0, gain=1.0, dt=0.01, n=200):
        """Textbook step response, start 0, target `step`; gain < 1 leaves a steady error."""
        out = []
        for i in range(n):
            t = i * dt
            if zeta < 1:
                wd = wn * math.sqrt(1 - zeta * zeta)
                r = 1 - math.exp(-zeta * wn * t) * (math.cos(wd * t) + zeta / math.sqrt(1 - zeta * zeta) * math.sin(wd * t))
            else:
                r = 1 - math.exp(-wn * t) * (1 + wn * t)
            out.append(step * gain * r)
        return out

    m = metrics(second_order(0.2, 12), 0.01, 0, 10)
    assert 0.5 < m["overshoot"] < 0.56 and hints(m)[0][1] == {0: 0.8, 2: 1.3}, m  # textbook 53 %
    m = metrics(second_order(0.5, 12), 0.01, 0, 10)
    assert 0.15 < m["overshoot"] < 0.18 and hints(m)[0][1] == {2: 1.2}, m          # textbook 16 %
    m = metrics(second_order(1.0, 3), 0.01, 0, 10)
    assert m["overshoot"] < 0.01 and m["rise"] > 0.4 and hints(m)[0][1] == {0: 1.25}, m
    m = metrics(second_order(0.8, 15), 0.01, 0, 10)
    assert hints(m)[0][0].startswith("Looks well tuned") and m["settle"] < 0.6, (m, hints(m))
    m = metrics(second_order(0.8, 15, gain=0.9), 0.01, 0, 10)
    assert any(ch == {3: 1.5} for _, ch in hints(m)) and abs(m["sse"] - 1) < 0.05, m
    m = metrics(second_order(0.8, 15, step=-10), 0.01, 0, -10)                     # negative steps read the same
    assert hints(m)[0][0].startswith("Looks well tuned"), hints(m)
    m = metrics(second_order(0.8, 15), 0.01, 0, 10, rpm=10)                         # 60 deg/s cap: rise is speed-bound
    assert m["limited"] and "speed limit" in hints(m)[-1][0]
    assert metrics([0.0] * 5, 0.01, 0, 10) is None and hints(None)[0][1] == {}
    slow = [min(10.0, 7.0 * i * 0.01) for i in range(200)]                           # straight 7 deg/s ramp, limit 54
    m = metrics(slow, 0.01, 0, 10, rpm=9)
    assert abs(m["speed"] - 7) < 0.1 and "something else caps" in hints(m)[0][0] and not any(c for _, c in hints(m)), hints(m)
    for zeta in (0.2, 0.5, 0.7, 0.8, 0.9, 1.0):                                       # real loop shapes are not "capped"
        for wn in (3, 6, 15, 30):
            assert "caps the speed" not in hints(metrics(second_order(zeta, wn), 0.01, 0, 10, rpm=30))[0][0], (zeta, wn)

    def capped_spring(step, vmax, kp=20.0, kv=8.0):
        """A loop whose speed is clamped inside the driver: cruises at vmax, then eases in."""
        y = v = 0.0
        out = []
        for _ in range(200):
            for _ in range(10):
                v = max(-vmax, min(vmax, v + (kp * (step - y) - kv * v) * 0.001))
                y += v * 0.001
            out.append(y)
        return out
    for step, vmax in ((15, 15), (30, 15), (10, 5)):
        m = metrics(capped_spring(step, vmax), 0.01, 0, step, rpm=9)
        assert abs(m["speed"] - vmax) < 0.5 and "something else caps" in hints(m)[0][0], (step, vmax, m, hints(m))
    noisy = [v if i % 7 else float("nan") for i, v in enumerate(second_order(0.5, 12))]  # failed reads are skipped
    assert hints(metrics(noisy, 0.01, 0, 10))[0][1] == {2: 1.2}
    print("tune ok")

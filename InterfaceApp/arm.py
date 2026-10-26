"""Side-view kinematics of the three pitch joints: shoulder, elbow, wrist.

Angles in degrees, lengths in mm, x forward and y up from the shoulder axis.
g = joint angles as drawn: g[0] is the upper arm from the horizontal, g[1] and g[2] are each relative
to the previous link, counter-clockwise positive. The tool angle is their sum.
"""
from math import acos, atan2, cos, degrees, hypot, pi, radians, sin

MAX_JUMP = 30.0  # deg: a closed-form answer further than this from the current pose is a branch flip, not a step


def wrap(a):
    return (a + 180) % 360 - 180


def fk(g, L):
    """Positions of shoulder, elbow, wrist and tool tip."""
    pts, a = [(0.0, 0.0)], 0.0
    for gi, li in zip(g, L):
        a += gi
        x, y = pts[-1]
        pts.append((x + li * cos(radians(a)), y + li * sin(radians(a))))
    return pts


def closed_form(tip, tool, L):
    """Both elbow branches that put the tip exactly on `tip` with the tool at `tool`; [] when out of reach."""
    L1, L2, L3 = L
    wx, wy = tip[0] - L3 * cos(radians(tool)), tip[1] - L3 * sin(radians(tool))
    r = hypot(wx, wy)
    if not abs(L1 - L2) <= r <= L1 + L2:
        return []
    q = acos(max(-1.0, min(1.0, (r * r - L1 * L1 - L2 * L2) / (2 * L1 * L2))))
    out = []
    for q2 in (-q, q):  # elbow up (as in the idle pose), elbow down
        q1 = atan2(wy, wx) - atan2(L2 * sin(q2), L1 + L2 * cos(q2))
        out.append([degrees(q1), degrees(q2), tool - degrees(q1) - degrees(q2)])
    return out


def solve3(A, b):
    """Cramer's rule; A here is J J^T + damping, always invertible."""
    det = lambda m: (m[0][0] * (m[1][1] * m[2][2] - m[1][2] * m[2][1]) - m[0][1] * (m[1][0] * m[2][2] - m[1][2] * m[2][0])
                     + m[0][2] * (m[1][0] * m[2][1] - m[1][1] * m[2][0]))
    d = det(A)
    return [det([[b[r] if c == k else A[r][c] for c in range(3)] for r in range(3)]) / d for k in range(3)]


def dls(tip, tool, L, start, limits, tool_weight, steps=200, damping=20.0):
    """Damped least squares from `start` toward the target, kept inside `limits` after every step.

    Task = (x, y, tool_weight * tool angle in rad), so tool_weight is how many mm one radian of tool error is worth.
    """
    L1, L2, L3 = L
    g = [radians(v) for v in start]
    lo, hi = [radians(a) for a, _ in limits], [radians(b) for _, b in limits]
    for _ in range(steps):
        a1, a2 = g[0], g[0] + g[1]
        a3 = a2 + g[2]
        px = L1 * cos(a1) + L2 * cos(a2) + L3 * cos(a3)
        py = L1 * sin(a1) + L2 * sin(a2) + L3 * sin(a3)
        e = [tip[0] - px, tip[1] - py, tool_weight * ((radians(tool) - a3 + pi) % (2 * pi) - pi)]
        if max(map(abs, e)) < 0.05:
            break
        J = [[-py, -(L2 * sin(a2) + L3 * sin(a3)), -L3 * sin(a3)],
             [px, L2 * cos(a2) + L3 * cos(a3), L3 * cos(a3)],
             [tool_weight] * 3]
        A = [[sum(J[i][k] * J[j][k] for k in range(3)) + (damping ** 2 if i == j else 0) for j in range(3)] for i in range(3)]
        y = solve3(A, e)
        dq = [sum(J[i][k] * y[i] for i in range(3)) for k in range(3)]
        n = max(map(abs, dq))
        if n > 0.1:  # at most ~6 deg per step keeps it on one branch
            dq = [v * 0.1 / n for v in dq]
        g = [min(max(g[k] + dq[k], lo[k]), hi[k]) for k in range(3)]
    return [degrees(v) for v in g]


def ik(tip, tool, L, limits, near):
    """Joint angles for the tip at `tip` and the tool at `tool`, inside `limits` [(lo, hi)] x3. Returns (g, exact).

    exact: the closed-form branch inside the limits nearest to `near` (the current pose), if no joint has to jump.
    Otherwise damped least squares walks from `near` to the closest pose it can reach; the tool angle gives way first.
    """
    best = None
    for g in closed_form(tip, tool, L):
        g = [(lo + hi) / 2 + wrap(v - (lo + hi) / 2) for v, (lo, hi) in zip(g, limits)]  # the turn inside the range
        step = max(abs(a - b) for a, b in zip(g, near))
        if all(lo <= v <= hi for v, (lo, hi) in zip(g, limits)) and step <= MAX_JUMP and (best is None or step < best[0]):
            best = (step, g)
    if best:
        return best[1], True
    return dls(tip, tool, L, near, limits, tool_weight=0.3 * L[2]), False


if __name__ == "__main__":
    L = (300, 300, 142)
    lim = [(z - 150, z + 150) for z in (80, -132, 52)]  # the app's defaults: +-150 deg around the idle pose
    close = lambda a, b, tol=1e-6: all(abs(x - y) < tol for x, y in zip(a, b))
    for g in ([80, -132, 52], [45, -90, 30], [10, -20, -60], [60, 10, -60]):  # last one is elbow down
        got, exact = ik(fk(g, L)[-1], sum(g), L, lim, g)
        assert exact and close(got, g), (g, got)

    g0 = [80, -132, 52]
    got, exact = ik((5000, 0), 0, L, lim, g0)  # out of reach: stretches toward it, inside the limits
    assert not exact and all(lo <= v <= hi for v, (lo, hi) in zip(got, lim)) and hypot(*fk(got, L)[-1]) > 700, got

    tight = [(70, 90), (-180, 0), (-120, 120)]  # shoulder may not lean back: a target behind can't be met
    got, exact = ik((-300, 200), 180, L, tight, g0)
    tip = fk(got, L)[-1]
    assert not exact and 70 <= got[0] <= 90 and hypot(tip[0] + 300, tip[1] - 200) < hypot(387 + 300, 65 - 200), got

    flip = closed_form(fk(g0, L)[-1], 0, L)[1]  # the elbow-down twin of the idle pose is a big jump: not taken
    got, exact = ik(fk(g0, L)[-1], 0, L, [(-180, 180), (-180, 180), (-180, 180)], flip)
    assert exact and close(got, flip), got
    got, exact = ik(fk(g0, L)[-1], 0, L, [(-180, 180), (-180, 180), (-180, 180)], [70, -120, 50])
    assert exact and close(got, g0), got
    print("arm ok")

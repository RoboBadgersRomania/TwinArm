"""Outer-loop model predictive control for one CAN joint: the reference for ArduinoCode/.../motorcontroller/JointMPC.h.

The SteadyWin driver keeps its own position and velocity loops. Seen from the Teensy, a joint is the closed loop
the step tests fit (compare.py): acceleration = kp * (setpoint - angle) - kv * speed, kp = wn^2, kv = 2 zeta wn.
Every tick (dt = 20 ms, the replay rate) the MPC picks the next N setpoints so the predicted angle follows the
reference N ticks ahead, and sends the first one:

    minimise  sum_i q (y_i - r_i)^2  +  lam sum_j (u_j - u_{j-1})^2
    subject to  r_{j+1} - lead <= u_j <= r_{j+1} + lead      (lead 25 deg: never far from what the replay sends today)

solved by FISTA (accelerated projected gradient; the constraints are a box, so the projection is a clamp), warm
started from the last solution. The state (angle, speed) comes from an observer fed with the measured angle.

python mpc.py              self-test, and the numbers JointMPC.h's self-test must reproduce
python mpc.py --demo out   compares, on simulated joints (speed cap, noise, a model that is 30 % off), sending the
                           reference as it is (today's replay), an inverse-model feedforward, and the MPC; plot to out
"""
import math
import sys

import numpy as np

DT = 0.02       # s per tick (PLAY_MS on the Teensy)
N = 25          # horizon, ticks: 0.5 s
SUB = 20        # Euler substeps per tick when discretising, the same on the Teensy
ITERS = 40      # FISTA iterations per tick


def discretise(wn, zeta, dt=DT, sub=SUB):
    """A (2x2), B (2) of x = (angle, speed) over one tick, by substepping the joint like sim.py does."""
    kp, kv, h = wn * wn, 2 * zeta * wn, dt / sub

    def run(y, v, u):
        for _ in range(sub):
            v += (kp * (u - y) - kv * v) * h
            y += v * h
        return y, v

    a0, a1, b = run(1.0, 0.0, 0.0), run(0.0, 1.0, 0.0), run(0.0, 0.0, 1.0)
    return np.array([[a0[0], a1[0]], [a0[1], a1[1]]]), np.array(b)


class JointMPC:
    def __init__(self, wn, zeta, q=1.0, lam=0.05, lead=25.0, n=N):
        self.n, self.q, self.lam, self.lead = n, q, lam, lead
        self.A, self.B = discretise(wn, zeta)
        # prediction: y_{i+1} = Sx[i] . x0 + sum_j<=i Su[i, j] u_j
        self.Sx = np.zeros((n, 2))
        self.Su = np.zeros((n, n))
        Ak = np.eye(2)
        for i in range(n):
            col = Ak @ self.B          # C A^i B
            for j in range(n - i):
                self.Su[i + j, j] = col[0]
            Ak = self.A @ Ak
            self.Sx[i] = Ak[0]         # C A^{i+1}
        D = np.eye(n) - np.eye(n, k=-1)
        self.H = q * self.Su.T @ self.Su + lam * D.T @ D
        self.L = self.max_eig(self.H)
        self.U = None
        self.x = np.zeros(2)
        self.u_prev = 0.0

    @staticmethod
    def max_eig(H, iters=60):
        """Power iteration, as on the Teensy (no eigen solver there)."""
        v = np.ones(len(H))
        for _ in range(iters):
            w = H @ v
            v = w / np.linalg.norm(w)
        return float(v @ H @ v)

    def reset(self, angle, speed=0.0):
        self.x = np.array([angle, speed])
        self.u_prev = angle
        self.U = None

    def observe(self, measured, l1=0.6, l2=0.3):
        """Predict with the setpoint sent last tick, correct with the measured angle (None: no reading)."""
        self.x = self.A @ self.x + self.B * self.u_prev
        if measured is not None:
            e = measured - self.x[0]
            self.x = self.x + np.array([l1 * e, l2 * e / DT])

    def step(self, ref):
        """ref: the reference for the next n ticks (r_1..r_n). Returns the setpoint to send now."""
        ref = np.asarray(ref, float)
        lo, hi = ref - self.lead, ref + self.lead
        d = np.zeros(self.n)
        d[0] = self.u_prev
        D_T_d = d.copy()  # D' d for D lower bidiagonal: only d[0], into element 0
        g = self.q * self.Su.T @ (self.Sx @ self.x - ref) - self.lam * D_T_d
        U = np.clip(np.append(self.U[1:], self.U[-1]) if self.U is not None else ref.copy(), lo, hi)
        Y, t = U.copy(), 1.0
        for _ in range(ITERS):
            Un = np.clip(Y - (self.H @ Y + g) / self.L, lo, hi)
            tn = (1 + math.sqrt(1 + 4 * t * t)) / 2
            Y = Un + (t - 1) / tn * (Un - U)
            U, t = Un, tn
        self.U, self.u_prev = U, float(U[0])
        return self.u_prev


def plant(wn, zeta, vmax, dt=DT, sub=SUB):
    """The 'real' joint for the demo: the same loop with the driver's speed cap."""
    kp, kv, h = wn * wn, 2 * zeta * wn, dt / sub
    st = {"y": 0.0, "v": 0.0}

    def tick(u):
        for _ in range(sub):
            st["v"] = max(-vmax, min(vmax, st["v"] + (kp * (u - st["y"]) - kv * st["v"]) * h))
            st["y"] += st["v"] * h
        return st["y"]
    return st, tick


def reference(T=12.0, dt=DT):
    """A take-like path: slow and quick swings, then a minimum-jerk move and a hold."""
    t = np.arange(0, T, dt)
    r = 25 * np.sin(2 * np.pi * 0.25 * t) + 8 * np.sin(2 * np.pi * 0.9 * t + 1)
    s = np.clip((t - 8) / 1.5, 0, 1)
    r = np.where(t < 8, r, r[int(8 / dt)] + (30 - r[int(8 / dt)]) * s ** 3 * (10 - 15 * s + 6 * s * s))
    return t, r


def run(method, r, true_wn, true_zeta, vmax, model_wn, model_zeta, noise=0.02, seed=1):
    rng = np.random.default_rng(seed)
    st, tick = plant(true_wn, true_zeta, vmax)
    st["y"] = r[0]
    ctl = JointMPC(model_wn, model_zeta)
    ctl.reset(r[0])
    ys, us = [], []
    for k in range(len(r)):
        meas = st["y"] + rng.normal(0, noise)
        ahead = [r[min(k + i, len(r) - 1)] for i in range(1, N + 1)]
        if method == "direct":
            u = r[k]
        elif method == "feedforward":  # u = r + (2 zeta / wn) r' + r'' / wn^2, from the model, on the reference
            r1, r2 = r[min(k + 1, len(r) - 1)], r[min(k + 2, len(r) - 1)]
            rd, rdd = (r2 - r[k]) / (2 * DT), (r2 - 2 * r1 + r[k]) / DT ** 2
            u = r[k] + 2 * model_zeta / model_wn * rd + rdd / model_wn ** 2
        else:
            ctl.observe(meas)
            u = ctl.step(ahead)
        us.append(u)
        ys.append(tick(u))
    ys = np.array(ys)
    err = r[1:] - ys[:-1]  # y after tick k against r at k+1
    return ys, np.array(us), float(np.sqrt(np.mean(err ** 2))), float(np.max(np.abs(err)))


def selftest():
    A, B = discretise(4.47, 0.89)
    assert B[0] > 0 and abs(A[0, 0] + B[0] - 1) < 1e-9 and abs(A[1, 0] + B[1]) < 1e-9  # at rest on its setpoint: stays
    m = JointMPC(4.47, 0.89)
    m.reset(0.0)
    assert abs(m.step([0.0] * N)) < 1e-6                     # at rest on a still reference: stays
    m.reset(0.0)
    u = m.step([10.0] * N)
    assert 10.0 < u <= 10.0 + m.lead, u                      # a step ahead: leads it, inside the lead box
    m.reset(0.0)
    assert abs(m.step([100.0] * N) - 100.0 - m.lead) < 1e-6  # far away: held at the box edge, ref + lead
    t, r = reference()
    base = run("direct", r, 4.47, 0.89, 60, 4.47, 0.89)[2]
    mpc = run("mpc", r, 4.47, 0.89, 60, 4.47, 0.89)[2]
    assert mpc < 0.5 * base, (base, mpc)
    # the numbers JointMPC.h's self-test reproduces: wn 6, zeta 0.8, from rest at 0, reference ramping 0.5 deg a tick
    m = JointMPC(6.0, 0.8)
    m.reset(0.0)
    us = [m.step([0.5 * (k + i) for i in range(1, N + 1)]) for k in range(3)]
    print("JointMPC.h self-test reference: L %.4f  u %.4f %.4f %.4f" % (m.L, *us))
    print("mpc ok")


def demo(out):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    t, r = reference()
    cases = [("model right", 4.47, 0.89, 4.47, 0.89), ("real joint 30 % slower", 3.13, 0.89, 4.47, 0.89),
             ("real joint 30 % faster, less damped", 5.8, 0.6, 4.47, 0.89)]
    fig, axes = plt.subplots(len(cases), 1, figsize=(11, 3.2 * len(cases)), squeeze=False)
    for ax, (name, twn, tz, mwn, mz) in zip(axes[:, 0], cases):
        ax.plot(t, r, color="#999999", lw=2, label="reference (the take)")
        print(name)
        for method, col in (("direct", "#A4161A"), ("feedforward", "#1D6FA5"), ("mpc", "#2E8B57")):
            ys, us, rms, mx = run(method, r, twn, tz, 60, mwn, mz)
            print("  %-11s RMS %.2f deg  max %.2f deg" % (method, rms, mx))
            ax.plot(t, ys, color=col, lw=1.4, label="%s  RMS %.2f°" % (method, rms))
        ax.set_title(name, fontsize=10)
        ax.set_ylabel("°")
        ax.legend(fontsize=8, loc="lower left")
    axes[-1, 0].set_xlabel("s")
    fig.tight_layout()
    fig.savefig(out, dpi=110)
    print("plot saved to %s" % out)


if __name__ == "__main__":
    if len(sys.argv) > 2 and sys.argv[1] == "--demo":
        demo(sys.argv[2])
    else:
        selftest()

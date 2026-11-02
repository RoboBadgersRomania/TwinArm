// JointMPC.h: outer-loop model predictive control for one CAN joint, the C++ twin of InterfaceApp/mpc.py
// (the maths, the demo against today's replay and the numbers this self-test checks are there).
//
// The SteadyWin driver keeps its own position / velocity loops; seen from the Teensy a joint is the closed loop the
// step tests fit:  acceleration = wn^2 (setpoint - angle) - 2 zeta wn speed.  Every tick (DT) step() takes the
// reference for the next N ticks and returns the setpoint to send now, the first of N that minimise
//     sum q (predicted angle - reference)^2 + lam sum (setpoint change)^2
// with every setpoint inside reference +- lead. Solved by FISTA (projected gradient, the box is a clamp), warm started.
// observe() keeps the state (angle, speed) from the model and the measured angle.
#pragma once
#include <Arduino.h>
#include <math.h>

template <int N = 25>
class JointMPC {
 public:
  static constexpr float DT = 0.02f;      // s per tick: the replay's PLAY_MS
  static constexpr int SUB = 20, ITERS = 40;
  float wn = 0, zeta = 0, lead = 25, lam = 0.05f, q = 1, L = 1;
  bool ready = false;                      // a model was given
  float x[2] = {0, 0}, uPrev = 0;          // state estimate (deg, deg/s), the setpoint sent last tick

  // The joint's fitted model (from a step test). Builds the prediction and the QP's matrices, ~0.2 ms.
  void setModel(float wn_, float zeta_) {
    wn = wn_;
    zeta = zeta_;
    discretise();
    float Ak[2][2] = {{1, 0}, {0, 1}};
    for (int i = 0; i < N; i++) {
      float col = Ak[0][0] * B[0] + Ak[0][1] * B[1];  // C A^i B
      for (int j = 0; j + i < N; j++) Su[i + j][j] = col;  // (the upper triangle stays 0)
      float n00 = A[0][0] * Ak[0][0] + A[0][1] * Ak[1][0], n01 = A[0][0] * Ak[0][1] + A[0][1] * Ak[1][1];
      float n10 = A[1][0] * Ak[0][0] + A[1][1] * Ak[1][0], n11 = A[1][0] * Ak[0][1] + A[1][1] * Ak[1][1];
      Ak[0][0] = n00, Ak[0][1] = n01, Ak[1][0] = n10, Ak[1][1] = n11;
      Sx[i][0] = Ak[0][0];                            // C A^{i+1}
      Sx[i][1] = Ak[0][1];
    }
    for (int r = 0; r < N; r++)
      for (int c = 0; c < N; c++) {
        float s = 0;
        for (int i = max(r, c); i < N; i++) s += Su[i][r] * Su[i][c];  // Su is lower triangular
        float dd = r == c ? (r < N - 1 ? 2 : 1) : abs(r - c) == 1 ? -1 : 0;  // D'D, D = I - subdiagonal
        H[r][c] = q * s + lam * dd;
      }
    float v[N], w[N];  // largest eigenvalue by power iteration: the step size
    for (int i = 0; i < N; i++) v[i] = 1;
    for (int it = 0; it < 60; it++) {
      mul(H, v, w);
      float n = 0;
      for (int i = 0; i < N; i++) n += w[i] * w[i];
      n = sqrtf(n);
      for (int i = 0; i < N; i++) v[i] = w[i] / n;
    }
    mul(H, v, w);
    L = 0;
    for (int i = 0; i < N; i++) L += v[i] * w[i];
    ready = true;
    warm = false;
  }

  void reset(float angle, float speed = 0) {
    x[0] = angle;
    x[1] = speed;
    uPrev = angle;
    warm = false;
  }

  // Predict with last tick's setpoint, then correct with the measured angle (ok = false: no reading this tick).
  void observe(float measured, bool ok, float l1 = 0.6f, float l2 = 0.3f) {
    float y = A[0][0] * x[0] + A[0][1] * x[1] + B[0] * uPrev;
    float v = A[1][0] * x[0] + A[1][1] * x[1] + B[1] * uPrev;
    if (ok) {
      float e = measured - y;
      y += l1 * e;
      v += l2 * e / DT;
    }
    x[0] = y;
    x[1] = v;
  }

  // ref: the reference for the next N ticks. Returns the setpoint to send now.
  float step(const float *ref) {
    float lo[N], hi[N], w[N], g[N], Y[N], Un[N], grad[N];
    for (int i = 0; i < N; i++) {
      lo[i] = ref[i] - lead;
      hi[i] = ref[i] + lead;
      w[i] = Sx[i][0] * x[0] + Sx[i][1] * x[1] - ref[i];
    }
    for (int j = 0; j < N; j++) {
      float s = 0;
      for (int i = j; i < N; i++) s += Su[i][j] * w[i];
      g[j] = q * s;
    }
    g[0] -= lam * uPrev;
    for (int i = 0; i < N; i++) {  // warm start: last solution moved on by a tick
      float u = warm ? U[min(i + 1, N - 1)] : ref[i];
      U[i] = Y[i] = constrain(u, lo[i], hi[i]);
    }
    float t = 1;
    for (int it = 0; it < ITERS; it++) {
      mul(H, Y, grad);
      for (int i = 0; i < N; i++) Un[i] = constrain(Y[i] - (grad[i] + g[i]) / L, lo[i], hi[i]);
      float tn = (1 + sqrtf(1 + 4 * t * t)) / 2, k = (t - 1) / tn;
      for (int i = 0; i < N; i++) {
        Y[i] = Un[i] + k * (Un[i] - U[i]);
        U[i] = Un[i];
      }
      t = tn;
    }
    warm = true;
    uPrev = U[0];
    return uPrev;
  }

  // mpc.py's selftest prints the same: wn 6, zeta 0.8, from rest at 0, the reference ramping 0.5 deg a tick.
  static bool selfTest(Print &log, uint32_t &us) {
    static JointMPC<N> m;
    m.setModel(6.0f, 0.8f);
    m.reset(0);
    const float want[3] = {8.2832f, 14.3847f, 19.3184f};
    bool ok = fabsf(m.L - 0.4200f) < 0.005f;
    float ref[N], got[3];
    uint32_t t0 = micros();
    for (int k = 0; k < 3; k++) {
      for (int i = 0; i < N; i++) ref[i] = 0.5f * (k + i + 1);
      got[k] = m.step(ref);
      ok &= fabsf(got[k] - want[k]) < 0.02f * fabsf(want[k]) + 0.01f;
    }
    us = (micros() - t0) / 3;
    log.printf("mpc self-test %s: L %.4f (0.4200), u %.4f %.4f %.4f (%.4f %.4f %.4f), %lu us a solve\n",
               ok ? "ok" : "FAILED", m.L, got[0], got[1], got[2], want[0], want[1], want[2], (unsigned long)us);
    return ok;
  }

 private:
  float A[2][2], B[2], Sx[N][2], Su[N][N] = {}, H[N][N], U[N];
  bool warm = false;

  void discretise() {  // the joint over one tick, substepped like sim.py: A, B of x = (angle, speed)
    float kp = wn * wn, kv = 2 * zeta * wn, h = DT / SUB;
    auto run = [&](float y, float v, float u, float &yo, float &vo) {
      for (int i = 0; i < SUB; i++) {
        v += (kp * (u - y) - kv * v) * h;
        y += v * h;
      }
      yo = y;
      vo = v;
    };
    run(1, 0, 0, A[0][0], A[1][0]);
    run(0, 1, 0, A[0][1], A[1][1]);
    run(0, 0, 1, B[0], B[1]);
  }

  static void mul(const float (&M)[N][N], const float *v, float *out) {
    for (int r = 0; r < N; r++) {
      float s = 0;
      for (int c = 0; c < N; c++) s += M[r][c] * v[c];
      out[r] = s;
    }
  }
};

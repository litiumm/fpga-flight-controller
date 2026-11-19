#!/usr/bin/env python3
"""Reference models for the FPGA flight-control core.

This file is the single source of truth for plant and gain parameters.
It generates the Verilog include (sim/params.vh) and the stimulus files,
then checks the RTL simulation output against two references:

  * GoldenPID - bit-exact integer model of rtl/pid_core.v + rtl/mixer.v.
                The RTL closed loop must match it sample for sample.
  * FloatPID  - ideal floating-point PID with the same gains (the
                "textbook" / MATLAB-style controller). Comparing against it
                shows the cost of the fixed-point datapath.

Plant: single-axis seesaw (equivalently one axis of a quadcopter) with a
first-order motor lag, viscous damping, a constant imbalance torque (so the
I-term has work to do) and an external push at t = 2.0 s.

Usage:
  python3 model/fc_model.py gen        # write sim/params.vh, stimulus files
  python3 model/fc_model.py check      # compare RTL output, make plots
"""
import math
import random
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
SIM = ROOT / "sim"
RES = ROOT / "results"

# ---------------------------------------------------------------- plant
J = 0.01            # kg m^2, beam inertia about the pivot
ARM = 0.20          # m, pivot-to-motor distance
TMAX = 5.0          # N, thrust per motor at 2000 us
TAU = 0.020         # s, motor/ESC time constant
DAMP = 0.002        # N m s/rad, viscous damping at the pivot
IMBAL = 0.02        # N m, constant imbalance torque
DIST = 0.30         # N m, external push
THETA0 = 15.0       # deg, released from this tilt

# ---------------------------------------------------------------- loop
LOOP_HZ = 1000
SUBSTEPS = 10                   # plant integration steps per control period
DT_CTRL = 1.0 / LOOP_HZ
DT_PLANT = DT_CTRL / SUBSTEPS
N_STEPS = 4000                  # 4 s
THROTTLE = 1500                 # us, hover-ish base command

# Setpoint profile and disturbance, defined by sample index (no float compares)
def setpoint_deg(k):
    if k < 1000:
        return 0.0
    if k < 3000:
        return 10.0
    return 0.0

DIST_START = 2000 * SUBSTEPS    # plant substep index
DIST_LEN = 50 * SUBSTEPS        # 50 ms push

# Sensor noise (1-sigma), same sequence fed to every controller
ANGLE_NOISE = 0.05  # deg
RATE_NOISE = 0.30   # deg/s
SEED = 1

# ---------------------------------------------------------------- gains
# Physical gains, picked by pole placement on the PD + plant model:
#   angular accel per us of differential command = 2*TMAX/1000*ARM/J rad/s^2
#   wn ~ 15 rad/s, zeta ~ 1 (PD), then Ki and I_MAX chosen by a sweep
#   minimising 2 % settling time with < 15 % overshoot (see README).
KP = 20.0           # us / deg
KI = 80.0           # us / (deg s)
KD = 2.6            # us / (deg/s)
OUT_MAX = 400       # us
I_MAX = 20          # us  (2x the imbalance it must cancel; limits windup)

KP_Q = round(KP * 256)                  # Q8.8
KI_Q = round(KI * DT_CTRL * 65536)      # UQ0.16, dt folded in
KD_Q = round(KD * 256)                  # Q8.8


def q(x, frac, bits=16):
    """Quantise a real to signed fixed point (round half up, saturate)."""
    v = math.floor(x * (1 << frac) + 0.5)
    lo, hi = -(1 << (bits - 1)), (1 << (bits - 1)) - 1
    return max(lo, min(hi, v))


# ---------------------------------------------------------------- controllers
class GoldenPID:
    """Bit-exact model of rtl/pid_core.v."""
    QF = 24

    def __init__(self, kp=KP_Q, ki=KI_Q, kd=KD_Q, out_max=OUT_MAX, i_max=I_MAX):
        self.kp, self.ki, self.kd = kp, ki, kd
        self.out_max, self.i_lim = out_max, i_max << self.QF
        self.i_acc = 0

    def reset(self):
        self.i_acc = 0

    def step(self, sp_q, ang_q, rate_q):
        err = sp_q - ang_q
        p = (err * self.kp) << 8
        i = self.i_acc + err * self.ki
        self.i_acc = max(-self.i_lim, min(self.i_lim, i))
        d = -((rate_q * self.kd) << 12)
        s = p + self.i_acc + d
        si = (s + (1 << (self.QF - 1))) >> self.QF   # Python >> is arithmetic
        return max(-self.out_max, min(self.out_max, si))


def mix(u, throttle=THROTTLE):
    clamp = lambda x: max(1000, min(2000, x))
    return clamp(throttle + u), clamp(throttle - u)


class FloatPID:
    """Ideal real-valued PID, same structure and (quantised) gains."""

    def __init__(self):
        self.kp = KP_Q / 256
        self.ki = KI_Q / 65536 / DT_CTRL
        self.kd = KD_Q / 256
        self.i = 0.0

    def step(self, sp, ang, rate):
        e = sp - ang
        self.i = max(-I_MAX, min(I_MAX, self.i + self.ki * DT_CTRL * e))
        u = self.kp * e + self.i - self.kd * rate
        return max(-OUT_MAX, min(OUT_MAX, u))


# ---------------------------------------------------------------- simulation
def noise_sequence():
    rng = random.Random(SEED)
    return [(rng.gauss(0, ANGLE_NOISE), rng.gauss(0, RATE_NOISE)) for _ in range(N_STEPS)]


def simulate(kind, ctrl=None):
    """Closed-loop run. kind = 'golden' (fixed point), 'float', or 'fpga'
    (pass ctrl = an object with .step(sp_q, ang_q, rate_q) -> u).

    The plant arithmetic here is written in exactly the same order as in
    sim/tb_closed_loop.v so that double-precision results are identical.
    """
    noise = noise_sequence()
    if ctrl is None:
        ctrl = GoldenPID() if kind == "golden" else FloatPID()
    theta = THETA0 * math.pi / 180.0    # rad
    omega = 0.0                         # rad/s
    f1 = f2 = TMAX * (THROTTLE - 1000) / 1000.0
    log = []
    sub = 0
    for k in range(N_STEPS):
        sp = setpoint_deg(k)
        ang = theta * 180.0 / math.pi + noise[k][0]
        rate = omega * 180.0 / math.pi + noise[k][1]
        if kind in ("golden", "fpga"):
            sp_q, ang_q, rate_q = q(sp, 8), q(ang, 8), q(rate, 4)
            u = ctrl.step(sp_q, ang_q, rate_q)
            m1, m2 = mix(u)
        else:
            u = ctrl.step(sp, ang, rate)
            m1 = max(1000.0, min(2000.0, THROTTLE + u))
            m2 = max(1000.0, min(2000.0, THROTTLE - u))
        log.append((k * DT_CTRL, sp, theta * 180.0 / math.pi, u, m1, m2))
        for _ in range(SUBSTEPS):
            f1c = TMAX * (m1 - 1000) / 1000.0
            f2c = TMAX * (m2 - 1000) / 1000.0
            f1 = f1 + (f1c - f1) * DT_PLANT / TAU
            f2 = f2 + (f2c - f2) * DT_PLANT / TAU
            tq = ARM * (f1 - f2) + IMBAL
            if DIST_START <= sub < DIST_START + DIST_LEN:
                tq = tq + DIST
            alpha = (tq - DAMP * omega) / J
            omega = omega + alpha * DT_PLANT
            theta = theta + omega * DT_PLANT
            sub += 1
    return log


# ---------------------------------------------------------------- file gen
def gen():
    SIM.mkdir(exist_ok=True)
    r = repr
    vh = f"""// AUTO-GENERATED by model/fc_model.py - do not edit
localparam real J        = {r(J)};
localparam real ARM      = {r(ARM)};
localparam real TMAX     = {r(TMAX)};
localparam real TAU      = {r(TAU)};
localparam real DAMP     = {r(DAMP)};
localparam real IMBAL    = {r(IMBAL)};
localparam real DIST     = {r(DIST)};
localparam real THETA0   = {r(THETA0)};
localparam real DT_CTRL  = {r(DT_CTRL)};
localparam real DT_PLANT = {r(DT_PLANT)};
localparam real PI       = {r(math.pi)};
localparam integer SUBSTEPS   = {SUBSTEPS};
localparam integer N_STEPS    = {N_STEPS};
localparam integer THROTTLE   = {THROTTLE};
localparam integer DIST_START = {DIST_START};
localparam integer DIST_LEN   = {DIST_LEN};
localparam integer OUT_MAX    = {OUT_MAX};
localparam integer I_MAX      = {I_MAX};
localparam integer KP_Q = {KP_Q};
localparam integer KI_Q = {KI_Q};
localparam integer KD_Q = {KD_Q};
"""
    (SIM / "params.vh").write_text(vh)

    # Per-sample stimulus: setpoint, angle noise, rate noise (exact reprs)
    with open(SIM / "stimulus.txt", "w") as f:
        for k, (na, nr) in enumerate(noise_sequence()):
            f.write(f"{r(setpoint_deg(k))} {r(na)} {r(nr)}\n")

    # Random unit-test vectors for tb_pid_vectors.v, golden expected output
    rng = random.Random(42)
    pid = GoldenPID()
    lines = []
    edge = [-32768, -32767, -1, 0, 1, 32767]
    for n in range(20000):
        if n % 500 == 0:  # new random gains every 500 samples
            pid.kp = rng.randint(-32768, 32767)
            pid.ki = rng.randint(0, 65535)
            pid.kd = rng.randint(-32768, 32767)
        rst = 1 if rng.random() < 0.01 else 0
        pick = lambda: rng.choice(edge) if rng.random() < 0.1 else rng.randint(-32768, 32767)
        small = lambda: rng.randint(-2000, 2000)
        if rng.random() < 0.5:
            sp, ang, rate = pick(), pick(), pick()
        else:
            sp, ang, rate = small(), small(), small()
        if rst:
            pid.reset()
            exp = 0
        else:
            exp = pid.step(sp, ang, rate)
        h = lambda v, b=16: format(v & ((1 << b) - 1), "04x")
        lines.append(f"{rst} {h(pid.kp)} {h(pid.ki)} {h(pid.kd)} {h(sp)} {h(ang)} {h(rate)} {h(exp)}")
    (SIM / "vectors.txt").write_text("\n".join(lines) + "\n")

    # Closed-loop sensor frames for tb_hil_top.v (first 300 samples of the run)
    rec = []
    class Rec(GoldenPID):
        def step(self, a, b, c):
            u = super().step(a, b, c)
            rec.append((a, b, c, u))
            return u
    simulate("fpga", Rec())
    h = lambda v: format(v & 0xFFFF, "04x")
    (SIM / "hil_vectors.txt").write_text(
        "".join(f"{h(a)} {h(b)} {h(c)} {h(u)}\n" for a, b, c, u in rec[:300]))
    print(f"wrote {SIM/'params.vh'}, stimulus.txt, vectors.txt  "
          f"(KP_Q={KP_Q} KI_Q={KI_Q} KD_Q={KD_Q})")


# ---------------------------------------------------------------- check
def metrics(t, sp, th, t0, t1, target):
    """Step-response metrics for the window [t0, t1)."""
    idx = [i for i, x in enumerate(t) if t0 <= x < t1]
    seg = [th[i] for i in idx]
    start = th[idx[0]]
    span = target - start
    pk = max(seg) if span > 0 else min(seg)
    over = (pk - target) / span * 100 if span else 0.0
    # 2 % settling time
    band = 0.02 * abs(span)
    ts = None
    for j in range(len(seg)):
        if all(abs(v - target) <= band for v in seg[j:]):
            ts = t[idx[j]] - t0
            break
    # 10-90 rise time
    r10 = next(t[idx[j]] for j in range(len(seg)) if (seg[j] - start) / span >= 0.1)
    r90 = next(t[idx[j]] for j in range(len(seg)) if (seg[j] - start) / span >= 0.9)
    sse = sum(abs(th[i] - target) for i in idx[-200:]) / 200
    return dict(rise_ms=(r90 - r10) * 1e3, overshoot_pct=over,
                settle_ms=None if ts is None else ts * 1e3, sse_deg=sse)


def check():
    import csv
    rtl_path = RES / "rtl_closed_loop.csv"
    if not rtl_path.exists():
        sys.exit(f"{rtl_path} missing - run `make sim` first")
    with open(rtl_path) as f:
        rtl = [tuple(map(float, row)) for row in csv.reader(f)]
    gold = simulate("golden")
    flt = simulate("float")

    # 1. Bit-exact check: RTL vs golden model
    mism = [k for k in range(N_STEPS)
            if int(rtl[k][3]) != gold[k][3] or abs(rtl[k][2] - gold[k][2]) > 1e-9]
    if mism:
        k = mism[0]
        print(f"FAIL: {len(mism)} mismatching samples, first at k={k}: rtl={rtl[k]} gold={gold[k]}")
    else:
        print(f"PASS: RTL closed loop matches golden model bit-exactly over {N_STEPS} samples")

    # 2. Fixed point vs ideal float controller
    err = [abs(rtl[k][2] - flt[k][2]) for k in range(N_STEPS)]
    print(f"RTL vs float PID: max |theta diff| = {max(err):.4f} deg, "
          f"mean = {sum(err)/len(err):.5f} deg")

    t = [r[0] for r in rtl]
    th_r = [r[2] for r in rtl]
    th_f = [r[2] for r in flt]
    sp = [r[1] for r in rtl]
    m_r = metrics(t, sp, th_r, 1.0, 2.0, 10.0)
    m_f = metrics(t, sp, th_f, 1.0, 2.0, 10.0)
    print("Step 0 -> 10 deg   rise(ms)  overshoot(%)  settle2%(ms)  ss-err(deg)")
    for name, m in (("RTL  ", m_r), ("float", m_f)):
        st = f"{m['settle_ms']:.0f}" if m["settle_ms"] is not None else "n/a"
        print(f"  {name}          {m['rise_ms']:6.0f}  {m['overshoot_pct']:11.1f}  "
              f"{st:>12}  {m['sse_deg']:11.3f}")

    plot(rtl, flt, err)
    return 0 if not mism else 1


def plot(rtl, flt, err):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    t = [r[0] for r in rtl]
    fig, ax = plt.subplots(3, 1, figsize=(10, 9), sharex=True,
                           gridspec_kw=dict(height_ratios=[3, 2, 1.3]))
    ax[0].plot(t, [r[1] for r in rtl], "k--", lw=1, label="setpoint")
    ax[0].plot(t, [r[2] for r in flt], color="#999", lw=3, label="ideal float PID")
    ax[0].plot(t, [r[2] for r in rtl], color="#1f6feb", lw=1.2, label="RTL (pid_core.v)")
    ax[0].axvspan(2.0, 2.05, color="#f85149", alpha=0.2, label="0.3 N·m push")
    ax[0].set_ylabel("angle (deg)")
    ax[0].legend(loc="upper right")
    ax[0].set_title("Closed loop: Verilog PID core stabilising a single-axis rig (1 kHz loop)")
    ax[0].grid(alpha=0.3)

    ax[1].plot(t, [r[4] for r in rtl], lw=1, label="motor 1")
    ax[1].plot(t, [r[5] for r in rtl], lw=1, label="motor 2")
    ax[1].set_ylabel("ESC pulse (us)")
    ax[1].legend(loc="upper right")
    ax[1].grid(alpha=0.3)

    ax[2].plot(t, err, color="#8957e5", lw=1)
    ax[2].set_ylabel("|RTL - float|\n(deg)")
    ax[2].set_xlabel("time (s)")
    ax[2].grid(alpha=0.3)

    fig.tight_layout()
    RES.mkdir(exist_ok=True)
    out = RES / "closed_loop.png"
    fig.savefig(out, dpi=150)
    print(f"plot: {out}")


if __name__ == "__main__":
    cmd = sys.argv[1] if len(sys.argv) > 1 else "check"
    if cmd == "gen":
        gen()
    elif cmd == "check":
        sys.exit(check())
    else:
        sys.exit(__doc__)

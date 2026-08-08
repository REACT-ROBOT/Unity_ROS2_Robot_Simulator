#!/usr/bin/env python3
"""Regenerate the figures in docs/Servo-Model-Validation*.md.

Reads the CSVs that ServoPendulumTests writes to <project>/TestOutput/ and
renders the static plots and the pin-in-slot animations into
docs/images/servo-model/.

    # 1. produce fresh CSVs
    Unity -batchmode -nographics -projectPath . -runTests \
          -testPlatform playmode -testFilter ServoPendulumTests \
          -testResults results.xml -logFile unity.log
    # 2. render
    python3 docs/tools/plot_servo_validation.py

Note on velocity: the CSVs carry an `omegaL` column straight from
ArticulationBody.jointVelocity, which does not report motion an xDrive is
producing (see the validation report). Every plot here derives the joint
velocity from the position instead, and `omegaL` is never drawn.
"""

import csv
import math
import os
import sys

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation, PillowWriter
from matplotlib.patches import Rectangle, Circle

ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
CSV_DIR = os.path.join(ROOT, "TestOutput")
IMG_DIR = os.path.join(ROOT, "docs", "images", "servo-model")

DT = 0.02
GAP = 2 * 0.0087          # total backlash [rad]
KP, KTRANS = 20.0, 50.0   # servo P gain, transmission stiffness of the backlash tests
TAU_S, TAU_C = 0.15, 0.08
WS, SIGMA = 0.1, 0.005

CMD_COLOR = "tab:orange"
JOINT_COLOR = "tab:blue"


def load(name):
    path = os.path.join(CSV_DIR, name)
    if not os.path.exists(path):
        sys.exit(f"missing {path} — run ServoPendulumTests first")
    with open(path) as fh:
        rows = [{k: float(v) for k, v in r.items()} for r in csv.DictReader(fh)]
    if not rows:
        sys.exit(f"{path} is empty")
    return {k: [r[k] for r in rows] for k in rows[0]}


def joint_velocity(theta):
    """Position finite difference — jointVelocity is not usable here."""
    return [0.0] + [(theta[i] - theta[i - 1]) / DT for i in range(1, len(theta))]


def save(fig, name):
    os.makedirs(IMG_DIR, exist_ok=True)
    out = os.path.join(IMG_DIR, name)
    fig.savefig(out, dpi=110, bbox_inches="tight")
    plt.close(fig)
    print("wrote", os.path.relpath(out, ROOT))


# --------------------------------------------------------------------------
# static plots
# --------------------------------------------------------------------------

def plot_hysteresis():
    d = load("hysteresis.csv")
    fig, (ax0, ax1) = plt.subplots(1, 2, figsize=(10, 4.2))
    ax0.plot(d["cmd"], d["thetaL"], lw=1.0, color=JOINT_COLOR)
    lim = max(map(abs, d["cmd"])) * 1.05
    ax0.plot([-lim, lim], [-lim, lim], ls=":", color="0.6", lw=0.9)
    ax0.set_xlabel("command [rad]")
    ax0.set_ylabel("joint [rad]")
    ax0.set_title("hysteresis loop")
    ax0.grid(alpha=0.3)

    ax1.plot(d["t"], d["cmd"], label="cmd", color=CMD_COLOR)
    ax1.plot(d["t"], d["thetaL"], label="thetaL", color=JOINT_COLOR)
    ax1.plot(d["t"], d["thetaM"], label="thetaM", color="tab:green", lw=0.9)
    ax1.set_xlabel("t [s]")
    ax1.legend(loc="upper right", fontsize=8)
    ax1.grid(alpha=0.3)
    save(fig, "hysteresis.png")


def plot_gravity_crossing():
    d = load("gravity_crossing.csv")
    vel = joint_velocity(d["thetaL"])
    # Time at which the pendulum passes its lowest point, where the load
    # torque reverses.
    bottom = min(range(len(d["t"])), key=lambda i: abs(d["thetaL"][i]))

    fig, ax = plt.subplots(4, 1, figsize=(9, 9), sharex=True)
    ax[0].plot(d["t"], d["cmd"], label="cmd", color=CMD_COLOR)
    ax[0].plot(d["t"], d["thetaL"], label="thetaL", color=JOINT_COLOR)
    ax[0].legend(loc="upper left", fontsize=8)
    ax[0].set_ylabel("angle [rad]")

    ax[1].axhline(GAP / 2, ls="--", color="0.5", lw=0.9)
    ax[1].axhline(-GAP / 2, ls="--", color="0.5", lw=0.9)
    ax[1].axhline(0, color="0.8", lw=0.8)
    ax[1].plot(d["t"], d["delta"], color=JOINT_COLOR)
    ax[1].set_ylabel("delta [rad]")
    ax[1].text(0.01, 0.08, "dashed: ±b (half the gap)", fontsize=7.5, color="0.35",
               transform=ax[1].transAxes)

    ax[2].axhline(0, color="0.8", lw=0.8)
    ax[2].plot(d["t"], d["driveForce"], color=JOINT_COLOR)
    ax[2].set_ylabel("transmitted\ntorque [N m]")

    ax[3].plot(d["t"], vel, color=JOINT_COLOR)
    ax[3].set_ylabel("joint vel [rad/s]")
    ax[3].set_xlabel("t [s]")

    for a in ax:
        a.axvline(d["t"][bottom], color="tab:red", lw=0.9, ls="-.", alpha=0.7)
        a.grid(alpha=0.3)
    ax[0].text(d["t"][bottom], ax[0].get_ylim()[0], " lowest point (load torque reverses)",
               fontsize=7.5, color="tab:red", va="bottom")
    save(fig, "gravity_crossing.png")


def plot_friction_curve():
    d = load("friction_curve.csv")
    fig, ax = plt.subplots(figsize=(6.5, 4.6))
    ws = [i / 200.0 * 2.2 for i in range(1, 201)]
    model = [TAU_C + (TAU_S - TAU_C) * math.exp(-(w / WS) ** 2) + SIGMA * w for w in ws]
    ax.plot(ws, model, color="black", lw=1.0, label="Stribeck model")
    ax.plot(d["omega"], d["tauMeasured"], "o", color=JOINT_COLOR,
            label="measured (servo balance − τ_trans)")
    ax.set_xlabel("omega [rad/s]")
    ax.set_ylabel("tau_f [N m]")
    ax.legend()
    ax.grid(alpha=0.3)
    save(fig, "friction_curve.png")


def plot_stiction_hold():
    d = load("stiction_hold.csv")
    vel = joint_velocity(d["thetaL"])
    fig, ax = plt.subplots(2, 1, figsize=(9, 5.6), sharex=True)
    ax[0].axhline(d["cmd"][0], ls="--", color="0.5", lw=0.9)
    ax[0].plot(d["t"], d["thetaL"], color=JOINT_COLOR)
    ax[0].set_ylabel("thetaL [rad]")
    ax[1].plot(d["t"], vel, color=JOINT_COLOR)
    ax[1].set_ylabel("joint vel [rad/s]\n(position difference)")
    ax[1].set_xlabel("t [s]")
    for a in ax:
        a.grid(alpha=0.3)
    save(fig, "stiction_hold.png")


def _detrend(t, y, win_s=1.0):
    """Remove the slowly growing gravity deflection with a moving average, so
    what is left is the wind-up/release judder. A straight-line fit is not
    enough: the gravity torque follows sin(theta) and its curvature alone is
    larger than the judder."""
    w = max(3, int(win_s / DT) | 1)
    half = w // 2
    out = []
    for i in range(len(y)):
        a, b = max(0, i - half), min(len(y), i + half + 1)
        out.append(y[i] - sum(y[a:b]) / (b - a))
    return out


def _breakaways(omega_m):
    stuck = [w == 0.0 for w in omega_m]
    return stuck, sum(1 for i in range(1, len(stuck)) if stuck[i - 1] and not stuck[i])


def plot_stick_slip():
    """Same 0.07 rad/s sweep, twice: lightly damped servo vs the validated Kd."""
    lo_bound = (TAU_S - TAU_C) / (KP + KTRANS)
    up_bound = (TAU_S - TAU_C) / (1.0 / (1.0 / KP + 1.0 / KTRANS))

    cases = [("stick_slip.csv", "Kd = 0.02 (lightly damped)"),
             ("stick_slip_damped.csv", f"Kd = 0.5 (validated, ζ = 1.25)")]
    fig, ax = plt.subplots(3, 2, figsize=(11, 8), sharex=True)
    for col, (name, label) in enumerate(cases):
        d = load(name)
        err = [c - x for c, x in zip(d["cmd"], d["thetaL"])]
        stuck, nb = _breakaways(d["omegaM"])
        detr = _detrend(d["t"], err)

        ax[0][col].plot(d["t"], d["cmd"], label="cmd", color=CMD_COLOR)
        ax[0][col].plot(d["t"], d["thetaL"], label="joint", color=JOINT_COLOR)
        ax[0][col].set_title(f"{label} — {nb} breakaways", fontsize=10)
        if col == 0:
            ax[0][col].legend(loc="upper left", fontsize=8)
            ax[0][col].set_ylabel("angle [rad]")

        ax[1][col].plot(d["t"], detr, color=JOINT_COLOR)
        for sgn in (1, -1):
            ax[1][col].axhline(sgn * up_bound / 2, ls="--", color="0.5", lw=0.9)
            ax[1][col].axhline(sgn * lo_bound / 2, ls=":", color="0.6", lw=0.9)
        # Clip to the steady judder; breaking away from rest is a much larger
        # one-off excursion and would flatten everything else.
        ax[1][col].set_ylim(-0.8 * up_bound, 0.8 * up_bound)
        if col == 0:
            ax[1][col].set_ylabel("judder [rad]\n(gravity trend removed)")
            ax[1][col].text(0.02, 0.06,
                            f"dashed ±(τ_s−τ_c)/2K_series = ±{up_bound / 2:.4f}\n"
                            f"dotted ±(τ_s−τ_c)/2(Kp+K) = ±{lo_bound / 2:.4f}",
                            fontsize=7.5, color="0.35", transform=ax[1][col].transAxes)

        ax[2][col].plot(d["t"], d["omegaM"], color="tab:green", lw=0.9)
        st = [d["t"][i] for i in range(len(stuck)) if stuck[i]]
        if st:
            ax[2][col].plot(st, [0.0] * len(st), "|", ms=7, color="tab:red",
                            label="rotor stuck (Karnopp)")
            ax[2][col].legend(loc="upper right", fontsize=8)
        ax[2][col].set_xlabel("t [s]")
        if col == 0:
            ax[2][col].set_ylabel("motor vel [rad/s]")
        skip = int(1.0 / DT)   # ignore the breakaway-from-rest transient
        steady = detr[skip:]
        print(f"  {name}: {nb} breakaways, steady judder p-p "
              f"{max(steady) - min(steady):.4f} rad "
              f"(bounds {lo_bound:.4f}..{up_bound:.4f})")

    # share the vertical scale across the two columns so the contrast is honest
    for r, row in enumerate(ax):
        if r != 1:   # the judder row is already clipped to a common range
            ylo = min(a.get_ylim()[0] for a in row)
            yhi = max(a.get_ylim()[1] for a in row)
            for a in row:
                a.set_ylim(ylo, yhi)
        for a in row:
            a.grid(alpha=0.3)
    fig.suptitle("Stick-slip: identical 0.07 rad/s sweep under gravity, only the servo damping differs",
                 fontsize=11)
    fig.tight_layout(rect=(0, 0, 1, 0.97))
    save(fig, "stick_slip.png")


# --------------------------------------------------------------------------
# pin-in-slot animations
# --------------------------------------------------------------------------

def _pendulum_axes(ax):
    ax.set_xlim(-0.34, 0.34)
    ax.set_ylim(-0.36, 0.14)
    ax.set_aspect("equal")
    ax.axis("off")
    ax.add_patch(Rectangle((-0.09, 0.005), 0.18, 0.03, color="0.6"))
    ax.annotate("", xy=(0.2, -0.09), xytext=(0.2, -0.02),
                arrowprops=dict(arrowstyle="->", color="0.4", lw=1.0))
    ax.text(0.215, -0.06, "g", color="0.4", style="italic", fontsize=9)
    joint_line, = ax.plot([], [], lw=2.6, color=JOINT_COLOR, label="joint", zorder=2)
    # Drawn on top and slightly longer: the two arms are within ~0.02 rad of
    # each other, so a same-length line underneath would never be visible.
    cmd_line, = ax.plot([], [], ls="--", lw=1.6, color=CMD_COLOR, label="command",
                        zorder=3)
    tip, = ax.plot([], [], "o", ms=5, color=JOINT_COLOR, zorder=4)
    ax.legend(loc="lower left", fontsize=8, frameon=False)
    return cmd_line, joint_line, tip


def _slot_axes(ax):
    """Magnified 'pin (motor) in slot (joint)' schematic."""
    ax.set_xlim(-1.9, 1.9)
    ax.set_ylim(-1.6, 1.5)
    ax.set_aspect("equal")
    ax.axis("off")
    wall = 0.22
    left = Rectangle((-1.0 - wall, -0.9), wall, 1.9, color="#c9d9f0")
    right = Rectangle((1.0, -0.9), wall, 1.9, color="#c9d9f0")
    floor = Rectangle((-1.0 - wall, -0.9), 2.0 + 2 * wall, wall, color="#c9d9f0")
    for p in (left, right, floor):
        ax.add_patch(p)
    pin = Circle((0, 0.15), 0.44, color=CMD_COLOR)
    ax.add_patch(pin)
    ax.annotate("", xy=(-1.0, 1.18), xytext=(1.0, 1.18),
                arrowprops=dict(arrowstyle="<->", color="0.45", lw=0.9))
    ax.text(0, 1.24, "gap 2b (=1 deg)", ha="center", fontsize=8, color="0.3")
    ax.text(0, -1.42, "pin (motor) in slot (joint), magnified",
            fontsize=7, color="0.35", ha="center")
    return pin, left, right


def _flank_colors(delta, b, left, right):
    """Darken the wall the pin is pressing against."""
    engaged = "#1f4e9c"
    idle = "#c9d9f0"
    left.set_color(engaged if delta < -0.35 * b else idle)
    right.set_color(engaged if delta > 0.35 * b else idle)


def _animate(csvname, outname, title, right_builder, stride, speed):
    d = load(csvname)
    n = len(d["t"])
    idx = list(range(0, n, stride))
    b = GAP / 2

    fig = plt.figure(figsize=(6.4, 2.72), dpi=100)
    fig.suptitle(title, fontsize=9)
    axp = fig.add_axes([0.02, 0.05, 0.28, 0.82])
    axs = fig.add_axes([0.33, 0.05, 0.26, 0.82])
    cmd_line, joint_line, tip = _pendulum_axes(axp)
    pin, left, right = _slot_axes(axs)
    update_right = right_builder(fig, d)

    L = 0.3

    def draw(k):
        i = idx[k]
        a = d["cmd"][i]
        cmd_line.set_data([0, 1.12 * L * math.sin(a)], [0, -1.12 * L * math.cos(a)])
        a = d["thetaL"][i]
        joint_line.set_data([0, L * math.sin(a)], [0, -L * math.cos(a)])
        tip.set_data([L * math.sin(d["thetaL"][i])], [-L * math.cos(d["thetaL"][i])])
        delta = d["delta"][i]
        pin.center = (max(-1.0, min(1.0, delta / b)) * 0.56, 0.15)
        _flank_colors(delta, b, left, right)
        update_right(i)
        return ()

    anim = FuncAnimation(fig, draw, frames=len(idx), interval=40, blit=False)
    os.makedirs(IMG_DIR, exist_ok=True)
    out = os.path.join(IMG_DIR, outname)
    anim.save(out, writer=PillowWriter(fps=25))
    plt.close(fig)
    print("wrote", os.path.relpath(out, ROOT), f"({len(idx)} frames, {speed}x speed)")


def _phase_right(fig, d):
    ax = fig.add_axes([0.68, 0.16, 0.29, 0.7])
    ax.plot(d["cmd"], d["thetaL"], lw=0.9, color=JOINT_COLOR)
    lim = max(map(abs, d["cmd"])) * 1.08
    ax.plot([-lim, lim], [-lim, lim], ls=":", color="0.7", lw=0.8)
    ax.set_xlabel("command [rad]", fontsize=8)
    ax.set_ylabel("joint [rad]", fontsize=8)
    ax.tick_params(labelsize=7)
    ax.grid(alpha=0.3)
    marker, = ax.plot([], [], "o", ms=5, color=CMD_COLOR)

    def update(i):
        marker.set_data([d["cmd"][i]], [d["thetaL"][i]])
    return update


def _timeseries_right(fig, d, second, second_label, second_ylim=None):
    a0 = fig.add_axes([0.68, 0.55, 0.29, 0.32])
    a1 = fig.add_axes([0.68, 0.14, 0.29, 0.32])
    a0.plot(d["t"], d["thetaL"], color=JOINT_COLOR, lw=1.0)
    if max(d["cmd"]) == min(d["cmd"]):
        a0.axhline(d["cmd"][0], ls="--", color="0.6", lw=0.8)
    else:
        a0.plot(d["t"], d["cmd"], color=CMD_COLOR, lw=0.8)
    a0.set_ylabel("joint [rad]", fontsize=8)
    a1.plot(d["t"], second, color=JOINT_COLOR, lw=1.0)
    a1.axhline(0, color="0.8", lw=0.8)
    a1.set_ylabel(second_label, fontsize=8)
    a1.set_xlabel("t [s]", fontsize=8)
    if second_ylim is not None:
        a1.set_ylim(*second_ylim)
    for a in (a0, a1):
        a.tick_params(labelsize=7)
        a.grid(alpha=0.3)
    m0, = a0.plot([], [], "o", ms=4.5, color=CMD_COLOR)
    m1, = a1.plot([], [], "o", ms=4.5, color=CMD_COLOR)

    def update(i):
        m0.set_data([d["t"][i]], [d["thetaL"][i]])
        m1.set_data([d["t"][i]], [second[i]])
    return update


def anim_hysteresis():
    _animate("hysteresis.csv", "hysteresis_anim.gif",
             "Hysteresis: slow triangle command, response lags by the gap on each reversal (8x speed)",
             _phase_right, stride=8, speed=8)


def anim_gravity_crossing():
    d = load("gravity_crossing.csv")
    vel = joint_velocity(d["thetaL"])
    _animate("gravity_crossing.csv", "gravity_crossing_anim.gif",
             "Gravity crossing: monotonic command, the flank hands over at the bottom (2x speed)",
             lambda fig, dd: _timeseries_right(fig, dd, dd["delta"], "delta [rad]"),
             stride=2, speed=2)
    del vel


def anim_stiction_hold():
    d = load("stiction_hold.csv")
    vel = joint_velocity(d["thetaL"])
    _animate("stiction_hold.csv", "stiction_hold_anim.gif",
             "Stiction hold: step to 0.25 rad under gravity settles with no stick-slip chatter",
             lambda fig, dd: _timeseries_right(fig, dd, vel, "joint vel [rad/s]"),
             stride=2, speed=2)


def anim_stick_slip():
    d = load("stick_slip.csv")
    detr = _detrend(d["t"], [c - t for c, t in zip(d["cmd"], d["thetaL"])])
    _animate("stick_slip.csv", "stick_slip_anim.gif",
             "Stick-slip: 0.07 rad/s sweep, lightly damped servo — stick, wind-up, breakaway (2x speed)",
             lambda fig, dd: _timeseries_right(fig, dd, detr, "judder [rad]",
                                               second_ylim=(-0.0025, 0.0025)),
             stride=2, speed=2)


def main():
    plot_hysteresis()
    plot_gravity_crossing()
    plot_friction_curve()
    plot_stiction_hold()
    plot_stick_slip()
    anim_hysteresis()
    anim_gravity_crossing()
    anim_stiction_hold()
    anim_stick_slip()


if __name__ == "__main__":
    main()

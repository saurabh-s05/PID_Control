import math
import matplotlib.pyplot as plt

from pid import PID
from plants import MassSpringDamper
from simulate import simulate
from metrics import step_metrics
from style import save, HALF

m_, c_, k_ = 1.0, 10.0, 20.0
T_END = 6.0


def run(kp, ki=0.0, kd=0.0):
    # D on measurement: same as de/dt for a constant target, without the spike at t = 0 (see sec 7c)
    pid = PID(kp, ki, kd, d_on_measurement=True)
    log = simulate(MassSpringDamper(m_, c_, k_), pid, 1.0, t_end=T_END)
    return log, step_metrics(log["t"], log["y"])


def overshoot_theory(zeta):
    if zeta >= 1:
        return 0.0
    return 100 * math.exp(-zeta * math.pi / math.sqrt(1 - zeta ** 2))


def plot_set(runs, label_fn, title, name, xlim, zoom=None):
    fig, ax = plt.subplots(figsize=HALF)
    for gain, (log, _) in runs.items():
        ax.plot(log["t"], log["y"], label=label_fn(gain))
    ax.axhline(1.0, color="k", ls="--", lw=1, label="target")
    ax.set_xlim(0, xlim)
    ax.set_title(title)
    ax.set_xlabel("Time [s]")
    ax.set_ylabel("Position [m]")

    if zoom:
        ins = ax.inset_axes([0.42, 0.13, 0.55, 0.32])
        for log, _ in runs.values():
            ins.plot(log["t"], log["y"], lw=1.2)
        ins.axhline(1.0, color="k", ls="--", lw=1)
        ins.set_xlim(*zoom[0])
        ins.set_ylim(*zoom[1])
        ins.tick_params(labelsize=6.5)
        ax.set_ylim(-0.05, 2.0)
        ax.legend(loc="upper right", ncol=3, fontsize=6.5)
    else:
        ax.legend(loc="lower right")
    save(fig, name)


# ---- P only ----
print("P only: final error = k/(k+Kp),  zeta = c / (2*sqrt(k+Kp))")
print(f"{'Kp':>5} {'e_ss th':>8} {'e_ss sim':>9} {'zeta':>6} {'OS th %':>8} {'OS sim %':>9} {'t_r':>6} {'t_s':>6}")
p_runs = {}
for kp in [20, 50, 100, 300]:
    log, m = run(kp)
    p_runs[kp] = (log, m)
    zeta = c_ / (2 * math.sqrt(m_ * (k_ + kp)))
    print(f"{kp:>5} {k_ / (k_ + kp):>8.4f} {m['final_error']:>9.4f} {zeta:>6.3f} "
          f"{overshoot_theory(zeta):>8.1f} {m['overshoot']:>9.1f} {m['rise_time']:>6.3f} {m['settling_time']:>6.3f}")
plot_set(p_runs, lambda g: f"$K_p$ = {g}", "P only: stronger P, smaller gap", "sec5_p_only", 2.0)

# ---- PI ----
print("\nPI with Kp = 300")
print(f"{'Ki':>5} {'e_ss':>8} {'OS %':>6} {'peak':>6} {'t_r':>6} {'t_s':>6}")
pi_runs = {}
for ki in [0, 100, 300, 1000]:
    log, m = run(300, ki)
    pi_runs[ki] = (log, m)
    print(f"{ki:>5} {m['final_error']:>8.4f} {m['overshoot']:>6.1f} {log['y'].max():>6.3f} "
          f"{m['rise_time']:>6.3f} {m['settling_time']:>6.3f}")
plot_set(pi_runs, lambda g: f"$K_i$ = {g}", "PI ($K_p$ = 300): I removes the gap", "sec5_pi", 3.0,
         zoom=((1.0, 6.0), (0.92, 1.02)))

log = pi_runs[300][0]
print(f"PI Ki=300 at t = {T_END:.0f} s: P force = {log['p'][-1]:.2f} N, I force = {log['i'][-1]:.2f} N")
fig, ax = plt.subplots(figsize=HALF)
ax.plot(log["t"], log["p"], label="P force $K_p e$")
ax.plot(log["t"], log["i"], label="I force $K_i \\int e\\,dt$")
ax.axhline(k_, color="k", ls="--", lw=1, label="holding force 20 N")
ax.set_xlim(0, 3.0)
ax.set_ylim(-60, 120)
ax.set_title("PI ($K_p$ = 300, $K_i$ = 300): who pushes?")
ax.set_xlabel("Time [s]")
ax.set_ylabel("Force [N]")
ax.legend(loc="upper right")
save(fig, "sec5_pi_forces")

# ---- PD ----
print("\nPD with Kp = 300: zeta = (c+Kd) / (2*sqrt(k+Kp))")
print(f"{'Kd':>5} {'zeta':>6} {'OS th %':>8} {'OS sim %':>9} {'e_ss':>8} {'t_r':>6} {'t_s':>6}")
pd_runs = {}
for kd in [0, 10, 20, 30]:
    log, m = run(300, 0, kd)
    pd_runs[kd] = (log, m)
    zeta = (c_ + kd) / (2 * math.sqrt(m_ * (k_ + 300)))
    print(f"{kd:>5} {zeta:>6.3f} {overshoot_theory(zeta):>8.1f} {m['overshoot']:>9.1f} "
          f"{m['final_error']:>8.4f} {m['rise_time']:>6.3f} {m['settling_time']:>6.3f}")
plot_set(pd_runs, lambda g: f"$K_d$ = {g}", "PD ($K_p$ = 300): D removes the bouncing", "sec5_pd", 1.5)

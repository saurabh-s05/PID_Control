import numpy as np
import matplotlib.pyplot as plt

from pid import PID
from plants import ThirdOrderLag
from simulate import simulate
from metrics import step_metrics
from style import save

T_OSC = 40.0


def swing(kp):
    """Return (growth ratio of the swings, period) for P-only control."""
    log = simulate(ThirdOrderLag(), PID(kp), 1.0, t_end=T_OSC)
    t, y = log["t"], log["y"]
    peaks = np.where((y[1:-1] > y[:-2]) & (y[1:-1] >= y[2:]))[0] + 1
    valleys = np.where((y[1:-1] < y[:-2]) & (y[1:-1] <= y[2:]))[0] + 1
    n = min(len(peaks), len(valleys))
    amp = y[peaks[:n]] - y[valleys[:n]]
    period = np.diff(t[peaks]).mean()
    return amp[-1] / amp[0], period


# Step 1: raise Kp until the swings neither grow nor shrink (bisection on the growth ratio)
lo, hi = 4.0, 12.0
while hi - lo > 0.01:
    mid = (lo + hi) / 2
    ratio, _ = swing(mid)
    lo, hi = (mid, hi) if ratio < 1 else (lo, mid)
Ku = round((lo + hi) / 2, 2)
_, Tu = swing(Ku)
print(f"Measured: Ku = {Ku:.2f}, Tu = {Tu:.3f} s")
print(f"Theory:   Ku = 8.00, Tu = 2*pi/sqrt(3) = {2 * np.pi / np.sqrt(3):.3f} s\n")

# Step 2: Ziegler-Nichols table (classic PID)
kp_zn = 0.6 * Ku
ti, td = Tu / 2, Tu / 8
ki_zn, kd_zn = kp_zn / ti, kp_zn * td
print(f"Ziegler-Nichols: Kp = 0.6 Ku = {kp_zn:.2f}, Ti = Tu/2 = {ti:.3f} s, Td = Tu/8 = {td:.3f} s")
print(f"                 -> Ki = Kp/Ti = {ki_zn:.3f}, Kd = Kp*Td = {kd_zn:.3f}\n")

fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(7.2, 2.6))
for kp in [4.0, 6.0, Ku]:
    log = simulate(ThirdOrderLag(), PID(kp), 1.0, t_end=25)
    label = f"$K_p$ = {kp:g}" + (" = $K_u$" if kp == Ku else "")
    ax1.plot(log["t"], log["y"], label=label)

t, y = log["t"], log["y"]
peaks = t[np.where((y[1:-1] > y[:-2]) & (y[1:-1] >= y[2:]))[0] + 1]
p1 = peaks[peaks > 12][0]
ax1.annotate("", xy=(p1 + Tu, 1.85), xytext=(p1, 1.85),
             arrowprops=dict(arrowstyle="<->", color="k"))
ax1.text(p1 + Tu / 2, 1.92, f"$T_u$ = {Tu:.2f} s", ha="center", fontsize=8)
ax1.set_ylim(-0.2, 2.6)
ax1.set_title("Step 1: raise P until it swings steadily")
ax1.set_xlabel("Time [s]")
ax1.set_ylabel("Output [-] (no unit)")
ax1.legend(loc="upper left", ncol=3, fontsize=7)

# Step 3: apply the table, then improve it
cases = {
    "Ziegler-Nichols": (kp_zn, ki_zn, kd_zn),
    "cut $K_i$ to 0.8": (kp_zn, 0.8, kd_zn),
    "+ raise $K_d$ to 4.0": (kp_zn, 0.8, 4.0),
}
print(f"{'':<22} {'Kp':>5} {'Ki':>6} {'Kd':>5} {'t_r':>6} {'OS %':>6} {'t_s':>6}")
for name, (kp, ki, kd) in cases.items():
    log = simulate(ThirdOrderLag(), PID(kp, ki, kd, d_on_measurement=True), 1.0, t_end=25)
    m = step_metrics(log["t"], log["y"])
    print(f"{name.replace('$', ''):<22} {kp:>5.2f} {ki:>6.3f} {kd:>5.2f} "
          f"{m['rise_time']:>6.2f} {m['overshoot']:>6.1f} {m['settling_time']:>6.2f}")
    ax2.plot(log["t"], log["y"], label=name)
ax2.axhline(1.0, color="k", ls="--", lw=1)
ax2.set_xlim(0, 15)
ax2.set_title("Steps 2-3: apply the table, then improve")
ax2.set_xlabel("Time [s]")
ax2.set_ylabel("Output [-] (no unit)")
ax2.legend(loc="lower right", fontsize=7)
fig.tight_layout()
save(fig, "sec8_ziegler_nichols")

# A small search also found a slightly faster set; not plotted, kept for comparison
log = simulate(ThirdOrderLag(), PID(4.0, 0.8, 3.0, d_on_measurement=True), 1.0, t_end=25)
m = step_metrics(log["t"], log["y"])
print(f"\nAlternative Kp 4.0, Ki 0.8, Kd 3.0: t_r {m['rise_time']:.2f} s, "
      f"OS {m['overshoot']:.1f} %, t_s {m['settling_time']:.2f} s")

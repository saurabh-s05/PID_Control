import matplotlib.pyplot as plt

from pid import PID
from plants import MassSpringDamper
from simulate import simulate
from metrics import step_metrics
from style import save, FULL

target = 1.0
log = simulate(MassSpringDamper(), PID(kp=200), target, t_end=2.0)
t, y = log["t"], log["y"]
m = step_metrics(t, y, target)
y_final = y[-1]

print("Example: P-only, Kp = 200 N/m")
print(f"  rise time     : {m['rise_time']:.3f} s")
print(f"  overshoot     : {m['overshoot']:.1f} %")
print(f"  settling time : {m['settling_time']:.3f} s")
print(f"  final error   : {m['final_error']:.4f} m")

t10 = t[(y >= 0.1 * y_final).argmax()]
t90 = t[(y >= 0.9 * y_final).argmax()]
i_peak = y.argmax()

fig, ax = plt.subplots(figsize=FULL)
ax.plot(t, y, color="tab:blue", label="position $y$")
ax.axhline(target, color="k", ls="--", lw=1, label="target 1 m")
ax.axhspan(0.98 * y_final, 1.02 * y_final, color="tab:green", alpha=0.15, label="±2 % band")

# rise time
for tt in (t10, t90):
    ax.axvline(tt, color="tab:orange", ls=":", lw=1)
ax.annotate("", xy=(t90, 0.25), xytext=(t10, 0.25),
            arrowprops=dict(arrowstyle="<->", color="tab:orange"))
ax.text(t90 + 0.02, 0.22, f"rise time\n{m['rise_time']:.3f} s", color="tab:orange", fontsize=8)

# overshoot
ax.annotate("", xy=(t[i_peak], y[i_peak]), xytext=(t[i_peak], y_final),
            arrowprops=dict(arrowstyle="<->", color="tab:red"))
ax.text(t[i_peak] + 0.03, (y[i_peak] + y_final) / 2, f"overshoot {m['overshoot']:.1f} %",
        color="tab:red", fontsize=8)

# settling time
ax.axvline(m["settling_time"], color="tab:green", ls="--", lw=1)
ax.text(m["settling_time"] + 0.02, 0.45, f"settling time\n{m['settling_time']:.2f} s",
        color="tab:green", fontsize=8)

# final error
ax.plot([1.85, 1.85], [y_final, target], color="tab:purple", lw=2, marker="_", ms=10)
ax.text(1.52, 1.06, f"final error {m['final_error']:.3f} m", color="tab:purple", fontsize=8)

ax.set_title("The four measures on one step response (P-only, $K_p$ = 200 N/m)")
ax.set_xlabel("Time [s]")
ax.set_ylabel("Position [m]")
ax.set_xlim(0, 2.0)
ax.legend(loc="lower right")
save(fig, "sec4_metrics")

# Why overshoot is measured from the final value, not from the target
log = simulate(MassSpringDamper(), PID(kp=50), target, t_end=3.0)
peak, final = log["y"].max(), log["y"][-1]
print(f"\nP-only Kp = 50: peak {peak:.3f} m, final {final:.3f} m")
print(f"  overshoot vs target      : {max(0, peak - target) / target * 100:.1f} %  (hides the bounce)")
print(f"  overshoot vs final value : {(peak - final) / final * 100:.1f} %")

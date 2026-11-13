import matplotlib.pyplot as plt

from pid import PID
from plants import MassSpringDamper
from simulate import simulate
from metrics import step_metrics
from style import save

def make_pid(ki):
    return PID(kp=300, ki=ki, kd=20, dt=0.001,
               u_min=-30, u_max=30, anti_windup=True,
               d_on_measurement=True, d_filter_tau=0.01)


def target(t):
    return 1.0 if t < 4.0 else 0.5


# Ki = 200 from section 6 is too slow once the motor is limited
old = simulate(MassSpringDamper(), make_pid(200), target, t_end=4.0, noise=0.001, seed=1)
print(f"With Ki = 200 the block is still {(1 - old['y'][-1]) * 1000:.1f} mm short at t = 4 s -> raise Ki\n")

controller = make_pid(600)
log = simulate(MassSpringDamper(), controller, target, t_end=8.0, noise=0.001, seed=1)
t, y, u = log["t"], log["y"], log["u"]

print("Final PID on the block: 30 N motor, +/-1 mm sensor noise, 1 ms update")
print("Gains: Kp = 300, Ki = 600, Kd = 20 | anti-windup, D on measurement, D filter 10 ms\n")

first = t < 4.0
m1 = step_metrics(t[first], y[first], target=1.0)
second = t >= 4.0
y2 = 1.0 - y[second]
m2 = step_metrics(t[second] - 4.0, y2, target=0.5)

for name, m in [("Step 0 -> 1 m  ", m1), ("Step 1 -> 0.5 m", m2)]:
    print(f"{name}: rise {m['rise_time']:.3f} s | overshoot {m['overshoot']:.1f} % | "
          f"settling {m['settling_time']:.2f} s | final error {abs(m['final_error']) * 1000:.2f} mm")
print(f"\nMax |force| = {abs(u).max():.1f} N (limit 30 N)")
print(f"Force at rest (t = 3.5-4 s): mean {u[(t > 3.5) & first].mean():.1f} N "
      f"(spring needs 20 N at 1 m)")

fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(7.2, 2.5))
ax1.plot(t, y, label="position")
ax1.plot(t, log["r"], "k--", lw=1, label="target")
ax1.set_title("Final PID: position")
ax1.set_ylabel("Position [m]")
ax2.plot(t, u, color="tab:red", lw=0.8, label="motor force")
ax2.axhline(30, color="k", ls=":", lw=1, label="±30 N limit")
ax2.axhline(-30, color="k", ls=":", lw=1)
ax2.set_title("Final PID: motor force")
ax2.set_ylabel("Force [N]")
ax1.legend(loc="upper right", fontsize=7)
ax2.legend(loc="lower right", fontsize=7)
for ax in (ax1, ax2):
    ax.set_xlabel("Time [s]")
fig.tight_layout()
save(fig, "sec9_final_pid")

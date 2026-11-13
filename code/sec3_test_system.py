import numpy as np
import matplotlib.pyplot as plt

from plants import MassSpringDamper
from metrics import step_metrics
from style import save, HALF

m, c, k = 1.0, 10.0, 20.0
dt, t_end = 0.0005, 3.0

poles = np.roots([m, c, k])
print(f"Plant: {m} x'' + {c} x' + {k} x = F")
print(f"Open-loop poles: {poles[0]:.3f}, {poles[1]:.3f}  (both real -> no oscillation)")
print(f"Damping ratio zeta = {c / (2 * np.sqrt(k * m)):.3f}")
print(f"Force needed to hold x = 1 m: F = k * 1 = {k:.0f} N\n")

fig, ax = plt.subplots(figsize=HALF)
t = np.arange(0, t_end + dt, dt)

for force in [10, 20, 30]:
    plant = MassSpringDamper(m, c, k)
    x = np.zeros_like(t)
    for i in range(1, len(t)):
        x[i] = plant.step(force, dt)
    print(f"Constant F = {force:2d} N  ->  final x = {x[-1]:.3f} m  (theory F/k = {force / k:.3f} m)")
    ax.plot(t, x, label=f"F = {force} N")
    if force == 20:
        res = step_metrics(t, x)
        print(f"   with 20 N: rise time {res['rise_time']:.2f} s, settling time {res['settling_time']:.2f} s")

# What if the real spring is stiffer than we think?
plant = MassSpringDamper(m, c, 25.0)
for _ in range(int(5 / dt)):
    x_wrong = plant.step(20, dt)
print(f"\nIf the real spring is 25 N/m, 20 N only reaches x = {x_wrong:.2f} m")

ax.axhline(1.0, color="k", ls="--", lw=1, label="target 1 m")
ax.set_title("Open loop: block under a constant force")
ax.set_xlabel("Time [s]")
ax.set_ylabel("Position x [m]")
ax.set_ylim(-0.05, 1.95)
ax.legend(loc="upper left", ncol=2)
save(fig, "sec3_open_loop")

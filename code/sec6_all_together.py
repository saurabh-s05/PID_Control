import matplotlib.pyplot as plt

from pid import PID
from plants import MassSpringDamper
from simulate import simulate
from metrics import step_metrics
from style import save

KP, KI, KD = 300, 200, 20

controllers = {
    "P":   (KP, 0, 0),
    "PI":  (KP, KI, 0),
    "PD":  (KP, 0, KD),
    "PID": (KP, KI, KD),
}

fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(7.2, 2.7), gridspec_kw={"width_ratios": [1.4, 1]})

print(f"Gains: Kp = {KP} N/m, Ki = {KI} N/(m s), Kd = {KD} N s/m\n")
print(f"{'':>4} {'t_r [s]':>8} {'OS [%]':>7} {'t_s [s]':>8} {'e_ss [mm]':>10} {'max F [N]':>10}")
for name, (kp, ki, kd) in controllers.items():
    log = simulate(MassSpringDamper(), PID(kp, ki, kd, d_on_measurement=True), 1.0, t_end=6.0)
    m = step_metrics(log["t"], log["y"])
    print(f"{name:>4} {m['rise_time']:>8.3f} {m['overshoot']:>7.1f} {m['settling_time']:>8.3f} "
          f"{m['final_error'] * 1000:>10.2f} {abs(log['u']).max():>10.0f}")
    ax1.plot(log["t"], log["y"], label=name, lw=2.2 if name == "PID" else 1.4)
    if name == "PID":
        ax2.plot(log["t"], log["u"], color="tab:red", label="PID force")

ax1.axhline(1.0, color="k", ls="--", lw=1, label="target")
ax1.set_xlim(0, 1.5)
ax1.set_title("P, PI, PD and PID on the same block")
ax1.set_xlabel("Time [s]")
ax1.set_ylabel("Position [m]")
ax1.legend(loc="lower right", ncol=2)

ax2.axhline(20, color="k", ls="--", lw=1, label="holding force 20 N")
ax2.set_xlim(0, 0.5)
ax2.set_title("Force asked from the motor (PID)")
ax2.set_xlabel("Time [s]")
ax2.set_ylabel("Force [N]")
ax2.legend(loc="upper right")
fig.tight_layout()
save(fig, "sec6_compare")

print(f"\nChoosing Ki (Kp = {KP}, Kd = {KD}):")
for ki in [100, 200, 300, 500]:
    log = simulate(MassSpringDamper(), PID(KP, ki, KD, d_on_measurement=True), 1.0, t_end=6.0)
    m = step_metrics(log["t"], log["y"])
    print(f"  Ki = {ki:<4} OS {m['overshoot']:4.1f} %, t_s {m['settling_time']:.2f} s, "
          f"error after 6 s {m['final_error'] * 1000:.2f} mm")

print("\nRobustness of the chosen PID (interview checks):")
for label, plant in [("m = 0.5 kg", MassSpringDamper(m=0.5)),
                     ("m = 2 kg", MassSpringDamper(m=2.0)),
                     ("k = 40 N/m", MassSpringDamper(k=40.0))]:
    log = simulate(plant, PID(KP, KI, KD, d_on_measurement=True), 1.0, t_end=6.0)
    m = step_metrics(log["t"], log["y"])
    print(f"  {label:<11} t_r {m['rise_time']:.3f} s, OS {m['overshoot']:.1f} %, "
          f"t_s {m['settling_time']:.3f} s, e_ss {m['final_error'] * 1000:.2f} mm")

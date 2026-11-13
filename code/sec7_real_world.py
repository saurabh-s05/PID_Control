import matplotlib.pyplot as plt

from pid import PID
from plants import MassSpringDamper
from simulate import simulate
from metrics import step_metrics
from style import save

KP, KI, KD = 300, 200, 20
PAIR = (7.2, 2.4)


def pid(**kw):
    kw.setdefault("d_on_measurement", True)
    return PID(KP, KI, KD, **kw)


def report(tag, log, start=0.0):
    sel = log["t"] >= start
    m = step_metrics(log["t"][sel] - start, log["y"][sel])
    print(f"  {tag:<28} t_r {m['rise_time']:.3f} s | OS {m['overshoot']:5.1f} % | "
          f"t_s {m['settling_time']:.2f} s | max |F| {abs(log['u']).max():7.0f} N")
    return m


def finish(fig, axes, name):
    for ax in axes:
        ax.set_xlabel("Time [s]")
        ax.legend(loc="best")
    fig.tight_layout()
    save(fig, name)


# (a) Actuator limit -> integral windup
print("(a) Motor limited to +/-30 N")
free = simulate(MassSpringDamper(), pid(), 1.0, t_end=5)
wind = simulate(MassSpringDamper(), pid(u_min=-30, u_max=30), 1.0, t_end=5)
fixed = simulate(MassSpringDamper(), pid(u_min=-30, u_max=30, anti_windup=True), 1.0, t_end=5)
report("no limit", free)
report("30 N limit, no anti-windup", wind)
report("30 N limit, anti-windup", fixed)
print(f"  peak I force: {wind['i'].max():.1f} N without fix, {fixed['i'].max():.1f} N with fix")

fig, (ax1, ax2) = plt.subplots(1, 2, figsize=PAIR)
ax1.plot(free["t"], free["y"], color="gray", ls=":", label="no limit")
ax1.plot(wind["t"], wind["y"], color="tab:red", label="limit, no fix")
ax1.plot(fixed["t"], fixed["y"], color="tab:blue", label="limit + anti-windup")
ax1.axhline(1.0, color="k", ls="--", lw=1)
ax1.set_title("Position with a 30 N motor")
ax1.set_ylabel("Position [m]")
ax2.plot(wind["t"], wind["i"], color="tab:red", label="no fix")
ax2.plot(fixed["t"], fixed["i"], color="tab:blue", label="anti-windup")
ax2.axhline(20, color="k", ls="--", lw=1, label="needed: 20 N")
ax2.set_title("I part of the force")
ax2.set_ylabel("I force [N]")
finish(fig, (ax1, ax2), "sec7a_windup")

# (b) Noisy sensor -> jumpy D
print("\n(b) Sensor noise +/-1 mm")
raw = simulate(MassSpringDamper(), pid(), 1.0, t_end=2, noise=0.001)
filt = simulate(MassSpringDamper(), pid(d_filter_tau=0.01), 1.0, t_end=2, noise=0.001)
for tag, log in [("raw D", raw), ("D filtered, tau = 10 ms", filt)]:
    sel = log["t"] > 1.0
    report(tag, log)
    print(f"    force jitter after settling: std {log['u'][sel].std():.2f} N, "
          f"range {log['u'][sel].min():.1f} .. {log['u'][sel].max():.1f} N")

fig, (ax1, ax2) = plt.subplots(1, 2, figsize=PAIR, sharey=True)
ax1.plot(raw["t"], raw["u"], color="tab:red", lw=0.8, label="raw D")
ax2.plot(filt["t"], filt["u"], color="tab:blue", lw=0.8, label="filtered D")
for ax in (ax1, ax2):
    ax.axhline(20, color="k", ls="--", lw=1, label="20 N")
    ax.set_xlim(0.5, 2.0)
    ax.set_ylim(-40, 80)
ax1.set_title("Problem: motor force with raw D")
ax2.set_title("Fix: low-pass filter on D (10 ms)")
ax1.set_ylabel("Force [N]")
finish(fig, (ax1, ax2), "sec7b_noise")

# (c) Setpoint jump -> derivative kick
print("\n(c) Target jumps 0 -> 1 m at t = 0.5 s")
T_JUMP = 0.5
target = lambda t: 1.0 if t >= T_JUMP else 0.0
kick = simulate(MassSpringDamper(), pid(d_on_measurement=False), target, t_end=2)
smooth = simulate(MassSpringDamper(), pid(), target, t_end=2)
report("D on error", kick, T_JUMP)
report("D on measurement", smooth, T_JUMP)
print(f"  expected spike = Kd * jump / dt + Kp * jump = {KD} / 0.001 + {KP} = {KD / 0.001 + KP:.0f} N")

fig, (ax1, ax2) = plt.subplots(1, 2, figsize=PAIR)
ax1.plot(kick["t"], kick["u"], color="tab:red", label="D on error")
ax1.plot(smooth["t"], smooth["u"], color="tab:blue", label="D on measurement")
ax1.set_ylim(-150, 450)
ax1.annotate(f"spike {abs(kick['u']).max():,.0f} N\n(off the chart)", xy=(T_JUMP, 440),
             xytext=(0.75, 330), fontsize=7.5, color="tab:red",
             arrowprops=dict(arrowstyle="->", color="tab:red"))
ax1.set_title("Force when the target jumps")
ax1.set_ylabel("Force [N]")
ax2.plot(kick["t"], kick["y"], color="tab:red", label="D on error")
ax2.plot(smooth["t"], smooth["y"], color="tab:blue", label="D on measurement")
ax2.plot(kick["t"], kick["r"], color="k", ls="--", lw=1, label="target")
ax2.set_title("Position")
ax2.set_ylabel("Position [m]")
finish(fig, (ax1, ax2), "sec7c_kick")

# (d) Controller updates too slowly
print("\n(d) Slow update rate")
fig, (ax1, ax2) = plt.subplots(1, 2, figsize=PAIR)
for dt in [0.001, 0.040, 0.060, 0.080]:
    log = simulate(MassSpringDamper(), pid(dt=dt), 1.0, t_end=3)
    report(f"dt = {dt * 1000:.0f} ms", log)
    ax1.plot(log["t"], log["y"], label=f"{dt * 1000:.0f} ms")
ax1.axhline(1.0, color="k", ls="--", lw=1)
ax1.set_ylim(-0.1, 2.0)
ax1.set_title("Same gains, slower updates")
ax1.set_ylabel("Position [m]")

ref = simulate(MassSpringDamper(), pid(), 1.0, t_end=3)
slow = simulate(MassSpringDamper(), pid(dt=0.06), 1.0, t_end=3)
retuned = simulate(MassSpringDamper(), PID(100, 100, 8, dt=0.06, d_on_measurement=True), 1.0, t_end=3)
report("60 ms, retuned 100/100/8", retuned)
ax2.plot(ref["t"], ref["y"], color="gray", ls=":", label="1 ms, original")
ax2.plot(slow["t"], slow["y"], color="tab:red", label="60 ms, original")
ax2.plot(retuned["t"], retuned["y"], color="tab:blue", label="60 ms, retuned")
ax2.axhline(1.0, color="k", ls="--", lw=1)
ax2.set_title("Fix at 60 ms: gentler gains")
ax2.set_ylabel("Position [m]")
finish(fig, (ax1, ax2), "sec7d_sample_time")

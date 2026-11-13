import numpy as np


def simulate(plant, controller, setpoint, t_end=5.0, sim_dt=0.0005,
             noise=0.0, seed=0):
    """Run the closed loop and return a dict of time histories.

    setpoint : number, or a function of time r(t)
    noise    : sensor noise amplitude, uniform in [-noise, +noise]
    The plant is integrated every sim_dt; the controller only runs every
    controller.dt and holds its output in between, like a real microcontroller.
    """
    if controller.dt < sim_dt:
        raise ValueError("controller.dt must not be smaller than sim_dt")

    rng = np.random.default_rng(seed)
    r_func = setpoint if callable(setpoint) else (lambda t: setpoint)

    plant.reset()
    controller.reset()

    n = int(round(t_end / sim_dt))
    ctrl_every = max(1, int(round(controller.dt / sim_dt)))

    log = {k: np.zeros(n + 1) for k in ("t", "y", "y_meas", "r", "u", "p", "i", "d")}
    u = 0.0
    y_meas = 0.0

    for k in range(n + 1):
        t = k * sim_dt
        y = plant.output()
        r = r_func(t)

        if k % ctrl_every == 0:
            y_meas = y + (rng.uniform(-noise, noise) if noise > 0 else 0.0)
            u = controller.update(r, y_meas)

        log["t"][k] = t
        log["y"][k] = y
        log["y_meas"][k] = y_meas
        log["r"][k] = r
        log["u"][k] = u
        log["p"][k] = controller.p_term
        log["i"][k] = controller.i_term
        log["d"][k] = controller.d_term

        if k < n:
            plant.step(u, sim_dt)

    return log


if __name__ == "__main__":
    from pid import PID
    from plants import MassSpringDamper
    from metrics import step_metrics

    print("P-only check on the mass-spring-damper (target 1 m)")
    print(f"{'Kp':>5} {'final err sim':>14} {'theory 20/(20+Kp)':>18}")
    for kp in [20, 50, 100, 300]:
        log = simulate(MassSpringDamper(), PID(kp), 1.0, t_end=5)
        m = step_metrics(log["t"], log["y"])
        print(f"{kp:>5} {m['final_error']:>14.4f} {20 / (20 + kp):>18.4f}")

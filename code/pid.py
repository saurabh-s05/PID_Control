import math


class PID:
    """Discrete PID controller.

    Optional features (all off by default so the plain PID can be studied first):
      u_min, u_max     - actuator limits, output is clipped to this range
      anti_windup      - stop integrating while the output is saturated
      d_on_measurement - differentiate -y instead of e, removes derivative kick
      d_filter_tau     - time constant [s] of a low-pass filter on the D term
    """

    def __init__(self, kp, ki=0.0, kd=0.0, dt=0.001,
                 u_min=-math.inf, u_max=math.inf, anti_windup=False,
                 d_on_measurement=False, d_filter_tau=0.0):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.dt = dt
        self.u_min = u_min
        self.u_max = u_max
        self.anti_windup = anti_windup
        self.d_on_measurement = d_on_measurement
        self.d_filter_tau = d_filter_tau
        self.reset()

    def reset(self):
        self.integral = 0.0
        self.prev_error = 0.0
        self.prev_y = None
        self.d_filtered = 0.0
        self.p_term = self.i_term = self.d_term = 0.0

    def update(self, setpoint, y):
        error = setpoint - y

        # Assume the system was resting before the first sample
        if self.prev_y is None:
            self.prev_y = y

        if self.d_on_measurement:
            d_raw = -(y - self.prev_y) / self.dt
        else:
            d_raw = (error - self.prev_error) / self.dt

        if self.d_filter_tau > 0:
            alpha = self.d_filter_tau / (self.d_filter_tau + self.dt)
            self.d_filtered = alpha * self.d_filtered + (1 - alpha) * d_raw
        else:
            self.d_filtered = d_raw

        self.p_term = self.kp * error
        self.d_term = self.kd * self.d_filtered

        new_integral = self.integral + error * self.dt
        u_unclipped = self.p_term + self.ki * new_integral + self.d_term
        u = min(max(u_unclipped, self.u_min), self.u_max)

        # Clamping anti-windup: freeze the integral if it would push further into saturation
        saturated = u != u_unclipped
        pushing_further = (u_unclipped > self.u_max and error > 0) or \
                          (u_unclipped < self.u_min and error < 0)
        if not (self.anti_windup and saturated and pushing_further):
            self.integral = new_integral

        self.i_term = self.ki * self.integral
        self.prev_error = error
        self.prev_y = y
        return u

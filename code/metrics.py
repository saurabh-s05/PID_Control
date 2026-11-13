import numpy as np


def step_metrics(t, y, target=1.0, band=0.02):
    """Four measures of a step response from 0 to target.

    rise time      : time to go from 10% to 90% of the final value
    overshoot      : how far the peak goes past the final value, in % of final value
    settling time  : time after which the response stays within +/- band of its final value
    final error    : target minus final value
    """
    y_final = y[-1]

    def first_crossing(level):
        idx = np.argmax(y >= level)
        return t[idx] if y[idx] >= level else np.nan

    rise_time = first_crossing(0.9 * y_final) - first_crossing(0.1 * y_final)

    overshoot = max(0.0, (y.max() - y_final) / y_final * 100)

    outside = np.abs(y - y_final) > band * abs(y_final)
    settling_time = t[np.where(outside)[0][-1] + 1] if outside.any() else 0.0

    return {
        "rise_time": rise_time,
        "overshoot": overshoot,
        "settling_time": settling_time,
        "final_error": target - y_final,
    }

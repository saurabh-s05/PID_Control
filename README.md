# PID Control — Simulation Code

Python code for the report *PID Control: A Beginner's Guide*. The PID controller, the plant models and
the simulator are written from scratch with NumPy and Matplotlib only — no control library.

## Setup

```bash
python3 -m venv .venv
source .venv/bin/activate          # Windows: .venv\Scripts\activate
pip install -r requirements.txt
```

## Recreate every graph

```bash
cd code
python run_all.py
```

This runs every section script in order (about 1 minute), prints all the numbers used in the report,
and saves every figure to `report/figures/`. All runs are deterministic (the sensor noise uses a fixed
seed), so you get exactly the same graphs and numbers.

Each section can also be run on its own:

| Script | Report section | Figures produced |
|---|---|---|
| `sec3_test_system.py` | 3. The test system | `sec3_open_loop.png` |
| `sec4_metrics.py` | 4. How to judge a result | `sec4_metrics.png` |
| `sec5_each_part.py` | 5. Each part alone | `sec5_p_only.png`, `sec5_pi.png`, `sec5_pi_forces.png`, `sec5_pd.png` |
| `sec6_all_together.py` | 6. All together | `sec6_compare.png` |
| `sec7_real_world.py` | 7. Real-world problems | `sec7a_windup.png`, `sec7b_noise.png`, `sec7c_kick.png`, `sec7d_sample_time.png` |
| `sec8_tuning.py` | 8. Choosing the gains (Ziegler–Nichols) | `sec8_ziegler_nichols.png` |
| `sec9_final_pid.py` | 9. The code | `sec9_final_pid.png` |

`python simulate.py` runs a quick self-check: P-only final error vs. the formula `20 / (20 + Kp)`.

## Code layout

| File | Purpose |
|---|---|
| `pid.py` | Reusable `PID` class. Optional fixes: output limits, anti-windup, derivative on measurement, derivative filter. |
| `plants.py` | `MassSpringDamper` (m = 1 kg, c = 10 N·s/m, k = 20 N/m) and `ThirdOrderLag` = 1/(s+1)³, integrated with RK4. |
| `simulate.py` | Closed-loop simulation. The plant is integrated every `sim_dt`; the controller runs every `controller.dt` and holds its output in between (zero-order hold). |
| `metrics.py` | Rise time (10–90 %), overshoot, settling time (±2 %), final error. |
| `style.py` | Shared figure size/fonts and the save path. |

## Using the PID on your own system

```python
from pid import PID

pid = PID(kp=300, ki=600, kd=20, dt=0.001,
          u_min=-30, u_max=30, anti_windup=True,
          d_on_measurement=True, d_filter_tau=0.01)

while True:
    y = read_sensor()
    u = pid.update(setpoint, y)
    apply_output(u)
    wait(0.001)
```

## Trying changes

Edit the numbers in a script and rerun it, for example:

- **Gains**: `KP, KI, KD` at the top of `sec6_all_together.py` / `sec7_real_world.py`
- **Mass**: `MassSpringDamper(m=2.0)`
- **Noise**: `noise=0.001` in the `simulate(...)` call (amplitude in metres)
- **Force limit**: `u_min=-30, u_max=30` in `PID(...)`
- **Update rate**: `dt=0.001` in `PID(...)`

## Notes

- Tested with Python 3.14, NumPy 2.5 and Matplotlib 3.11 (any Python 3.9+ should work).
- The diagrams in the report (feedback loop, PID branches, discrete loop) are drawn in LaTeX/TikZ;
  the mass–spring–damper drawing (`mass_spring_damper.png`) was made separately. Every *graph* comes from this code.

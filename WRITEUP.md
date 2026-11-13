# Write-up: approach, results and next steps

## Approach

I wrote everything from scratch so I could see every step: a state-space plant integrated with RK4,
a discrete PID class, a simulator where the controller runs at its own update rate, and a function for
the four measures (rise time, overshoot, settling time, final error). One script per report section
prints its numbers and saves its figures, so every number in the report can be traced to an output.

Before tuning anything I checked the simulator against theory on the 1 kg block (c = 10 N·s/m,
k = 20 N/m). With P only, the block must stop short by `20 / (20 + Kp)` because P only pushes when there
is an error, and the 20 N spring force has to come from somewhere. Adding D works like adding to the
damper, so the damping ratio becomes `ζ = (10 + Kd) / (2·√(20 + Kp))`. Once the simulation matched these,
I tuned P → D → I, then added the real-world problems one by one.

## What worked

- **The simulator matches the hand calculations.** The P-only final error matches `20/(20+Kp)` exactly for
  Kp = 20, 50, 100 and 300, and overshoot agrees within 0.6 percentage points. For Ziegler–Nichols the
  measured ultimate gain and period were Ku = 7.99 and Tu = 3.630 s against 8 and 3.628 s by hand.
- **Tuning one gain at a time.** Kp = 300, Kd = 20, Ki = 200 gave 0.5 % overshoot, 0.24 s settling and
  0.03 mm final error on the ideal system, against 40.7 %, 0.78 s and 62.5 mm with P alone.
- **The four fixes each did one clear job:**
  | Problem | Without fix | With fix |
  |---|---|---|
  | 30 N motor limit (windup) | 10.4 % overshoot, I winds up to 57 N | 0 % overshoot |
  | ±1 mm sensor noise | force jitter 17.4 N (std) | 1.3 N with a 10 ms D filter |
  | target jump (derivative kick) | 20 300 N spike | 300 N with D on measurement |
  | 60 ms updates | 21.5 % overshoot | 0 % with gentler gains (100/100/8) |
- **Improving Ziegler–Nichols** by reasoning, not trial and error: its PID overshot by 52.5 %; cutting Ki
  (too much I overshoots) and raising Kd (D damps) brought it to 1.0 % and settling from 9.9 s to 4.1 s.
- **Final controller with everything switched on** (30 N limit, noise, all fixes, Ki = 600): 0 % overshoot,
  0.78 s settling and 0.04 mm error for a 1 m step, and no kick when the target changes to 0.5 m.

## What didn't work (and what I changed)

- **Overshoot measured from the target hid the bouncing.** With P only (Kp = 50) the block peaks at
  0.784 m and settles at 0.714 m, so "overshoot past 1 m" was 0 %. I switched to measuring it from the
  final value (9.7 %), which is also what the damping-ratio formula predicts.
- **D on the error broke the match with theory.** At the moment of a step it gives a huge one-sample
  spike, so the PD results no longer followed the damping formula. I compute D from the measurement
  instead; Section 7c of the report shows the spike itself.
- **My first gain search gave nonsense.** A tiny change in Kd (2.0 → 2.18) appeared to change overshoot
  from about 1 % to over 30 %. The simulation step was 2 ms but the controller assumed 1 ms, so the
  integral and slope were computed wrongly. `simulate.py` now refuses that combination.
- **Gains tuned on the ideal system were too slow on the real one.** With the 30 N limit and anti-windup,
  Ki = 200 left the block 5.4 mm short after 4 s, because I stays frozen while the motor is at its limit.
  Raising Ki to 600 fixed it; anti-windup is what makes the larger Ki safe.
- **Slow updates fail suddenly, not gradually.** 40 ms was almost as good as 1 ms (1.3 % vs 0.5 %
  overshoot), 60 ms gave 21.5 % and 80 ms was unstable. I did not expect such a sharp edge.
- **The gains are not robust to big plant changes.** The same PID gives 10.1 % overshoot on a 2 kg block,
  and a 40 N/m spring makes settling take 1.9 s instead of 0.24 s. A motor weaker than 20 N can never
  hold the block at 1 m at all (15 N can only hold it at 15/20 = 0.75 m), whatever the gains.

## What I would explore next

- **Feedforward of the holding force.** The force needed at rest is known: `k × target`. Adding it
  directly would leave I only the job of correcting model errors, so it could be smaller and slower to wind up.
- **Back-calculation anti-windup**, compared with the clamping I used, since clamping makes the integral
  start late after saturation.
- **Frequency-domain analysis** (Bode plot, gain and phase margin) to predict the 60–80 ms instability
  instead of finding it by simulation.
- **A more realistic plant**: friction, backlash, motor dynamics, sensor delay and Gaussian noise, and
  a robustness check over a range of masses and springs instead of a few single cases.
- **Tuning by optimisation**: minimise a cost such as ITAE under the 30 N limit and compare the result with
  hand tuning and Ziegler–Nichols.
- **Real hardware**: run the same `PID` class on a DC motor with an encoder and compare with the simulation.
- **Unit tests** for the `PID` class (step, saturation, filter, kick) so changes cannot silently break it.

# Lab 7 — Go-to-goal: driving the digits **78**

The robot uses its odometry `(x, y, θ)` to drive a list of waypoints that draw the last two digits of my student ID, **7** and **8**. The same go-to-goal program runs in the TRIK Studio 2D simulator and on the real TRIK robot, and the two odometry logs are compared with the planned path in Google Colab.

## Repository contents

| File | Content |
|---|---|
| `go_to_goal_sim (1).qrs` | TRIK Studio program for the 2D simulator. It writes `gotogol_data_sim.txt`. |
| `go_to_goal_real (2).qrs` | The same program and gains for the real TRIK robot. It writes `gotogol_data_real.txt`. |
| `gotogol_data_sim.txt` | Raw simulator log: `x,y,time` (cm, cm, ms), no header. 2715 samples. |
| `gotogol_data_real.txt` | Raw real-robot log, same format. 466 samples. |
| `gotogoal_analisys.ipynb` | Colab notebook: trajectory plot, waypoint-miss table, run times. |

## Waypoints (cm)

The robot starts at the top-left corner of the 7, at `(0, 0)`, facing +x. +y is to the robot's left, so the digits are upright both in the simulator and in the plots.

```
digit 7: (0,0) → (25,0) → (5,-40)
gap:     (5,-40) → (40,-20)
digit 8: (40,-20) → (40,-40) → (65,-40) → (65,-20) → (40,-20) → (40,0) → (65,0) → (65,-20)
```

The path has 10 goals and 265 cm in total. The 8 is two stacked squares drawn as one continuous path, starting and ending on its middle bar. Besides the 90° corners of the 8, there are three sharp turns: −117° at the top of the 7, +146° at its bottom, and −120° entering the 8.

## Program

Both `.qrs` files have the same block diagram:

```
Start → Clear Encoder E3,E4 → Remove File (old log) → Init
  while i < N:                                  (one pass per waypoint)
     while rho > eps:                           (drive to waypoint i)
        odometry + controller                   (expression block)
        if |eTh| > eTurn:  v = 0                (turn in place)
        motor power: scale + dead-zone lift     (expression block)
        M3 = pL,  M4 = pR
        Write To File: x,y,time
        Timer 5 ms
     i = i + 1
Stop Motors → End
```

**Odometry** (encoders in degrees; M3/E3 = left wheel, M4/E4 = right wheel):

```
dL = Δenc3 · π/180 · rw         dR = Δenc4 · π/180 · rw
dS = (dL + dR) / 2              dθ = (dR − dL) / base
x += dS · cos(θ + dθ/2)         y += dS · sin(θ + dθ/2)         θ += dθ
```

**Controller.** This is a proportional go-to-goal controller, following the lecture's table:

| Quantity | Implementation |
|---|---|
| Distance to goal | `rho = sqrt(dx² + dy²)` |
| Bearing to goal | `thG = atan2(dy, dx)` |
| Heading error, wrapped into (−π, π] | `eTh = atan2(sin(thG − θ), cos(thG − θ))` |
| Linear command | `v = max(vMin, min(Kv · rho, vMax))` |
| Angular command | `w = max(−wMax, min(wMax, Kth · eTh))` |
| Turn-then-drive | `v = 0` while `\|eTh\| > eTurn` |
| Wheel commands | `pL = kScale · (v − w)`, `pR = kScale · (v + w)` |
| Dead-zone lift | `p → sgn(p) · (dz + \|p\| · (100 − dz) / 100)` |
| Waypoint switching | next waypoint when `rho < eps` |

### Parameters

The controller gains are identical in both programs:

| Gain | Value | Meaning |
|---|---|---|
| `Kv` | 3 %/cm | forward command per cm of distance (full speed beyond ≈ 17 cm) |
| `Kth` | 60 %/rad | turning command per rad of heading error |
| `vMax` / `vMin` | 50 % / 15 % | forward command limits |
| `wMax` | 40 % | turning command limit |
| `eps` (ε) | 2 cm | acceptance radius |
| `eTurn` | 0.3 rad (17°) | turn in place above this heading error |

Robot and motor constants differ, because they describe the hardware rather than the controller:

| Constant | Simulator | Real robot | Source |
|---|---|---|---|
| Wheel radius `rw` | 2.8 cm | **4.2 cm** | 2D model: 16 px wheel = 5.6 cm; real: reference program `Fixed_1x1_trajectory.qrs` (r = 0.042 m) |
| Track width `base` | 15.4 cm | 19.4 cm | 2D model: wheels 44 px apart; real: course code |
| Dead-zone lift `dz` | 20 % | 15 % | Lab 5: wheels do not turn below ≈ 5–12 % PWM in the air, more on the floor. 20 % made the real robot shake. |
| Motor scale `kScale` | 1 | 0.6 | Lab 5: 1 % PWM moves a real wheel ≈ 2.4× faster than a simulated one. Without scaling, the real robot oscillated. |
| Log file | `C:\Users\user\Documents\robotics\gotogol_data_sim.txt` | `gotogol_data_real.txt` (on the robot) | |

On the real robot, the log values are first copied into plain variables (`lx`, `ly`, `lt`) before Write To File. When the program is uploaded as generated JavaScript, TRIK drops the brackets in `.. (time() - t0) ..`, which turned every log line into `NaN`.

## Results


### 1. Waypoint-miss table

Miss = closest approach of the logged trajectory to each waypoint (values from `gotogoal_analisys.ipynb`). The corner behaviour was read from the trajectories: *cut* means the path turned inside the corner, *overshoot* means it went past the corner to the outside.

| # | Waypoint | Turn | Sim miss, cm | Sim corner | Real miss, cm | Real corner |
|---|---|---|---|---|---|---|
| 0 | (0, 0) | start | 0.0 | — | 0.0 | — |
| 1 | (25, 0) | −117° | 1.8 | cut | 0.8 | cut |
| 2 | (5, −40) | +146° | 1.7 | cut | 0.6 | overshoot 0.6 cm |
| 3 | (40, −20) | −120° | 1.7 | cut | 0.2 | overshoot 0.8 cm |
| 4 | (40, −40) | +90° | 1.7 | cut | 0.2 | overshoot 0.5 cm |
| 5 | (65, −40) | +90° | 1.5 | cut | 0.3 | overshoot 2.1 cm |
| 6 | (65, −20) | +90° | 1.4 | cut | 0.4 | hit |
| 7 | (40, −20) | −90° | 1.7 | cut | 0.2 | hit |
| 8 | (40, 0) | −90° | 1.3 | cut | 0.2 | hit |
| 9 | (65, 0) | −90° | 1.3 | cut | 0.3 | hit |
| 10 | (65, −20) | end | 1.4 | — | 0.4 | — |

Waypoint (40, −20) is visited twice (#3 and #7). The notebook's `waypoint_miss` takes the closest approach over the whole run, so both rows show the better of the two visits.

### 2. Run times and other numbers

| | Simulator | Real robot |
|---|---|---|
| **Run time** | **27.1 s** | **22.0 s** |
| Samples / mean loop period | 2715 / 10 ms | 466 / 47 ms |
| Path length (planned: 265 cm) | 252 cm | 288 cm |
| Peak speed | ≈ 20 cm/s | ≈ 28 cm/s |
| Mean / max distance from the planned path | 0.6 / 1.5 cm | 1.2 / 4.7 cm |
| Last line of the log | (65.65, −18.19) | (64.56, −18.52) |

Mean / max sim-vs-real distance at the same progress along the path: 2.7 / 6.7 cm.

The speed plot shows the difference between the two runs. In the simulator the robot stops completely at every corner (speed 0) and turns in place. On the real robot the speed only drops to 1–6 cm/s and rises again, so the robot turns while still rolling.

### 4. Measured end point

| | x, cm | y, cm | distance to the last waypoint, cm |
|---|---|---|---|
| Last waypoint | 65.0 | −20.0 | 0 |
| Last line of `gotogol_data_real.txt` | 64.56 | −18.52 | 1.5 |
| Tape measurement | *…* | *…* | *…* |
| Tape − odometry (localization error) | *…* | *…* | *…* |

*Fill in the tape row: measure the axle centre after the run, with x along the start heading and y to the robot's left.*

## Discussion

In the simulator every corner was **cut** by 1.3–1.8 cm, just under the acceptance radius ε = 2 cm: the program switches to the next waypoint as soon as ρ < ε, so the robot turns before it reaches the corner. On the real robot the corners after the long strokes were **overshot** instead: the bottom of the 7 (+146°), the entry into the 8 (−120°) and the lower loop of the 8, worst at (65, −40) with 2.1 cm; the robot swung wide arcs there, and its path grew to 288 cm against 252 cm in the simulator. This comes from the turning gain `Kth` = 60 acting on a real robot that is faster (peak 28 vs 20 cm/s) and runs its control loop only every 47 ms instead of 10 ms. With motor inertia added, the robot is still rolling when it starts the turn and never fully stops to turn in place. Both curves are the robot's own odometry, which places it within 1.5 cm of the final waypoint, so the sim-vs-real gap visible in the plots (up to 6.7 cm) is a **control** problem: the same gains produce different dynamics on the real hardware. The tape measurement adds the **localization** part; for example, if the real wheel radius is the course's 3.75 cm rather than the 4.2 cm used here, the odometry over-reports distance by about 12 %.

## How to reproduce

1. **Simulator.** Open `go_to_goal_sim (1).qrs` in TRIK Studio (TRIK v6.2 kit, 2D model) and run it. The planned 78 is drawn on the field as red dashed lines.
2. **Real robot.** Open `go_to_goal_real (2).qrs`, connect the robot and use **Upload and run**. If one encoder counts backwards when the robot is pushed forward, set `encSignL` or `encSignR = -1` in the Init block. Copy `gotogol_data_real.txt` from the robot.
3. **Analysis.** Open `gotogoal_analisys.ipynb` in Colab, upload both txt files and run all cells.


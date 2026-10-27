# Lab 7: Go-to-goal (digits **78**)

The robot drives the last two digits of my student ID, **7** and **8**, as a list of waypoints. A go-to-goal controller runs on top of the Week 6 odometry. The same program runs in the TRIK Studio 2D simulator and on the real TRIK robot, and the two runs are compared in Colab.

## Contents

| File | What it is |
|---|---|
| `go_to_goal_sim.qrs` | TRIK Studio program for the 2D simulator. It logs to `odometry_data_sim.txt`. |
| `go_to_goal_real.qrs` | The **same** program and gains for the real robot. It logs to `odometry_data_real.txt`. |
| `odometry_data_sim.txt`, `odometry_data_real.txt` | Raw logs: `x,y,time` (cm, cm, ms), no header. *They are produced by running the programs.* |
| `Lab7_go_to_goal_analysis.ipynb` | Colab notebook: trajectory plot, waypoint-miss table, run times, cross-track error, real-vs-model gap and the end-point comparison. |
| `gotogol_analisys.ipynb` | The notebook's code as a plain script (`python analyze_go_to_goal.py` next to the two logs). |

## Waypoints (cm)

The robot starts at the top-left corner of the 7 at `(0, 0)`, facing +x. +y is to the robot's left, so the digits are upright both in the simulator and in the plots.

```
digit 7: (0,0) → (25,0) → (5,-40)
gap:     (5,-40) → (40,-20)
digit 8: (40,-20) → (40,-40) → (65,-40) → (65,-20) → (40,-20) → (40,0) → (65,0) → (65,-20)
```

The 8 is two stacked squares drawn as a single continuous path. It starts and ends on the middle bar, which is the only way to draw it without retracing a stroke. The path has 10 goals and 265 cm in total. Its corners are 90° turns plus three sharp ones: −117° at the top of the 7, +146° at its bottom, and −120° entering the 8.

## Program

The diagram follows the structure of the reference program `Fixed_1x1_trajectory.qrs`:

```
Start → Clear Encoder E3,E4 → Remove File (old log) → Init
  while i < N:                                   (waypoint loop)
     while rho > eps:                            (go-to-goal loop)
        odometry + control  (expression block)
        if |e_theta| > eTurn:  v = 0             (turn in place)
        pL = v - w,  pR = v + w, lifted over the motor dead zone
        M3 = pL,  M4 = pR
        Write To File: x,y,time
        Timer 5 ms
     i = i + 1
Stop Motors → End
```

**Odometry** (encoders in degrees, M3/E3 = left, M4/E4 = right):
`dL = Δenc3·π/180·r`, `dR = Δenc4·π/180·r`, `dS = (dL+dR)/2`, `dθ = (dR−dL)/B`,
`x += dS·cos(θ+dθ/2)`, `y += dS·sin(θ+dθ/2)`, `θ += dθ`.

**Controller** (the lecture's table):

| Quantity | Implementation |
|---|---|
| Distance | `rho = sqrt(dx² + dy²)` |
| Bearing | `thG = atan2(dy, dx)` |
| Heading error, wrapped | `eTh = atan2(sin(thG − θ), cos(thG − θ))` → always in (−π, π] |
| Linear | `v = max(vMin, min(Kv·rho, vMax))` |
| Angular | `w = max(−wMax, min(wMax, Kth·eTh))` |
| Wheels | `pL = v − w`, `pR = v + w` (the B/2 factor is folded into `Kth`) |
| Dead-zone lift | `M3 = sgn(pL)·(dz + |pL|·(100 − dz)/100)`, the same for `M4` with `pR`, `dz = 20 %` |
| Switching | next waypoint when `rho < eps` |

The wrap uses `atan2(sin, cos)` instead of an `if` block, so the error can never be outside (−π, π].

### Gains (identical in both programs)

| Parameter | Value | Why |
|---|---|---|
| `Kv` | 3 %/cm | Full speed further than 17 cm from the goal, then slows down linearly. |
| `Kth` | 60 %/rad | Heading time constant ≈ 0.5 s in the simulator. 80 gave almost no gain and has more risk of oscillation on the real robot. |
| `vMax` / `vMin` | 50 % / 15 % | Forward command limits. `vMin` keeps the robot moving on the final approach. |
| `wMax` | 40 % | Limits turning so a large heading error cannot saturate a wheel. |
| `eps` (ε) | 2 cm | Smaller ε means sharper corners. Each corner is still cut by about ε. |
| `eTurn` | 0.3 rad (17°) | Turn-then-drive: sharp corners are turned in place and are not swung wide. |

### Motor dead zone `dz = 20 %` (same in both files)

Lab 5 showed that a TRIK wheel does not turn at all below roughly 5–15 % PWM in the air, and more on the floor carrying the robot. Without compensation, the real robot stalled. It sat at a corner trying to turn in place with about 18 % power, or crept up to a waypoint and never reached ε. The "power" block therefore maps every wheel command `u` to `sgn(u)·(dz + |u|·(100 − dz)/100)`: zero stays zero, and any other command starts above the dead zone. This describes the motors, not the controller, so `Kv`, `Kth` and ε are unchanged. If your robot still creeps, raise `dz` in the Init block (try 25–30) in **both** programs.

### Robot geometry (the only difference between the two files)

| | wheel radius `rw` | track `base` | source |
|---|---|---|---|
| `go_to_goal_sim.qrs` | 2.8 cm | 15.4 cm | TRIK Studio 2D model: wheel 16 px = 5.6 cm, wheels 44 px apart |
| `go_to_goal_real.qrs` | 3.75 cm | 19.4 cm | the course's real-robot code (`localization+point_to_point.py`). **Measure your robot.** |

## How to run

**Simulator**
1. Open `go_to_goal_sim.qrs` in TRIK Studio. Select the TRIK v6.2 kit and the 2D model.
2. Press *Run*. The planned "78" is drawn on the field as red dashed lines. The robot starts on the green start marker, facing right.
3. The program removes any old log, then appends to `odometry_data_sim.txt`. The file is written next to the TRIK Studio executable; the relative path is the TRIK default.

**Real robot**
1. Open `go_to_goal_real.qrs`. Check `rw` and `base` in the *Init* block against your robot.
2. Check the encoder signs. Push the robot forward by hand while watching the sensors: `encoder3` and `encoder4` must both increase. If one decreases, set `encSignL` or `encSignR = -1` in *Init*. This is not a gain change.
3. Place the robot with its axle centre on a tape mark and point it along the +x direction you will measure.
4. Use **Upload and run** (the Python or JavaScript generator). The log `odometry_data_real.txt` is written in the program's working folder on the robot (usually under `/home/root/trik/`). Copy it to the PC with WinSCP/scp.
   *Note:* the log line ends with `"\n"`. The code generators escape this correctly. TRIK Studio's direct remote-interpretation mode passes the text into a JavaScript string unescaped, so prefer uploading the generated program.
5. When the robot stops, measure the axle centre with a tape (step 4 of the deliverables).

**Analysis**: open `Lab7_go_to_goal_analysis.ipynb` in Colab and *Run all*. Upload both logs, and fill `MEASURED_END` in the parameters cell with your tape measurement.

To change the waypoints or gains, edit `tools/make_qrs.py` and run `python tools/make_qrs.py`. Then copy the new values into the parameters cell of the notebook.

## Measured end point (fill in after the real run)

| | x, cm | y, cm | distance to last waypoint, cm |
|---|---|---|---|
| Last waypoint | 65.0 | −20.0 | 0 |
| Last line of `odometry_data_real.txt` | … | … | … (≤ ε by construction) |
| Tape measurement | … | … | … |
| Tape − odometry (localization error) | … | … | … |

## Discussion (3–6 sentences, fill in from your results)

> Corners **…** were cut by ≈ … cm in both runs. That equals the acceptance radius ε = 2 cm: the robot switches to the next waypoint as soon as ρ < ε and turns before reaching the corner. Turning in place at |e_θ| > 0.3 rad keeps the sharp corners (−117° and +146° on the 7, −120° into the 8) from becoming wide arcs; with `eTurn` = 0.5 rad the arcs after each corner were visibly larger. *[Name any corner that overshot on the real robot. Overshoot comes from motor inertia/lag at high `Kth`, which the simulator does not model.]* According to the real robot's own odometry, it reached every waypoint within … cm, so the **controller** works. The tape shows the true end point … cm away from the odometry end point, so the remaining real-vs-model gap is a **localization** problem (wheel radius/track error, slip, unequal motors), and better gains cannot fix it.

## Verification done before handing in

TRIK Studio itself could not be run in the build environment, so the programs were checked in three ways:

1. **Format.** `python tests/check_qrs_format.py Fixed_1x1_trajectory.qrs go_to_goal_sim.qrs go_to_goal_real.qrs` parses every XML record. It checks that every block type has the same properties as in the reference file written by TRIK Studio 2025.4. The Remove File and Stop Motors blocks follow TRIK's `trikMetamodel.xml`.
2. **Language and semantics, checked against the trik-studio source code.** Write To File evaluates its text as an expression and appends without adding a newline. `..` converts numbers to strings. `"\n"` is unescaped by the parser and re-escaped by the code generators. `time()` returns ms since the program started. Integers are coerced to floats.
3. **Behaviour.** `python tests/trik_emulator.py go_to_goal_sim.qrs --out <dir>` runs the diagram from the `.qrs` file block by block, with the 2D model's kinematics. In that emulation the sim program finished "78" in ≈ 27 s; every waypoint was missed by ≤ 2 cm (cut by ε) with a mean cross-track error of 0.6 cm. `--real-like` adds motor lag, a dead zone (`--deadzone`), motor mismatch and slip, and `--tick` sets a slower control loop. With dead-zone compensation, the real-like robot finished for dead zones up to 30 % and loop periods up to 100 ms. Without it, the robot stalled from a 20 % dead zone on. **These emulated logs are for testing only and are not the lab results.**

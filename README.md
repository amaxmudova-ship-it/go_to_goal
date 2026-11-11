# Lab 7 — Go-to-goal: driving the digits **78**

The robot uses its odometry `(x, y, θ)` to drive a list of waypoints that draw the last two digits of my student ID, **7** and **8**. The same go-to-goal program runs in the TRIK Studio 2D simulator and on the real TRIK robot, and the two odometry logs are compared with the planned path in Google Colab.

## Repository contents

| File | Content |
|---|---|
| `go_to_goal_sim (2).qrs` | TRIK Studio program for the 2D simulator. It writes `gotogol_data_sim.txt`. |
| `go_to_goal_real (3).qrs` | The same program and gains for the real TRIK robot. It writes `gotogol_data_real.txt`. |
| `gotogol_data_sim(2).txt` | Raw simulator log: `x,y,time` (cm, cm, ms), no header. 2715 samples. |
| `gotogol_data_real.txt` | Raw real-robot log, same format. 466 samples. |
| `gotogoal_analisys.ipynb` | Colab notebook: trajectory plot, waypoint-miss table, run times. |

## Waypoints

The robot starts at the top-left corner of the 7, facing +x; +y is to its left.

```
digit 7: (0,0) → (25,0) → (5,-40)                     cm
gap:     (5,-40) → (40,-20)
digit 8: (40,-20) → (40,-40) → (65,-40) → (65,-20) → (40,-20) → (40,0) → (65,0) → (65,-20)

x_target = {0.25, 0.05, 0.40, 0.40, 0.65, 0.65, 0.40, 0.40, 0.65, 0.65}      m
y_target = {0.0, -0.40, -0.20, -0.40, -0.40, -0.20, -0.20, 0.0, 0.0, -0.20}
```

The path has 10 segments and 265 cm in total. The 8 is two stacked squares drawn as one line. Its segments are 20–25 cm long, against 25–45 cm for the 7 and the gap.

## Program

Both `.qrs` files have the same block diagram:

```
Start → Clear Encoder E3,E4 → Remove File (old log) → Init
  while i < 10:                         one pass per waypoint
     while dist > threshold:            drive to waypoint i
        odometry                        x, y, theta, dist, err
        if |err| > pi: wrap err         turn the short way
        u = 150·err,  mov = 150·dist·cos(err)
        M3 = mov − u,  M4 = mov + u
        Write To File: theta_l, theta_r, time
        Timer 5 ms
     i = i + 1
End
```

### Parameters

The controller gains are identical in both programs:

| Parameter | Simulator | Real robot |
|---|---|---|
| Turning gain, `u = K_θ · err` | 50 | 50 |
| Forward gain, `mov = K_v · dist · cos(err)` | 125 | 125 |
| Acceptance radius `threshold` (ε) | 0.02 m | 0.02 m |
| Wheel radius `r` | 0.028 m (2D model) | 0.042 m |
| Track width `W` | 0.154 m (2D model) | 0.194 m |

## Analysis (Colab)

1. **Rebuild** each trajectory from the wheel angles, with the `r` and `B` of that robot: `path = (θr + θl)·r/2`, `course = (θr − θl)·r/B`, then step x and y along the course.
2. **Ideal segment** `i` runs from waypoint `i−1` to waypoint `i`, with course `course_ref[i]`.
3. **Assign** each sample to the current segment or the next one, whichever is nearer.
4. **`distance`** is how far the robot is from its segment (+ means left of it). **`courseAngle`** = `course_ref − course`, in degrees.
5. **Plots and summary:** trajectory plot, `distance(t)` and `courseAngle(t)`, waypoint miss, summary table.


## Discussion

In the simulator every waypoint was passed 1.7–2.0 cm away, so every corner was **cut** by about the acceptance radius ε = 2 cm. The robot switches to the next waypoint as soon as it is closer than ε and turns before reaching the corner. Each early switch leaves the robot up to about 1.3 cm beside the next segment, and the turning gain of 150 then pulls it back. On the long strokes of the 7 and the gap it is back on the line within a few seconds (`distance` ≤ 0.3 cm). On the short 20–25 cm segments of the 8 the next corner comes first, so `distance` (up to 0.9 cm) and `courseAngle` (2.3–3.5°) stay larger there. On the real robot the corners after the long strokes were overshot instead of cut: the robot kept rolling past them and swung wide, worst at (65, −40) with about 2 cm. The cause is the turning gain K_θ on a robot with motor inertia and a slower control loop (46 ms vs 10 ms in the simulator). distance grew most right after the sharp corners, up to 5.9 cm on segment 4. Over the whole run, RMS distance was 1.7 cm and mean |courseAngle| on straight parts was 11.6°, against 0.4 cm and 1.9° in the simulator. The robot’s own odometry places it 1.5 cm from the last waypoint, so the gap to the model is a control problem.
 

## How to reproduce

1. **Simulator.** Open `go_to_goal_sim (2).qrs` in TRIK Studio (TRIK v6.2 kit, 2D model) and run it. 
2. **Real robot.** Open `go_to_goal_real (3).qrs`, connect the robot and use **Upload and run**. Copy `gotogol_data_real.txt` from the robot.
3. **Analysis.** Open `gotogoal_analisys.ipynb` in Colab, upload both txt files and run all cells.


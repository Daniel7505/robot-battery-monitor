# ButlerBot (repo: robot-battery-monitor)

**A simulated delivery robot plus a live "car dashboard" for its battery and power use.**
Owner: [Daniel7505](https://github.com/Daniel7505) · Co-built with Grok Build · Standing goals: [`docs/NORTH_STAR.md`](docs/NORTH_STAR.md) · Review rules: [`docs/REVIEW_CHECKLIST.md`](docs/REVIEW_CHECKLIST.md) · Project story: [`JOURNEY.md`](JOURNEY.md)

> **About the name.** The repo is still called "robot-battery-monitor" because that is how it started. It has grown into a full robot simulation project. The name is leftover.

---

## Contents

1. [What is this, in plain English?](#1-what-is-this-in-plain-english)
2. [What you'll see](#2-what-youll-see)
3. [Where the project stands (honest status)](#3-where-the-project-stands-honest-status)
4. [How it fits together (diagrams)](#4-how-it-fits-together-diagrams)
5. [Pick your setup](#5-pick-your-setup)
   - [A. Just look around (no install)](#a-just-look-around-no-install)
   - [B. Run the lightweight simulator and tests (no Webots)](#b-run-the-lightweight-simulator-and-tests-no-webots)
   - [C. Full Windows setup: dashboard + Webots robot](#c-full-windows-setup-dashboard--webots-robot)
   - [D. Linux or macOS (untested)](#d-linux-or-macos-untested)
   - [E. Developer / contributor workflow](#e-developer--contributor-workflow)
   - [F. Overnight optimizer (Windows)](#f-overnight-optimizer-windows)
6. [Reading the dashboard](#6-reading-the-dashboard)
7. [Settings you might change](#7-settings-you-might-change)
8. [Common problems](#8-common-problems)
9. [Where things live (folder map)](#9-where-things-live-folder-map)
10. [Credits and Licenses](#10-credits-and-licenses)
11. [Project journey](#11-project-journey)

---

## 1. What is this, in plain English?

Imagine a small wheeled robot that carries things around a warehouse. Before anyone buys parts and builds it, this project lets you:

- **Drive it in a 3D simulator** ([Webots](https://cyberbotics.com/), free). The robot follows yellow painted lanes using two downward-facing cameras, turns at corners and intersections, slows for obstacles, and stops cleanly.
- **Watch its "fuel gauge" live** in a web page: battery %, how many watts each body part uses (wheels, arms, torso, computer, cooling), safety warnings, and how long the battery should last.
- **Save the history** in a database so you can look back at past runs.
- **Test ideas without the simulator**: a fast "offline" lane-following simulator and 880 automated tests.
- **Start designing real robots**: a small catalog of real, buyable parts and a first-draft "design agent" that suggests the cheapest parts list for a mission.

The long-term goal ([`docs/NORTH_STAR.md`](docs/NORTH_STAR.md)) is an agent that picks real parts, designs the cheapest robot that can finish a mission, tests it in the simulator, and then builds it for real. Today's robot and dashboard are the **test bench** for that goal. The north star may evolve as the project learns; `NORTH_STAR.md` is the current version.

You **do not need a real robot**. Everything runs on a normal PC.

---

## 2. What you'll see

| Where | What it looks like |
|-------|--------------------|
| **Browser** at `http://127.0.0.1:5000` | The page that shipped after PR #23. Cards for **battery**, **system draw**, **runtime left**, and **energy left**. A **This lap** panel (phase, speed, position, mission time, energy used, distance) with teleop buttons (Forward, Back, Left, Right, Stop, Lane keep) and a **Proof** panel (measured vs model, steer source, cross-track, width and center error). The badge reads **TWIN LIVE** while Webots is posting and **AWAITING TWIN** otherwise. The page polls `GET /api/live` every 500 ms. |
| **Webots window** | ButlerBot (wheeled base, torso, two arms) driving along a yellow lane: by default a double S-curve (about 17.4 m along the lane) that ends at a red finish line at GPS x=16.5 m, or corners, intersections, or a warehouse with boxes. Small camera overlays show what the robot sees. |
| **Dashboard while Webots runs** | The badge changes from **AWAITING TWIN** to **TWIN LIVE**. Battery, system draw, and the This lap numbers come from the simulated robot. |
| **Terminal (no Webots)** | `rowfit_sim.py` prints a small table: did the robot finish the course, how far off the lane it drifted (cm), and how long it took. |

---

## 3. Where the project stands (honest status)

**Works today**

- Live power dashboard with safety rules, forecasts and an "onboard agent" that throttles the robot if you push it too hard.
- Webots robot that lane-keeps on its own, handles 90° corners, T-junctions and crossroads, detects obstacles (stereo cameras + radar) in obstacle worlds, and runs multi-trip "shopping list" pick runs in the warehouse world.
- A reliable stop (ABS, no wheel spin after stopping). See [`docs/STABILITY.md`](docs/STABILITY.md).
- Docker one-command startup, Postgres history, automated tests on every push (GitHub Actions).

**Limitations (please read)**

- **Simulation only.** There is no real robot or battery hardware connected. "Real" in settings like `HARDWARE_MODE=real` means "the full, realistic software path," **not** a physical robot.
- **Lane accuracy is a simulator number** (about 3 cm in ideal Webots). Real cameras, lighting and floors will be worse.
- **"Legs" means wheels.** The power channels are named Legs / Arms / Torso / Compute / Cooling because a walking robot was planned. On the wheeled ButlerBot, "Legs" is the drive wheels. A biped profile exists but the wheeled one is the trusted baseline.
- **ROS 2 is mocked by default.** No ROS install is needed; real ROS 2 is optional and less tested.
- **Design agent is a first draft.** The parts catalog has about 10 parts, and the suggested designs are **not** yet test-driven in Webots automatically.
- **Developed and tested on Windows 10/11 only.** Linux/macOS scripts exist but are **untested**.

### Mapping the apartment

Stage 1 explores `webots/worlds/apartment.wbt` with no map and no true pose. `apartment_mapper` reads the depth camera (`oak_depth`), the color camera (`oak_rgb`), the gyro yaw rate (`oak_gyro`), roll and pitch from `oak_imu`, the two wheel encoders, a ring of short-range sonars, and a downward floor camera (`floor_cam`). It builds a 2D occupancy grid and drives to frontiers until none longer than 0.3 m is reachable. A second controller, `apartment_pose_logger`, is the only process allowed to read the simulator pose. The mapper does not open `eval/apartment_ground_truth.json` or the world file. `tests/test_apartment_twin.py` fails the build if it does.

Run artifacts (`map.npy`, `map_meta.json`, `pose_log.csv`, and the logger's `gt_pose_log.csv`) go under `logs/mapping_runs/<timestamp>/`. That directory is gitignored. `logs/mapping_runs/CURRENT` points at the latest folder.

**Windows (Webots).** From the repo root:

```powershell
powershell -ExecutionPolicy Bypass -File scripts\launch_webots_twin.ps1 -World apartment
```

Optional caps, set in the same terminal before launch: `RBM_MAP_MAX_S` (stop after this many seconds), `RBM_MAP_STRIDE` (depth decimation, default 4), `RBM_MAP_DIR` (run folder). `RBM_FUSION`, `RBM_FLOOR`, and `RBM_LOOP` default on. Set any of them to `0` to turn that part off. `RBM_FLOOR=0` is the run without the downward floor sensor. `RBM_GYRO_CAL_S` (default 3.5) holds the hubs still at the start so the gyro bias can settle. `RBM_STRAIGHT_S` drives both hubs at 0.22 m/s for that many seconds and then stops. It is the open-floor contact check, not an exploration run.

**Score a run.** The scorer is the only tool that reads the tape and the world. It prints occupancy IoU, free-space IoU, coverage %, and mean wall error in cm, and writes `map_vs_ground_truth.png` next to the map.

```powershell
python eval\score_apartment_map.py (Get-Content logs\mapping_runs\CURRENT).Trim()
```

**No Webots.** The same mapper can be driven by a NumPy ray caster (not a Webots render). This is what CI runs:

```powershell
python eval\kinematic_apartment_map.py --steps 1600 --noise
python eval\score_apartment_map.py (Get-Content logs\mapping_runs\CURRENT).Trim()
```

One kinematic run (1600 steps, depth noise on, 64×40 rays) had not finished the frontier search. Honest scores against the tape: occupancy IoU 0.168, free-space IoU 0.673, coverage 91.8%, wall error 17.9 cm. Occupancy IoU stays low because walls are thin and furniture is only partly seen. That number is not a Webots run.

The first Webots run (R2025a, fast, no rendering) scored occupancy IoU 0.232, free-space IoU 0.641, coverage 90.5%, wall error 38.9 cm, and printed `MAPPING done` at 355 s. The robot was physically stuck by the bedroom doorway for most of that time. `oak_gyro` reads +0.001 rad/s at rest, and integrating that bias walked the heading about 14° while the hubs were still. Wheel slip after that added false travel, and the warped grid looked closed. The mapper now learns that rest bias and does not integrate it, drops encoder travel when a wall in view does not move, backs up and blacklists the frontier when the body is not progressing, and will not declare done in the few seconds after a slip. A correlative match of the depth scan against occupied cells (Olson, *Real-Time Correlative Scan Matching*, ICRA 2009) corrects translation and yaw when the match beats the current pose by a clear margin. The step is capped, and a scan into a contact is not used. The inertial unit's yaw is not fused: in Webots it is a world heading.

The second Webots run, with that bias fix, scored occupancy IoU 0.180, free-space IoU 0.657, coverage 93.8%, wall error 28.4 cm, and printed `MAPPING done` at 229 s. Heading error was about 9° at the end. From 140–200 s the body was pressed into the wall beside the bath opening, inside the depth camera's 0.35 m minimum, so the range image was empty and the old slip check never fired. The encoders counted about 6.5 m. An empty forward view is now contact: that wheel travel is dropped, the drive does not keep going forward, and `done` stays off while the pose is blind or the latest scan fits the map better somewhere else. Doorway goals are the centre of the opening. A bumper was not added to the shared robot; a new collision shape would change warehouse contact. Wheel-motor torque at a speed the hubs are already reaching is rolling friction, not a contact flag, so it is not used as one.

The third Webots run kept heading to about +3°, then printed `MAPPING done` at 88 s after the living room and a corner of the kitchen. Scores: occupancy IoU 0.162, free-space IoU 0.625, coverage 86.7%, wall error 24.9 cm. It stalled on the bedroom doorway, backed up, banned that opening, and finished while it was still driving. Bans now expire, a contact does not black out the whole doorway, and `done` waits for a final scan plus a few seconds stopped with no reachable frontier. Planning inflation is 0.20 m, the tire radius, because 0.26 m plus a smeared jamb leaves no path through the 0.81 m opening. A stall no longer paints a permanent cell 0.28 m ahead of the nose: that point is the opening, while the tire is the part that meets the jamb, and a latched cell there is what closed the door. Contact only reinforces a wall the depth camera has already seen, and a free ray can wear that mark back down. The mapper prints contact, backup, blacklist, goal, and the done reason. Encoder travel is dropped while a forward command is skidding or the view is blind. The reverse that leaves that wall is kept, and it lasts long enough to clear the camera's minimum range. A door jamb is a shoulder to steer past, not a wall to stop for: stopping on the jamb is what turned the robot away from the bedroom door and the hallway. The goal is placed in the room beyond the opening, so the robot drives through instead of spinning in the doorway. The pose logger quits the world when the mapper writes `mapping_done`.

A headless R2025a run of that code (`--mode=fast --no-rendering`) did not finish on its own. The time cap stopped it at 540 s. The true pose went through the west hallway and into the bath, and reached the north end of the bath (about 1.35, 4.40), short of the bedroom. It did not enter the bedroom, the walk-in, or the laundry. Scores: occupancy IoU 0.156, free-space IoU 0.631, coverage 94.8%, wall error 30.6 cm. Coverage is above run 2. The other three are not. The odometry pose at the cap was several metres off the true pose, and the scored map is rotated. `done` was the cap, not a stopped hold with no frontier left.

The scan match is now a search over x, y, and yaw, not a yaw trim. A step is applied only when it beats the current pose, a second scan agrees, and the move stays inside 5 cm and 0.02 rad. A wider search was tried and walked the pose onto the wrong wall, so the window is 16 cm and 0.08 rad. Three headless seeds of that code (the stock heading, and the same world with the start heading shifted by +0.40 rad and −0.35 rad) all stopped on the 600 s cap. None entered the bedroom, the bath, the walk-in, or the laundry. Door G was not crossed. The stock-heading true pose did reach the west hallway. Final position error was 0.53 m, 2.30 m, and 2.50 m. Scores did not beat run 2: the best occupancy IoU was 0.165, and only the stock heading's wall error (24.9 cm) was under 28 cm. `done` was the cap in every seed.

The pose update fuses wheel travel, gyro yaw, depth-corner visual odometry, and a downward floor camera. The floor camera sits 4 cm above the ground. Its frame-to-frame shift is phase correlation (Kuglin and Hines, 1975), scaled by the nominal height, with pixel noise, a scale error when the floor height changes, and a reject when the image has no texture. The gain is a scalar Kalman weight on x, y, and yaw (Thrun, Burgard, and Fox, *Probabilistic Robotics*). If the floor image, or a forward view that should have moved by several pixels, stays still while the hubs turn, that wheel step is dropped as slip. A fast yaw between frames is left to the gyro: the patch search cannot follow a spin, and applying it opened a false frontier. Loop closure recognizes a place when several landmark distances agree, allows one object to have moved, checks the correction against the distance travelled, and spreads it back along the keyframes. The same match has to agree twice, and the next closure waits for another 2.5 m of travel. One pass was not enough: about thirty wrong votes on `b833bf4` wiped the occupancy grid (IoU 0.055). The correlative scan match (Olson, ICRA 2009) is only the done-gate disagree check. The stop distance is the warehouse shape (`stop_distance_m`: standoff, lag, and `v² / 2a`) measured from the shell, with a few centimetres of standoff so an 0.81 m door still fits, plus a ring of short-range sonars for the depth camera's blind hole inside 0.35 m. Those sensors are an opt-in `mappingSensors` field. Warehouse instances leave it off, and the expanded warehouse robot is unchanged.

Headless scores of that stack on `cabddd3`, Webots R2025a, `--mode=fast --no-rendering`, stock spawn heading, cap 600 s. Every run printed `MAPPING time cap`. None was the stopped-hold finish. Door G was not crossed (true x stayed above 3.77). The bedroom, bath, walk-in, and laundry were not entered. The other two start headings were not run: the stock heading already missed the door and the finish. RTAB-Map is not installed in this environment and the apt cache has no `rtabmap` package, so it was not scored as a yardstick. It is BSD-licensed (Mathieu Labbé and François Michaud) and is not part of this pipeline.

Position error is the odometry pose, turned by the spawn heading, against `gt_pose_log.csv` at the cap. The baseline is the stock-heading seed on `5574389`: occupancy IoU 0.162, free IoU 0.537, coverage 77.8%, wall error 24.9 cm, position error 0.53 m.

| | Baseline `5574389` | Bubble only | Fusion, no floor | Fusion + floor | Loop, unguarded | Loop, two looks |
|---|---:|---:|---:|---:|---:|---:|
| Occupancy IoU | 0.162 | 0.200 | 0.182 | 0.167 | 0.055 | 0.211 |
| Free-space IoU | 0.537 | 0.576 | 0.608 | 0.603 | 0.554 | 0.632 |
| Coverage | 77.8% | 85.8% | 90.5% | 94.8% | 73.5% | 85.2% |
| Wall error | 24.9 cm | 35.8 cm | 33.2 cm | 36.2 cm | 30.0 cm | 22.9 cm |
| Final position error | 0.53 m | 0.93 m | 0.23 m | 1.80 m | 0.57 m | 0.37 m |
| Median position error | — | 0.39 m | 0.38 m | 0.21 m | 0.41 m | 0.22 m |
| Floor/visual slip lines | — | 0 | 0 | 75 | 145 | 2616 |
| Stopped | time cap | time cap | time cap | time cap | time cap | time cap |

The floor sensor is the pair in the middle, same code, `RBM_FLOOR` on or off, loop off. It did not cut the final position error: 1.80 m with the floor camera, 0.23 m without it. The median along the run did drop, from 0.38 m to 0.21 m. With the floor camera the error sat near 0.11–0.19 m from 200 s to 500 s, then the last minute drove back across the main room on a heading that was already about 30° off and the final error went to 1.80 m. Without it the final error was 0.23 m and the wall error was still 33 cm. The two-look loop run did not actually close a loop. Its wall error (22.9 cm) and occupancy IoU (0.211) are the best of these seeds, and the final position error (0.37 m) is under the 0.53 m baseline. It still hit the time cap, the floor sensor rejected a wheel step 2616 times, and the true pose never left the main room. No seed met the whole bar: cross door G, finish on the done gate, and beat both the wall error and the position error.

The 30–40° heading walk is the `oak_gyro` lookup. It adds about +0.001 rad/s at every rate, and 600 s of that is 0.6 rad, about 34°. The rest learner only updates while the hubs are still. Once exploration started they were not, so the bias stayed at 0 and the raw rate was integrated the whole run. Wheel yaw diverges from the gyro in a turn, and subtracting it pulled a good bias off (a half-second "straight" window walked 0.00094 rad/s down to 0.00045, and the heading was about 13° off at 600 s). That window is gone. Wheel yaw is logged and left out of the bias. The floor update writes no yaw: `floor_applied_yaw` is 0 on every step. Depth-corner visual odometry and a match of a depth wall line against occupied cells the robot already painted each moved the heading by a few degrees at most. The line match uses the angle between the two lines. It does not assume the walls sit on a compass direction.

The mapper now holds the motors at 0 for 3.5 s before the first move. The bias settles at 0.00094 rad/s and stays there unless the hubs are still again. Three headless seeds of fusion + floor + two-look loop on `83ba2be` (Webots R2025a, `--mode=fast --no-rendering`, cap 600 s, stock heading and the same world with the start heading shifted by +0.40 rad and −0.35 rad) all ended under 5°. Heading error in degrees, fused yaw against the pose logger, spawn heading removed:

| t (s) | Stock | +0.40 rad | −0.35 rad |
|---:|---:|---:|---:|
| 60 | −0.51 | −2.48 | +4.03 |
| 120 | +0.56 | −2.08 | +3.50 |
| 180 | +0.69 | −0.44 | +5.57 |
| 240 | +2.09 | −1.35 | +3.36 |
| 300 | +2.94 | +0.25 | +3.75 |
| 360 | +4.23 | +0.97 | +5.90 |
| 420 | +4.82 | +1.92 | +7.75 |
| 480 | +3.64 | +3.06 | +7.66 |
| 540 | +1.10 | +2.40 | +3.39 |
| 600 | +0.74 | −2.68 | +1.42 |

The −0.35 seed peaked at +7.8° in the middle. That is the gyro's rate noise (about 5% of the reading) during spins, on top of a bias that was already removed. On the +0.40 seed the robot sat against a wall with the encoders frozen and the body still turning, and the rest learner ran again: the bias went from 0.00094 to 0.00195 at 540 s and back to 0.00101 at 600 s. The end error was still −2.68°.

The same three seeds, scored the same way as the table above. Every one printed `MAPPING time cap` and then `MAPPING done` at 600 s. That is the cap, not a stopped hold. Door G is the west wall at about x = 3.71. Crossing means the true pose went below x = 3.70.

| | Stock | +0.40 rad | −0.35 rad |
|---|---:|---:|---:|
| Occupancy IoU | 0.170 | 0.170 | 0.081 |
| Free-space IoU | 0.639 | 0.509 | 0.579 |
| Coverage | 93.8% | 96.6% | 79.8% |
| Wall error | 24.8 cm | 31.4 cm | 29.3 cm |
| Final position error | 0.41 m | 11.00 m | 0.75 m |
| Median position error | 0.27 m | 1.02 m | 0.42 m |
| Floor slip lines | 54 | 16 | 52 |
| End heading error | +0.74° | −2.68° | +1.42° |
| Door G | stayed east (min x 3.89) | crossed at 267 s | stayed east (min x 3.84) |
| Stopped | time cap | time cap | time cap |

The stock seed beats the `5574389` baseline on occupancy IoU (0.170 vs 0.162), free-space IoU, coverage, wall error (24.8 cm vs 24.9 cm), and final position error (0.41 m vs 0.53 m). It does not beat the two-look wall error (22.9 cm) or the no-floor position error (0.23 m). The +0.40 seed is the one that crossed door G: first true pose below x = 3.70 at t = 266.7 s (x = 3.698, y = 1.619), then 41663 steps on the west side, ending at (2.95, 2.11). Its odometry ended at (9.17, 0.11). The largest single odometry step was 2.4 cm, so the 11 m is accumulated while the body was pressed into that wall. The bedroom, bath, walk-in, and laundry were not entered on any seed. RTAB-Map is not installed here and the apt cache has no `rtabmap` package, so it was not scored. It is BSD-licensed (Mathieu Labbé and François Michaud) and is not part of this pipeline.

End heading is under 5° on all three seeds. Door G was crossed on the +0.40 seed. None of the three finished through the done gate before the cap.

**Wheel contact.** The 2616 rejected steps on the two-look run are the floor-flow gate (`floor slip, wheel motion rejected`), not tires floating off the floor. A straight drive in the main room (`RBM_STRAIGHT_S`, both hubs 0.22 m/s, Webots R2025a, fast, no main-view rendering) was logged by the pose logger: per-wheel contact points, and true speed from the pose, against encoder speed. The laminate top is at z = 0. The drive-wheel cylinders have radius 0.08 m and their centres sit at z = 0.08 m, so they meet that face. Contact properties for `drive_wheel` on laminate are mu 8, force-dependent slip 0.01, soft CFM 1e-6, soft ERP 0.7. The body mass is 3.2 kg with the centre of mass at z = 0.12 m. `basicTimeStep` is 8 ms. The world and the proto were not changed: the settled contact was already right, and the warehouse robot hash is unchanged. The open-floor slip count is the same before and after the heading work.

Once the start-up pitch had settled (t > 0.52 s, hubs commanded, true speed above 0.15 m/s), 0 of 736 steps had |encoder − true| / encoder above 15%. Mean encoder speed 0.220 m/s, mean true speed 0.220 m/s, largest gap 0.015 m/s. Both tires had two floor contacts on 893 of 899 steps. Wheel centres stayed at z = 0.080 m and the body at z ≈ 0. Wall-contact steps: 0. The other six steps are a 24 ms unload at the start of the push (t = 0.464–0.480) and a 24 ms unload when the motors stop (t = 6.52–6.536). The catch casters are spheres of radius 0.04 m centred at z = 0.044 m, so their bottoms sit 4 mm above the tire contact, and the body leans about 2° (up component 0.9993) until two of them touch. That pitch unloads the tires for those three steps each way. It is the caster clearance already in the proto. Steady driving on open floor does not slip.

During the three exploration seeds, encoder axle speed against body speed along the true heading, with no wheel or caster wall contact, a hub command above 0.4 rad/s, true forward speed above 0.08 m/s, and t > 5 s: stock 1028 of 40182 steps (2.6%, mean encoder − true +0.003 m/s, both tires on the floor 98.8% of those steps), +0.40 rad 1891 of 31332 (6.0%, mean +0.000 m/s, 98.3% in contact), −0.35 rad 959 of 39599 (2.4%, mean +0.004 m/s, 99.0% in contact). The higher +0.40 rate is the long press against the west wall, where contact flickers on and off. Wall-contact rows, separate from that slip count: stock 843, all with x in 4.69–4.80 m and y in 0.73–0.92 m (south side of the main room); +0.40 rad 16415, all with x in 2.91–3.20 m and y in 2.07–2.14 m (the west side of door G); −0.35 rad 957, mostly x 4.7–5.3 m and y near 0.9–1.0 m, plus a smaller cluster near y = 2.5 m. A 4 s straight probe of the floor camera, sampled the way the fusion samples it, measured a shift of −0.34 m against about 0.88 m of true travel (29 frames, texture std 7.4). The disagreement gate drops that update. The same gate is what fired 54, 16, and 52 times on these seeds, down from 2616.

The +0.40 wedge was a side press on the door frame. The forward depth still saw the room, a slow hub command never tripped the one-step stall check, and odometry translation kept the progress timer alive. The mapper now freezes wheel translation when the depth scene, the floor flow, and visual odometry are all still while the hubs are commanded, and it backs away from a side that stays inside 9 cm until that sonar gap opens.

Before each pursuit segment it also sweeps the 0.39 m shell plus a 3 cm margin along the next 0.55 m of the commanded arc, including the swing of a turn, and checks those cells against occupied cells on the robot's own grid. Unknown cells are not walls. Cells the shell already covers are ignored. If the sweep clips, the command becomes a wider arc, a straight pass at 0.12 m/s, a spin toward the next path segment, or a stop. Outside a doorway the robot spins onto the centre-line heading and then drives straight. Once both side sonars see the frame, a small centering yaw remains and the sweep can still reject it. Side-escape reverses are not swept. The 2 s note and the `MAPPING done` line print the counts.

Three headless seeds of that stack on `0a2348b` (same worlds, cap, and sensor flags as the table above). Heading error in degrees, fused yaw against the pose logger, spawn heading removed:

| t (s) | Stock | +0.40 rad | −0.35 rad |
|---:|---:|---:|---:|
| 60 | +1.38 | −2.11 | +5.38 |
| 120 | +1.20 | −0.67 | +3.65 |
| 180 | +0.74 | +1.25 | +1.93 |
| 240 | −0.59 | +1.24 | +1.36 |
| 300 | −0.06 | +0.63 | +1.55 |
| 360 | −0.12 | −1.58 | +1.69 |
| 420 | +0.14 | −1.07 | +0.94 |
| 480 | +0.35 | −0.98 | +1.72 |
| 540 | +1.07 | −1.56 | +3.03 |
| 600 | +2.84 | −1.25 | −1.15 |

| | Stock | +0.40 rad | −0.35 rad |
|---|---:|---:|---:|
| Occupancy IoU | 0.198 | 0.163 | 0.239 |
| Free-space IoU | 0.600 | 0.625 | 0.556 |
| Coverage | 85.0% | 88.0% | 95.7% |
| Wall error | 23.7 cm | 18.1 cm | 39.0 cm |
| Final position error | 0.31 m | 0.30 m | 2.17 m |
| Median position error | 0.31 m | 0.25 m | 0.33 m |
| Floor slip lines | 17 | 1 | 270 |
| End heading error | +2.84° | −1.25° | −1.15° |
| Door G | stayed east (min x 3.90) | stayed east (min x 3.80) | crossed at 184 s, back by 191 s |
| Stopped | time cap | time cap | time cap |
| Sweep checks | 65369 | 65285 | 51839 |
| Sweep rejected | 14059 (21.5%) | 15659 (24.0%) | 15682 (30.3%) |
| Wider arc | 2051 | 1415 | 3807 |
| Lined up in place | 8577 | 8369 | 1772 |
| Straightened | 4049 | 4119 | 2128 |
| Stopped by the sweep | 6891 | 9717 | 8666 |

Door G wall-contact rows are 0, 0, and 0. On `83ba2be` those counts were 843, 16415, and 957, and the +0.40 odometry ended 11 m away while the body sat on the west jamb. On this run the +0.40 final position error is 0.30 m. The −0.35 seed is the one that crossed the opening: first true x below 3.70 at t = 184.1 s (x = 3.700, y = 1.962), 848 steps west of the wall, deepest x = 3.47, back on the east side by t = 190.9 s. It entered the hall and did not reach the bedroom, the bath, the walk-in, or the laundry. Neither did the other two seeds. Every seed printed `MAPPING time cap` at 600 s. That is the cap, not the stopped-hold finish.

The −0.35 seed still collected 11483 wall-contact rows, all at x 3.90–3.97 m and y 4.44–4.76 m, the east face of the north end of the shared wall. That is not the door opening (the opening is y 1.27–2.29). Its odometry ended at (−4.76, 2.83) against a true pose of (3.90, 1.78), which is the 2.17 m final error and the 39 cm wall error. End heading is under 5° on all three seeds. The lineup and straight counts include doorway approaches whose sweep was already clear, so those two plus the wider-arc and stop counts add up to more than the rejected count. A rejected move is one the sweep changed because the shell would have entered an occupied cell.

That cluster was the tail, not a frozen encoder and not door G. Heading stayed near 0, so the wall was behind the shell. The rear sonar origin sits inside the face and reads the 0.02 m lookup floor, the bubble called the gap blocked, and the pivot backup reversed, which drove the tail in further. About half the commands in that stretch were both wheels in reverse. True forward speed was about zero while the encoders kept integrating. Visual odometry returned no sample when the forward view had too few corners, so the three-sensor pin never closed. A sweep stop shorter than 0.25 s also wiped the still window. Depth of the room ahead does not change when the tail slides on a wall behind the camera, and the footprint sweep only tests cells the robot has already marked occupied.

The same seed turned around about 7 s after it entered the hall because a replan replaced the goal whenever the new frontier was more than 1.2 m away. A clear straight sweep in a doorway approach or mouth now commits that heading for 16 s, and frontiers behind it are scored at a quarter. An unchanged forward image is a still vote for the pin and does not write yaw. A commanded spin does not integrate encoder translation. The forward radar no longer counts as a shell block when the bubble is open: a return inside 0.45 m was spinning the robot in place and reversing every time the depth shoulders swapped. The footprint margin is 2 cm and the sweep horizon is 0.35 m. Three centimetres on a 5 cm cell, and a 0.55 m lookahead, were stopping straight passes the next step would recheck.

Three headless seeds of that stack on `146611c`. Heading error in degrees:

| t (s) | Stock | +0.40 rad | −0.35 rad |
|---:|---:|---:|---:|
| 60 | +1.97 | −1.84 | +6.58 |
| 120 | +1.13 | −2.64 | +6.91 |
| 180 | −0.13 | −1.75 | +6.56 |
| 240 | +0.78 | −2.49 | +7.48 |
| 300 | +0.07 | −3.45 | +9.05 |
| 360 | +1.71 | −3.15 | +7.99 |
| 420 | +3.32 | −4.13 | +8.15 |
| 480 | +4.07 | −5.84 | +7.70 |
| 540 | +4.27 | −5.04 | +5.34 |
| 600 | +6.44 | −5.96 | +5.02 |

| | Stock | +0.40 rad | −0.35 rad |
|---|---:|---:|---:|
| Occupancy IoU | 0.159 | 0.069 | 0.122 |
| Free-space IoU | 0.595 | 0.629 | 0.621 |
| Coverage | 89.4% | 84.4% | 89.3% |
| Wall error | 35.5 cm | 29.0 cm | 26.1 cm |
| Final position error | 1.39 m | 1.24 m | 0.51 m |
| Median position error | 1.07 m | 0.35 m | 0.52 m |
| Floor slip lines | 285 | 96 | 32 |
| End heading error | +6.44° | −5.96° | +5.02° |
| Door G | stayed east (min x 3.88) | crossed at 488 s, hall only | crossed at 167 s, back east |
| Stopped | time cap | time cap | time cap |
| Sweep checks | 60290 | 67590 | 68079 |
| Sweep rejected | 14556 (24.1%) | 8164 (12.1%) | 9396 (13.8%) |
| Wider arc | 6929 | 2615 | 2088 |
| Lined up in place | 2603 | 11086 | 15330 |
| Straightened | 4519 | 1000 | 2627 |
| Stopped by the sweep | 1804 (3.0%) | 3855 (5.7%) | 4690 (6.9%) |
| Wall-contact rows | 540 | 0 | 0 |
| Longest wall press | 3.69 s | 0 | 0 |

Door G wall-contact rows are again 0, 0, and 0. The stock contacts are 540 rows at x 3.90–3.96 m and y 5.48–5.54 m, the east face north of the shared wall, in three episodes, two of them longer than 2 s (longest 3.69 s at t = 123 s). The +0.40 seed crossed at t = 487.9 s (x = 3.700, y = 1.941) and spent 4518 steps west of the wall, in the hall. The −0.35 seed crossed at t = 167.3 s (x = 3.700, y = 1.793), spent 2709 steps west, and was back on the east side by the end (true pose 4.22, 1.17). No seed entered the bedroom, the bath, the walk-in, or the laundry. Every seed printed `MAPPING time cap`. Sweep stops fell from 10.6%, 14.9%, and 16.7% of checks on `0a2348b` to 3.0%, 5.7%, and 6.9%.

The success bar was not met. No seed reached the bedroom. The stock seed has wall-press episodes longer than 2 s. Final position error is 1.39 m, 1.24 m, and 0.51 m, all above 0.3 m. End heading is above 5° on the stock and +0.40 seeds. An intermediate revision (`5f5feb4`) had no wall-press episode longer than 2 s (13, 0, and 0 contact rows; longest 0.08 s) and final position error 0.68 m, 0.87 m, and 0.54 m, and the −0.35 seed reached the hall at t = 550 s. It also missed the bedroom and the 0.3 m position bar. The radar change on `146611c` is what brought the 3.69 s press back.

`python3 eval/run_mapping_scenarios.py` runs five headless checks (door crossing, tail escape, closed loop, head-on stop, backup). The robot is not given the spawn. On `c407498` the loop passed (heading +0.46°, position 1.2 cm) and the tail escaped with a 0.03 s press. The bubble quit while the body was still at 0.24 m/s, and the door run never entered the hall. The first tail judge failed at a 0.42 m walk by summing a pin-snap sawtooth; the net chord while the body stayed inside a 3 cm box was 2.3 cm. On `38444db` all five pass. The door run from a lined-up spawn reached x = 1.22 and finished at (1.51, 2.36) on the bedroom side, contacts 0. The closed loop finished at +0.15° and 1.1 cm, with visual yaw held at 0. The head-on stop and the backup both ended at 0 m/s short of the wall.

Heading on the open square was already under half a degree once still-image samples stopped writing yaw. The remaining long-run pull was a visual yaw gain of about 0.4, about +0.005 rad on that square. `38444db` leaves heading on the gyro. A rear depth camera covers reverse, the tail pin, and visual tracking when the forward view has no corners. Frontiers past a doorway the robot segmented itself score higher, and a crossing in front of the robot outranks a larger crossing behind it. The map is labelled `room_1`, `room_2`, … with no prior room count. A passage is named `hall` only when the robot's own grid shows one. The scorer is the only step that compares those labels with the four rooms and the hallway.

Three headless seeds of that stack on `38444db`. Heading error in degrees:

| t (s) | Stock | +0.40 rad | −0.35 rad |
|---:|---:|---:|---:|
| 60 | −1.40 | −4.97 | +3.37 |
| 120 | −1.63 | −4.82 | +4.06 |
| 180 | −0.23 | −3.88 | +3.81 |
| 240 | +0.52 | −4.13 | +3.16 |
| 300 | +0.45 | −4.31 | +3.85 |
| 360 | −0.12 | −4.17 | +5.86 |
| 420 | −0.13 | −4.89 | +4.58 |
| 480 | −1.71 | −4.32 | +5.18 |
| 540 | −0.30 | −4.10 | +4.12 |
| 600 | −0.53 | −6.00 | +4.30 |

| | Stock | +0.40 rad | −0.35 rad |
|---|---:|---:|---:|
| Occupancy IoU | 0.170 | 0.130 | 0.134 |
| Free-space IoU | 0.658 | 0.605 | 0.652 |
| Coverage | 94.4% | 88.1% | 94.6% |
| Wall error | 22.4 cm | 20.4 cm | 24.6 cm |
| Final position error | 0.43 m | 0.34 m | 0.60 m |
| Median position error | 0.34 m | 0.25 m | 0.38 m |
| Floor slip lines | 101 | 1 | 4 |
| End heading error | −0.53° | −6.00° | +4.30° |
| Visual yaw applied | 0 | 0 | 0 |
| Door G | hall at 568 s, ended in the hall (min x 2.88) | hall at 138 s, ended in the living room (min x 2.63) | hall at 37 s, bath at 122 s, ended in the bath (min x 0.57) |
| Bedroom | not entered | not entered | not entered |
| Stopped | time cap | time cap | time cap |
| Sweep checks | 66595 | 68159 | 53634 |
| Sweep rejected | 9983 (15.0%) | 3724 (5.5%) | 16643 (31.0%) |
| Wider arc | 3331 | 1232 | 7481 |
| Lined up in place | 4133 | 14623 | 2449 |
| Straightened | 2360 | 1661 | 854 |
| Stopped by the sweep | 3302 (5.0%) | 776 (1.1%) | 7550 (14.1%) |
| Wall-contact rows | 27 | 0 | 136 |
| Longest wall press | 0.39 s | 0 | 0.26 s |

The stock 27 contact rows are one 0.39 s episode at x 4.72–4.77 m, y 1.43–1.48 m. The −0.35 seed's 136 rows are 41 episodes, the longest 0.26 s, at x 1.63–1.64 m, y 2.12 m, inside the bath. No episode is longer than 2 s. The −0.35 seed is the first of these runs to enter the bath. No seed's body entered the bedroom. The success bar was not met: position error is above 0.3 m on every seed, and the +0.40 end heading is 6.0°.

Rooms the scorer matched at overlap 0.15 or better. The label is the robot's own name for the region that covered that floor. Living, kitchen, and the bedroom floor share `room_1` on every seed, and the body never stood in the bedroom, so that overlap is one segment warped across those floors.

| Floor | Stock | +0.40 rad | −0.35 rad |
|---|---|---|---|
| Living | room_1 (0.95) | room_1 (0.94) | room_1 (0.97) |
| Kitchen | room_1 (0.78) | room_1 (0.70) | room_1 (0.86) |
| Bedroom | room_1 (0.92) | room_1 (0.89) | room_1 (0.81) |
| Bath | room_5 (0.37) | room_3 (0.36) | room_2 (0.42) |
| Hall | room_2 (0.57) | room_3 (0.49) | room_1 (0.16) |

Distinct labels: stock `room_1`–`room_5`, +0.40 `room_1`–`room_3`, −0.35 `room_1` and `room_2`. None of the three maps used the name `hall`. The stock hall is its own region (`room_2`). On +0.40 the hall shares `room_3` with the bath. On −0.35 the hall's overlap is 0.16 with the main-room label.

Eight headless seeds on `69dcb93`, 600 s, fusion + floor + two-look loop. Spawns are in the living room with clearance of at least 0.69 m. The three historical headings are included. The carpet step the casters climb is a ramp; the bedroom floor height is unchanged. Saved room labels are only cells the shell covered.

Heading error in degrees. The +0.40 column is no longer −5° at t = 60 s. Wall yaw applied on that seed is 0 through t = 180 s.

| t (s) | Stock | +0.40 | −0.35 | East | West | SW | North | South |
|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| 60 | −1.41 | −0.24 | +1.59 | +0.63 | +0.37 | −1.16 | −0.14 | +0.24 |
| 120 | −1.14 | −0.40 | +1.41 | +1.58 | +1.91 | −0.32 | −0.46 | −0.00 |
| 180 | −0.58 | −4.24 | +2.94 | +0.71 | +3.87 | −0.28 | −2.42 | −0.00 |
| 240 | +1.26 | −3.85 | −18.54 | +0.47 | +5.25 | −0.54 | −4.08 | −1.07 |
| 300 | +1.71 | −3.21 | −33.37 | +0.98 | +5.23 | +1.18 | −5.72 | −0.05 |
| 360 | +2.83 | −3.64 | −42.82 | +1.71 | +4.73 | −1.47 | −5.10 | −1.08 |
| 420 | +5.06 | −1.23 | −42.87 | +1.58 | +5.80 | −3.61 | −2.38 | −2.26 |
| 480 | +6.67 | −2.38 | −33.72 | +1.03 | +4.95 | −3.29 | −3.10 | −2.98 |
| 540 | +7.70 | −1.45 | −30.21 | −0.33 | +5.82 | −2.87 | −4.43 | −2.27 |
| 600 | +10.15 | −0.39 | −25.49 | −0.26 | +3.83 | −3.17 | −5.10 | −3.43 |

| | Stock | +0.40 | −0.35 | East | West | SW | North | South |
|---|---:|---:|---:|---:|---:|---:|---:|---:|
| Spawn | (6.10, 4.70) +90° | (6.10, 4.70) +113° | (6.10, 4.70) +70° | (5.40, 3.80) 0° | (6.60, 3.90) 180° | (4.90, 4.50) +40° | (5.80, 5.60) +126° | (6.40, 6.30) −69° |
| Occupancy IoU | 0.155 | 0.146 | 0.104 | 0.264 | 0.196 | 0.177 | 0.163 | 0.159 |
| Free-space IoU | 0.652 | 0.625 | 0.575 | 0.686 | 0.625 | 0.610 | 0.629 | 0.628 |
| Coverage | 97.5% | 96.2% | 87.7% | 92.4% | 89.4% | 89.4% | 92.6% | 93.3% |
| Wall error | 20.9 cm | 33.6 cm | 39.0 cm | 15.4 cm | 30.2 cm | 27.8 cm | 30.4 cm | 26.9 cm |
| Final position error | 0.44 m | 1.08 m | 2.99 m | 0.21 m | 1.08 m | 0.84 m | 0.83 m | 0.70 m |
| Bedroom entry | no | 63 s | 66 s | 27 s | no | 35 s | 143 s | 126 s |
| Bath entry | 261 s | 261 s | 151 s | 480 s | no | no | no | 247 s |
| Hall entry | 148 s | no | no | 521 s | 567 s | 369 s | 597 s | 340 s |
| Longest wall press | 0.76 s | 0 | 2.17 s | 0.21 s | 0 | 2.27 s | 1.63 s | 1.71 s |
| Wall-contact rows | 131 | 0 | 1442 | 232 | 0 | 1575 | 427 | 338 |
| Floor slip lines | 30 | 5 | 69 | 2 | 7 | 202 | 1 | 7 |
| Stopped | time cap | time cap | time cap | time cap | time cap | time cap | time cap | time cap |

Six of the eight bodies entered the bedroom. The east seed is the one under the 0.3 m position bar (0.21 m) and under 2° of heading at the end. The +0.40 end heading is −0.39°. Its early offset is gone: −0.24° at t = 60 s, and the wall-yaw sum is still 0. The later −4.24° at t = 180 s is gyro walk while the body sat on the bedroom threshold, not a startup nudge. The −0.35 seed's heading leaves the band after t = 200 s (wall yaw only −1.4° by t = 240 s; the rest is gyro during a press). Position error above 0.3 m is concentrated in a few stretches, written up on the pull request. Room labels below count only space the shell covered. A doorway split shows up when the bedroom label differs from the living room.

| Floor | Stock | +0.40 | −0.35 | East | West | SW | North | South |
|---|---|---|---|---|---|---|---|---|
| Living | room_1 (0.09) | room_1 (0.16) | room_6 (0.16) | room_1 (0.04) | room_1 (0.07) | room_1 (0.30) | room_2 (0.20) | room_3 (0.09) |
| Kitchen | room_1 (0.03) | — | — | room_1 (0.05) | room_1 (0.17) | room_1 (0.02) | room_2 (0.01) | room_3 (0.08) |
| Bedroom | room_4 (0.00) | room_1 (0.37) | room_8 (0.19) | room_7 (0.26) | room_1 (0.03) | room_1 (0.19) | room_4 (0.20) | room_3 (0.25) |
| Bath | room_4 (0.09) | room_1 (0.04) | — | room_6 (0.08) | room_4 (0.02) | — | — | room_1 (0.20) |
| Hall | room_1 (0.63) | — | — | room_5 (0.41) | room_1 (0.47) | room_1 (0.69) | room_2 (0.46) | room_3 (0.35) |

Overlap under 0.15 is listed anyway so a seen-through cell is not mistaken for a visit. The bedroom is its own region on the −0.35 seed (`room_8`), the east seed (`room_7`), and the north seed (`room_4`). On +0.40, SW, and south it still shares the living-room label. The kitchen aisle is as wide as the kitchen, so a clearance neck does not split it from the living room on these maps. The west seed is the one that used the name `hall`.

Eight headless seeds on `828e01d`, same living-room spawns and the same 600 s cap. Door and bedroom scenarios passed on this commit before the seeds (bedroom body inside at 23 s, longest press 0). When depth, the downward floor image, and vision agree the shell is still, translation and gyro-integrated yaw freeze, including while the wheels spin. A commanded press that stays in that agreement backs off after 0.70 s. The carpet ramp stays in `f2ccd13` only.

Heading error in degrees, spawn yaw removed.

| t (s) | Stock | +0.40 | −0.35 | East | West | SW | North | South |
|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| 60 | −1.45 | −0.61 | +1.27 | −0.40 | −0.62 | −0.69 | +0.04 | +13.02 |
| 120 | −0.47 | −5.55 | +1.04 | −1.70 | +3.07 | +0.88 | −0.29 | −23.44 |
| 180 | −1.56 | −5.55 | +1.73 | +0.07 | +8.15 | +2.68 | −2.00 | −21.46 |
| 240 | −2.56 | −5.66 | +4.82 | +0.92 | +7.66 | +4.27 | −2.86 | −22.57 |
| 300 | −6.14 | +1.93 | +0.91 | +1.06 | +8.01 | +5.89 | −3.98 | −22.87 |
| 360 | −5.65 | +2.39 | −6.80 | +0.97 | +7.87 | +6.58 | −5.01 | −22.47 |
| 420 | −6.39 | +2.45 | −1.75 | +0.56 | +7.82 | +6.86 | −4.19 | −21.06 |
| 480 | −6.29 | +4.18 | −0.35 | +0.34 | +8.35 | +7.23 | +5.11 | −21.08 |
| 540 | −6.29 | +2.29 | −0.92 | +1.43 | +9.38 | +5.54 | +3.52 | −20.60 |
| 600 | −0.21 | +2.53 | −2.35 | +2.63 | +5.08 | +12.12 | +4.44 | −18.78 |

| | Stock | +0.40 | −0.35 | East | West | SW | North | South |
|---|---:|---:|---:|---:|---:|---:|---:|---:|
| Occupancy IoU | 0.085 | 0.184 | 0.170 | 0.167 | 0.103 | 0.176 | 0.178 | 0.135 |
| Free-space IoU | 0.580 | 0.645 | 0.638 | 0.629 | 0.628 | 0.618 | 0.650 | 0.657 |
| Coverage | 80.0% | 92.2% | 94.0% | 91.4% | 90.9% | 91.9% | 97.2% | 96.7% |
| Wall error | 30.3 cm | 18.9 cm | 20.5 cm | 24.3 cm | 31.9 cm | 26.4 cm | 21.1 cm | 33.1 cm |
| Final position error | 0.77 m | 0.42 m | 0.44 m | 0.53 m | 0.78 m | 2.06 m | 0.91 m | 1.00 m |
| Max position error | 0.77 m at 600 s | 0.46 m at 360 s | 0.44 m at 600 s | 0.53 m at 600 s | 0.79 m at 360 s | 2.30 m at 360 s | 0.91 m at 600 s | 1.35 m at 180 s |
| Bedroom entry | 172 s | no | 64 s | 27 s, still inside | 37 s | 67 s | 196 s | 171 s, still inside |
| Bath entry | 294 s | 278 s | 534 s | 97 s | 80 s | 145 s, ended there | 224 s | no |
| Hall entry | 275 s | 94 s, ended there | 563 s | no | no | no | 99 s | no |
| Longest wall press | 2.74 s | 0 | 1.71 s | 0 | 0 | 3.20 s | 0 | 0 |
| Wall-contact rows | 343 | 0 | 132 | 0 | 0 | 1705 | 0 | 0 |
| Floor slip lines | 20 | 1 | 11 | 1 | 4 | 119 | 5 | 0 |
| Zero-velocity updates | 7 | 7 | 23 | 0 | 6 | 13 | 13 | 25 |
| Stopped | time cap | time cap | time cap | time cap | time cap | time cap | time cap | time cap |

The −0.35 heading runaway is gone. On `69dcb93` that seed finished at −25.49° after the rest bias climbed to about 0.009 rad/s during a press. Here the bias dipped to 0.00012 at t = 180 s, returned to 0.00100, wall yaw stayed 0, and the end heading is −2.35°. The old +1.6 m odom walk at the bath walkway is gone with it: the largest mark-to-mark gain is +0.27 m in the first minute, and the end position error is 0.44 m (it was 2.99 m). The longest press is 1.71 s at t = 410–412 s, x 3.92–3.95 m, y 4.80–4.83 m, on the east face beside letter G.

The stock end heading is −0.21° (it was +10.15°). Bias stayed between 0.00090 and 0.00107 rad/s and wall yaw is 0. The trace still reaches −6.39° at t = 420 s, then comes back. The north end heading is +4.44° (it was −5.10°). Wall-line nudges on that seed sum to +0.048 rad, about +2.8°, applied while the shell was actually turning, so that remainder is the wall match and not a stationary gyro walk. The west end is +5.08°, with −0.072 rad of wall yaw by t = 600 s.

The south heading did not come out of the same fix. It finishes at −18.78° (it was −3.43°). Bias stays near 0.00105 and wall yaw is 0. Between about t = 50 s and t = 140 s the true yaw swings through more than half a turn while the fused yaw barely moves, and 25 zero-velocity updates fire in that window, plus three still backups. A pure yaw can leave the floor image's phase peak near zero, so the stationary check froze a real turn.

Presses over 2 s are still present. The stock seed's longest is 2.74 s at t = 127–130 s, x 8.11–8.15 m, y 3.88–3.92 m, on the east living-room wall (the previous run's longest there was 0.76 s). The SW seed's longest is 3.20 s at t = 302–305 s, x 1.43–1.54 m, y 4.35–4.50 m, on the bath side of the walkway, and a second episode is 3.08 s a few seconds earlier at the same gap (the previous longest was 2.27 s). Across t = 240–300 s the true pose only creeps from about (1.32, 4.49) to (1.56, 4.36) while the odometry walks about 1.3 m and the body speed stays in the 4–12 cm/s band, so floor flow keeps reporting motion and the zero-velocity update does not freeze that stretch. That seed logs 119 floor-slip lines and finishes at 2.06 m. The wheels-lifted detector did not fire on any seed; the carpet ramp's pitch stays under its threshold.

No seed finished under 0.3 m. Every seed stopped on the 600 s cap. Seven of the eight bodies entered the bedroom. The +0.40 seed did not; it spends the second half of the run in the hall. The east and south seeds are still in the bedroom at the cap. The −0.35 seed ends in the kitchen. The kitchen aisle is still the same region as the living room wherever both were observed.

| Floor | Stock | +0.40 | −0.35 | East | West | SW | North | South |
|---|---|---|---|---|---|---|---|---|
| Living | room_1 (0.29) | room_1 (0.10) | room_1 (0.12) | room_2 (0.04) | room_1 (0.21) | room_1 (0.08) | room_2 (0.19) | room_1 (0.13) |
| Kitchen | room_1 (0.00) | room_1 (0.01) | room_1 (0.02) | — | — | — | room_2 (0.02) | — |
| Bedroom | room_1 (0.33) | — | room_6 (0.34) | room_5 (0.48) | room_2 (0.33) | room_4 (0.16) | room_3 (0.17) | room_1 (0.19) |
| Bath | room_1 (0.14) | room_4 (0.08) | room_8 (0.05) | room_3 (0.06) | room_4 (0.03) | room_4 (0.12) | room_2 (0.30) | — |
| Hall | room_2 (0.28) | room_4 (0.64) | room_5 (0.37) | — | — | — | room_2 (0.89) | — |

Overlap under 0.15 is listed anyway. The bedroom is its own region on −0.35 (`room_6`), east (`room_5`), west (`room_2`), SW (`room_4`, just over 0.15, and the bath shares that label), and north (`room_3`). On stock and south it shares the living-room label. The +0.40 and −0.35 label sets include the name `hall`. The east label set includes that name, and the hall floor itself has no overlap.

The downward camera is the translation-stuck signal on `d87fb1b`. It missed "wheels spinning, floor not moving" at the carpet lip for three reasons, and none of them is focus or range. The camera is a pinhole with no focus field and a near plane of 1 cm. The body rises onto the carpet, so the pile stays about 4 cm away. Laminate in this 32×32 view is about std 5–8. The pile, 4 cm up, lands around 3, and the shift used to reject std under 4, so every carpet frame was low-texture and never became a reading. The cutoff is now 2. The vertical face of the lip is a flat frame, std about 0.5, for a fraction of a second while the pitch is about 4.5°, and that frame stays rejected: a blank picture looks the same whether or not the shell is moving. A yaw abort used to drop the window before it was judged (about 1.1 m of encoder travel on the SW walkway scrub). A long correlation has a separate failure: 32 px is one period, about 3.9 cm of floor, and a shift of one period comes back as a high-quality zero. That false still reversed a real 0.12 m/s doorway crossing. The stall is now a run of short steps, each about 1.2 cm, and only a sharp match counts. The camera sits on the spin axis, so a real pivot also leaves the picture still. Rewinding the gyro to that anchor added 0.50 rad in one sample and left the door heading at +44.5°. A pure pivot keeps the gyro. Forward travel with the same still picture backs off immediately. On the walkway the caster was in contact, true speed was about 1 cm/s, and the drive hubs were off the laminate (left z = 0.093 m, right 0.087 m, seated contact 0.080 m) while the body lean was only about 2°, under the 0.12 rad wheels-lifted threshold.

Front casters are unchanged. Sphere radius 0.04 m, joint anchor and endpoint at z = 0.044, so the bottom of the ball is 4 mm above the drive-wheel contact plane. The carpet step is 12.7 mm. A later round will sweep caster and wheel size against lip heights.

On `d87fb1b` the door scenario crosses and stays (heading −0.72°, position error 0.26 m, final (1.73, 4.22)). It also logs a 2.33 s press at the walkway, x 1.48–1.50 m, y 4.37–4.39 m. The bedroom scenario does not enter: longest press 0.66 s, final (3.92, 4.84), on the living-room side of letter G, with the sweep refusing the opening. The eight-seed tables above are still `828e01d`. Those caster seeds were not re-run.

`ButlerBotSkid` is a separate proto. `ButlerBot.proto` is unchanged, including the 0.04 m front caster and its 4 mm clearance. The skid robot has four driven tires and no casters. Track stays 0.34 m and radius stays 0.08 m. The front axle is at x = ±0.11, and the rear hubs keep the names the mapper already reads. The front motors are slaved only when those devices exist. Each tire's bounding cylinder is rotated 90° about X together with the visual. A cylinder left on Z is a vertical puck and does not roll. The prototype carries no payload. The chassis is a flat deck, bottom z = 0.105 m and top z = 0.145 m, with a 0.14 m square pocket 2 mm deep left empty for a later tray. That pocket sits aft of the forward cameras and ahead of the rear camera, so it does not enter those views. The full visual width is 0.453 m and the length is 0.379 m, so the body fits a 0.813 m door with about 0.36 m to spare. The widest parts are the side-sonar faces at |y| = 0.227 m. The tire faces reach |x| = 0.19 m. The shoulder cameras are compact pods under the deck at y = ±0.05 m, z = 0.055 m, looking straight down. They are floor-flow cameras, not lane cameras. The four arms above the tires are the corner sonars. Their origins sit outside and above the tread, and each three-ray cone misses all four tires. Body collision is the deck box plus a box or an upright cylinder for every part that sticks past the deck: the floor pod, both floor-flow pods, the eight sonars, the rear camera, the radar dish, the stereo bar, and the oak body. None of those bounds is a mesh.

The mapper shell is that outline: nose and tail 0.19 m, half-width 0.227 m. The sweep still adds the 2 cm footprint margin. The planner disk stays 0.20 m, so one extra jamb cell does not seal the 0.81 m door. A test places the visual mesh, including the four rolled tires, inside that shell.

The downward frames are the cameras' own images on a lit beige floor. `floor_cam` is 32×32 and one color, (179, 175, 156). Each nadir frame is 128×128, the same color on 16220 pixels, and (180, 175, 156) on 164 pixels in one corner. No pixel is a tire, a chassis, or a bracket.

The physics check below is this narrowed body, headless, Webots R2025a, fast, no rendering. The floor is flat laminate. The lip is a vertical box, bottom at z = 0 and top at z = 0.0127, with no ramp. Hub depth is how far the centre sits below the seated height (0.080 m on the low floor, 0.0927 m on the lip). Contact depth is how far the contact point sits below that face. Positive would be sinking. At rest the body was level: roll 0.00°, pitch 0.00°, up component 1.00000. All four tires touched on every rest sample. Centre z = 0.08000 m, contact z = 0.00000 m, hub depth 0.00 mm, contact depth 0.00 mm. The straight run covered 2.000 m with 0.003 m of lateral drift and 0.08° of heading change. The spin turned 360.30° with 0.000 m of excursion. The climb seated all four centres at z = 0.09270 m, contact z = 0.01270 m, depth 0.00 mm, and the body was level again. It finished at x = 3.253 m, y = 0.007 m, in 4.07 s on the lip. The new colliders sit above the 12.7 mm step, so these numbers match the flat-deck check. That spin took 7.4 s. A no-scrub command at 0.22 m/s would have finished the same turn in about 4.9 s, so the hubs were sliding while the shell stayed on one spot.

The door scenario on this body crosses the lower shared-wall opening and stays. Heading is +0.15°, position error is 0.041 m, final pose is (1.31, 2.50), minimum x is 0.36 m, and the wall-press time is 0. The run is 90 s with `done` set. That opening is the west-hallway door, not letter G. One sweep at 86.3 s refused a footprint that would clip; it was already through the door.

The eight seeds below are the earlier skid body, not this flat deck and not this narrowed body. They were not re-run. They use the apartment, carpet ramp included, on the current mapper. They are a different robot from the `828e01d` tables. Every run slaved the front hubs, reached the 600 s cap, and exited 0. Lip-stall time is the sum of episodes of at least 0.5 s where true speed stayed under 0.05 m/s inside the walkway box or the letter-G box and no tire was on a wall. Airborne time is the sum of episodes of at least 0.3 s in those boxes where a hub centre was above 0.086 m and speed stayed under 0.08 m/s. The same cut was applied to the `828e01d` logs.

| | Stock | +0.40 | −0.35 | East | West | SW | North | South |
|---|---:|---:|---:|---:|---:|---:|---:|---:|
| End heading, skid | −62.10° | −9.07° | −2.93° | +12.85° | +4.65° | +3.03° | −8.67° | −0.05° |
| End heading, `828e01d` | −0.21° | +2.53° | −2.35° | +2.63° | +5.08° | +12.12° | +4.44° | −18.78° |
| End position, skid | 3.76 m | 0.27 m | 0.65 m | 0.67 m | 1.48 m | 0.47 m | 1.25 m | 0.04 m |
| End position, `828e01d` | 0.77 m | 0.42 m | 0.44 m | 0.53 m | 0.78 m | 2.06 m | 0.91 m | 1.00 m |
| Longest wall press, skid | 2.26 s | 2.68 s | 0.29 s | 4.95 s | 11.70 s | 0.92 s | 3.97 s | 0.58 s |
| Longest wall press, `828e01d` | 2.74 s | 0 | 1.71 s | 0 | 0 | 3.20 s | 0 | 0 |
| Wall-contact rows, skid | 2017 | 705 | 76 | 461 | 2632 | 301 | 977 | 74 |
| Lip stall, skid | 27.5 s | 3.8 s | 49.9 s | 54.1 s | 42.7 s | 4.0 s | 47.2 s | 0 |
| Lip stall, `828e01d` | 75.4 s | 0 | 41.9 s | 9.3 s | 33.8 s | 44.7 s | 6.5 s | 4.9 s |
| Airborne at the lip, skid | 21.3 s | 6.7 s | 38.8 s | 39.7 s | 30.5 s | 0 | 52.4 s | 0 |
| Airborne at the lip, `828e01d` | 29.7 s | 0 | 25.6 s | 11.1 s | 31.0 s | 73.5 s | 3.1 s | 2.7 s |

South finishes at −0.05° and 0.04 m, on the carpet at (0.74, 6.51), with no lip stall and no airborne episode. SW's heading comes in from +12.12° to +3.03° and its position error from 2.06 m to 0.47 m. Its walkway press drops from 3.20 s to 0.92 s, and the airborne time at the lips drops from 73.5 s to 0. The +0.40 seed now enters the bedroom. Its end position is 0.27 m. Stock is the failure: heading is −0.9° at 60 s, −27° at 180 s, and −62° at 360 s, and the end position error is 3.76 m. West does not enter the bedroom. It holds the east face beside letter G for 11.70 s (t = 89–101 s, x 3.93–3.97 m, y 4.81–4.89 m) and logs 2632 wall-contact rows. East's longest press is 4.95 s on that same face, and north's is 3.97 s at the walkway (t = 151–155 s, x 1.51 m, y 4.56–4.58 m). Seven of the eight bodies entered the bedroom. The "floor slip" line is almost absent (2 on +0.40, 1 on north, 0 on the rest). The mapper records those moments as "floor still" instead: 65, 45, 86, 66, 50, 69, 55, and 3, with 16, 9, 31, 26, 19, 22, 25, and 1 still-backups. South is the only seed with a single still-backup.

Borrowed methods, implemented here (not copied from another repo): frontier cells are free cells next to unknown space (Yamauchi, *A Frontier-Based Approach for Autonomous Exploration*, IEEE CIRA 1997); the grid is log-odds (Moravec and Elfes, *High Resolution Maps from Wide Angle Sonar*, ICRA 1985; Thrun, Burgard, and Fox, *Probabilistic Robotics*, ch. 9); the drive is pure pursuit (Coulter, *Implementation of the Pure Pursuit Path Tracking Algorithm*, CMU-RI-TR-92-01, 1992). Depth noise matches a Webots `RangeFinder` fraction of max range: 0.002 × 8 m = 1.6 cm. Wheel angles are quantized to 0.0015 rad in the mapper only. The warehouse sensors and the warehouse worlds are unchanged.

---

## 4. How it fits together (diagrams)

### The simple picture

Three things run on your computer: the **Webots simulator** (optional), the **dashboard app**, and a **database**. Your browser talks to the dashboard. The robot in Webots reports its battery and power to the dashboard, and gets "drive", "stop" or "slow down" back.

```mermaid
flowchart LR
    YOU["👤 You<br/>web browser<br/>http://127.0.0.1:5000"]

    subgraph PC["Your computer"]
        WEBOTS["🤖 Webots robot simulator<br/>ButlerBot drives a painted lane<br/>(optional)"]
        subgraph DOCKER["Docker (or plain Python)"]
            DASH["📊 Dashboard app<br/>run_dashboard.py<br/>power math · safety · agent"]
            DB[("🗄️ Postgres<br/>history")]
        end
    end

    OFFLINE["🧪 Offline tools - no Webots needed<br/>tests · rowfit_sim.py · parts catalog"]

    YOU -- "watch live numbers,<br/>press Drive / Stop" --> DASH
    WEBOTS -- "battery, power, position" --> DASH
    DASH -- "drive, stop, slow down" --> WEBOTS
    DASH -- "saves readings" --> DB
    OFFLINE -. "same robot code,<br/>runs by itself" .- WEBOTS
```

### What actually flows where (for the curious)

Each box names the file that does the job. Items marked * only switch on in the obstacle or warehouse worlds.

```mermaid
flowchart TB
    subgraph HOST["① Webots process on your PC - webots/controllers/butlerbot_controller/"]
        direction LR
        SENS["Sensors<br/>GPS · IMU · encoders<br/>nadir cameras L/R<br/>stereo + radar*"]
        VIS["Lane vision<br/>src/lane_vision.py<br/>src/junction_vision.py"]
        OBS["Obstacles*<br/>controller_obstacles.py<br/>src/obstacle_gate.py"]
        INV["Pick list*<br/>controller_inventory.py<br/>src/shopping_list.py"]
        DRIVE["Steer + speed<br/>controller_rowfit.py<br/>src/rowfit_control.py"]
        WHEELS["Wheels · ABS · stop<br/>controller_wheels.py"]
        POWER["Power estimate<br/>src/twin/webots_power.py"]
        PUB["twin_publisher.py"]
        SENS --> VIS --> DRIVE
        SENS --> OBS -- "speed cap" --> DRIVE
        INV -- "stop / slow" --> DRIVE
        DRIVE --> WHEELS --> POWER --> PUB
    end

    subgraph APP["② Dashboard process - run_dashboard.py (Docker 'dashboard' container, or local Python)"]
        direction LR
        API["Flask HTTP<br/>src/dashboard.py"]
        BRIDGE["DigitalTwinBridge<br/>src/twin/bridge.py<br/>latest feed · stop_epoch"]
        TICK["PMS tick<br/>src/hardware_ros2.py"]
        PMS["Allocator · safety · LRU<br/>forecast · mission<br/>src/onboard_agent.py"]
        API <--> BRIDGE
        BRIDGE -- "Webots feed if fresh,<br/>else internal sim" --> TICK
        TICK <--> PMS
        PMS -- "agent throttle" --> BRIDGE
        TICK -- "live state" --> API
    end

    subgraph OUT["③ Storage and screens"]
        direction LR
        DB[("Postgres 16<br/>Docker 'postgres'<br/>src/database.py")]
        BROWSER["Browser<br/>ButlerBot Live Monitor"]
        CLI["robot_battery_monitor.py<br/>terminal reports"]
    end

    PUB -- "POST /api/twin/telemetry" --> API
    API -- "GET /api/twin/state:<br/>drive · stop_epoch · throttle" --> PUB
    TICK -- "write readings" --> DB
    DB -- "history / analytics" --> API
    DB --> CLI
    API -- "GET /api/live every 500 ms" --> BROWSER
    BROWSER -- "POST /api/twin/command:<br/>drive · drive_stop · battery_reset" --> API
```

How to read it:

1. **Inside Webots** (top): the cameras see the yellow lane, `lane_vision` turns pixels into "where is the lane", `rowfit_control` turns that into steering and speed, and `controller_wheels` drives the motors and handles stopping. The power estimate is computed from what the motors are doing.
2. **Over HTTP** (middle arrows): about twice a second the robot **POSTs** its readings to the dashboard and **GETs** back any commands (drive, stop, throttle). This is plain web traffic. ROS 2 is not needed for it.
3. **Inside the dashboard app**: the Twin Bridge keeps the latest robot reading. The PMS tick (Power Management System) uses that reading when it is fresh, or its own built-in simulation when Webots is not running, then runs the budget, safety and agent rules and saves a snapshot to Postgres.
4. **Out to you**: the browser polls `GET /api/live` every 500 ms, and its Drive/Stop buttons send commands back the same way.

More diagrams (stop handshake, per-tick sequence, deployment): [`docs/ARCHITECTURE.md`](docs/ARCHITECTURE.md).

---

## 5. Pick your setup

| I want to... | Go to | Install | Time |
|--------------|-------|---------|------|
| Understand the project without installing anything | [A](#a-just-look-around-no-install) | Nothing | 10 min |
| Run the offline robot simulator and tests | [B](#b-run-the-lightweight-simulator-and-tests-no-webots) | Git, Python 3.12 (Docker optional, for the database tests) | 20-30 min |
| See the robot drive and the dashboard live (Windows) | [C](#c-full-windows-setup-dashboard--webots-robot) | Git, Docker Desktop, Webots, Python | 45-60 min (mostly downloads) |
| Same on Linux or Mac (**untested**) | [D](#d-linux-or-macos-untested) | Git, Docker, Webots | 45-60 min |
| Change code and send it back | [E](#e-developer--contributor-workflow) | All of B | - |

Ports used: **5000** (dashboard) and **5432** (database), both only reachable from your own computer.

---

### A. Just look around (no install)

1. Read [Section 1](#1-what-is-this-in-plain-english) and the [diagrams](#4-how-it-fits-together-diagrams) above.
2. Open [`docs/NORTH_STAR.md`](docs/NORTH_STAR.md): the "why" and the build order.
3. Open [`docs/CHEAT_SHEET.md`](docs/CHEAT_SHEET.md): the latest state of the robot's driving brain.
4. Browse [`webots/README.md`](webots/README.md) for the simulator side and [`docs/TRACKS.md`](docs/TRACKS.md) for the test tracks.
5. Want to read code? Start with these, in order:

| File | What it does |
|------|--------------|
| `run_dashboard.py` | Starts everything: database → power loop → web dashboard |
| `src/dashboard.py` | The web page, the `/api/twin/*` endpoints and the live push |
| `src/twin/bridge.py` | The "post office" between Webots and the dashboard |
| `src/hardware_ros2.py` | The main power loop: budget, safety, agent, database |
| `webots/controllers/butlerbot_controller/butlerbot_controller.py` | Webots entry point. `_run_loop` calls the step modules below |
| `webots/controllers/butlerbot_controller/controller_devices.py` | Device setup, BoM reload, wheel commands |
| `webots/controllers/butlerbot_controller/controller_sensing.py` | IMU, follow camera, GPS / encoder reads |
| `webots/controllers/butlerbot_controller/controller_mission.py` | Route, R-key phases, nadir harvest, mission end |
| `webots/controllers/butlerbot_controller/controller_energy.py` | Pack watt-hours and telemetry POST |
| `src/lane_vision.py`, `src/rowfit_control.py` | How the robot sees the lane and steers |
| `config/config.yaml` | Most settings |

---

### B. Run the lightweight simulator and tests (no Webots)

Good for checking the project works on your machine, or trying a driving change quickly.

**You need:** [Git](https://git-scm.com/downloads) and [Python 3.12](https://www.python.org/downloads/) (tested with 3.12.10 on Windows; CI uses 3.12; on Windows tick **"Add python.exe to PATH"** during install). Optional: a Postgres database for the ~30 database tests; the easiest way is [Docker Desktop](https://www.docker.com/products/docker-desktop/).

**Step 1. Get the code**

```
git clone https://github.com/Daniel7505/robot-battery-monitor.git
cd robot-battery-monitor
```

**Step 2. Make a private Python environment and install the packages**

Windows (PowerShell):
```
python -m venv .venv
.\.venv\Scripts\Activate.ps1
pip install -r requirements.txt pytest
```

Mac / Linux:
```
python3 -m venv .venv
source .venv/bin/activate
pip install -r requirements.txt pytest
```

Your prompt now starts with `(.venv)`. Run the activate line again whenever you open a new terminal.

**Step 3. Run the offline lane-following simulator** (no database, no Webots)

```
python scripts/rowfit_sim.py --table
```

It takes about 15 seconds. ✅ **It worked if** the last lines look like this (numbers may differ slightly):

```
course          finish time_s worst_cm mean_cm min_v pivots  pivot_err_deg(imu/true) pivot_s  stop
s               yes      40.8      7.8     2.9  0.20      0                        -       -  finish
turn90          yes      15.7      7.4     3.0  0.20      0                        -       -  finish
```

`finish yes` means the robot completed the course; `worst_cm` is the furthest it drifted from the lane centre.

More to try:

```
python scripts/rowfit_sim.py corner90 --table --quiet       # a sharp 90° corner
python scripts/rowfit_sim.py --help                          # every option
python -m src.parts_db                                       # list the parts catalog
python -m src.design_agent --length 17                       # suggest cheapest parts list for a 17 m track
```

Any file in `tracks/` (for example `corner_mix`, `plus`, `t_end`, `widen`) can be used as a course name.

**Step 4. Run the automated tests**

```
python -m pytest
```

That works with or without a database:

- **No Postgres running:** the 27 tests marked `db` are **skipped**, and everything else runs. One more test skips when the `docker` command is not on PATH. ✅ **It worked if** the last line shows `passed` and `skipped`, with no `failed` or `error`.
- **With Postgres:** those 27 run. The only skip is `test_docker_compose_config_valid` when the docker CLI is absent. ✅ **It worked if** the last line shows `passed` (and `skipped` only for that docker check), with no `failed` or `error`. pytest collects 880 tests.

The full run takes about 5 minutes. To run just one file: `python -m pytest tests/test_lane_keep.py`.

**Optional: run the database tests too**

1. Copy the settings file once (Windows: `copy .env.example .env`, Mac/Linux: `cp .env.example .env`).
2. Start just the database with Docker and create the tables (this also creates the test database `robot_battery_test`):

   ```
   docker compose up -d postgres
   ```
   Wait about 10 seconds, then:

   Windows (PowerShell):
   ```
   $env:PGPASSWORD="robot"
   python scripts/setup_postgres.py
   ```
   Mac / Linux:
   ```
   export PGPASSWORD=robot
   python scripts/setup_postgres.py
   ```
3. Run `python -m pytest` again.

The tests empty the tables of `robot_battery_test` only. Your dashboard history in `robot_battery` is not touched.

Already have Postgres installed without Docker? Create a `robot` user (password `robot`) with permission to create databases and run `scripts/setup_postgres.py` as above, or point the tests elsewhere with `TEST_DATABASE_URL=postgresql://user:pass@host:5432/robot_battery_test`.

---

### C. Full Windows setup: dashboard + Webots robot

This is the main demo: the robot drives in Webots and the dashboard shows its battery live.

**You need**

| Tool | Why | Link |
|------|-----|------|
| Git | Download the code | https://git-scm.com/downloads |
| Docker Desktop | Runs the dashboard and database for you (no Python needed for that part) | https://www.docker.com/products/docker-desktop/ |
| Webots R2025a (R2023b or newer) | The 3D robot simulator. Runs on Windows, **not** in Docker | https://cyberbotics.com/ |
| Python 3.12 (tested with 3.12.10 on Windows) | Webots runs the robot's controller code with the Python on your PC. After installing, check **Webots → Tools → Preferences → Python command** points at it | https://www.python.org/downloads/ (tick "Add python.exe to PATH") |
| A browser | To view the dashboard | Chrome, Edge, Firefox |

About 5-10 GB of downloads in total.

**Step 1. Install and start Docker Desktop.** Reboot if asked. Start it and wait until it says **Engine running** (the first start can take a few minutes; WSL2 is normal).

**Step 2. Get the code** (PowerShell):
```
git clone https://github.com/Daniel7505/robot-battery-monitor.git
cd robot-battery-monitor
```

**Step 3. Start the dashboard and database**
```
.\scripts\start.ps1
```
The first run builds the app and takes a few minutes. It copies `.env.example` to `.env` for you if you don't have one.

✅ **It worked if** it prints `Dashboard ready: http://127.0.0.1:5000`. Open that address: you should see **ButlerBot Live Monitor** with numbers changing on their own.

**Step 4. Start the robot in Webots** (with the dashboard still running)
```
.\scripts\launch_webots_twin.ps1
```
Other worlds:
```
.\scripts\launch_webots_twin.ps1 -World corner90      # sharp corner
.\scripts\launch_webots_twin.ps1 -World plus          # crossroads
.\scripts\launch_webots_twin.ps1 -World warehouse     # warehouse
```
Full list: `butlerbot` (the S, default), `corner90`, `corner_mix`, `widen`, `plus`, `t_end`, `gap45`, `warehouse`, `warehouse_obstacles`, `radar_motion`. You can also open Webots yourself and use **File → Open World → `webots/worlds/butlerbot.wbt`**, then press Play. Every world instances the same robot from [`webots/protos/ButlerBot.proto`](webots/protos/ButlerBot.proto).

✅ **It worked if** the robot starts driving along the yellow lane and the dashboard badge says **TWIN LIVE**.

**Step 5. Stop everything**

1. Close Webots. When it asks to save the world, choose **No / Don't save** (saving writes the robot's position into the world file).
2. Stop the dashboard and database:
   ```
   .\scripts\stop.ps1
   ```

**Optional extras**
- One-click demo (starts the dashboard if needed, turns on demo highlighting, opens Webots): `.\scripts\launch_demo.ps1`
- Extra ROS 2 test container (not needed for the demo): `.\scripts\start.ps1 -Profile full`
- Run the dashboard without Docker: follow [B](#b-run-the-lightweight-simulator-and-tests-no-webots) steps 1-2 and "Optional: run the database tests too" steps 1-2, then:
  ```
  $env:DATABASE_URL="postgresql://robot:robot@localhost:5432/robot_battery"
  python run_dashboard.py
  ```
  Press Ctrl+C to stop it.
- Terminal summary of the latest readings (needs the database running and `DATABASE_URL` set as above): `python robot_battery_monitor.py --summary`

---

### D. Linux or macOS (untested)

> **Untested.** The project has only been run on Windows. These scripts exist and should work, but nobody has run this path end to end yet. If you try it, please open an issue with what happened.

Same as C, with these differences:

1. Install [Docker](https://docs.docker.com/get-docker/) (Docker Desktop on Mac), [Webots](https://cyberbotics.com/) and Python 3.12. On Linux the launcher looks for `webots` on your PATH or in `/usr/local/webots` (`WEBOTS_HOME` overrides it).
2. Use the `.sh` scripts:
   ```
   ./scripts/start.sh                    # dashboard + database
   ./scripts/launch_webots_twin.sh       # robot (default S world)
   ./scripts/launch_webots_twin.sh http://127.0.0.1:5000 corner90   # another world
   ./scripts/stop.sh                     # stop dashboard + database
   ```
3. `./scripts/start.sh full` adds the optional ROS 2 container.
4. Note: `launch_webots_twin.sh` closes any Webots window that is already open before starting a new one.

---

### E. Developer / contributor workflow

**Before you change anything** read [`docs/REVIEW_CHECKLIST.md`](docs/REVIEW_CHECKLIST.md). It lists the parts that must not be rewritten casually (stop/ABS and power core) and how credits are recorded.

1. **Set up** as in [B](#b-run-the-lightweight-simulator-and-tests-no-webots) (venv, `pip install -r requirements.txt pytest`, Postgres up).
2. **Branch** from `main` with a descriptive name, following the existing pattern:
   ```
   git checkout main && git pull
   git checkout -b feature/my-change
   ```
3. **Make one coherent change at a time.** Driving changes should be checked offline first:
   ```
   python scripts/rowfit_sim.py corner90 corner_mix --table --quiet
   ```
   then in Webots on the matching world.
4. **Run the tests** before pushing: `python -m pytest` (or one file: `python -m pytest tests/test_lane_keep.py`). New tests that need Postgres should use the `clean_database` fixture or `@pytest.mark.db` so they skip cleanly on machines without a database.
5. **Open a pull request into `main`.** GitHub Actions ([`.github/workflows/tests.yml`](.github/workflows/tests.yml)) runs the full test suite on Python 3.12 against a Postgres 16 service for every push and pull request to `main`. In CI a missing database is an error, never a skip (`RBM_REQUIRE_DB=1`). Merge only when it is green.
6. **Restore points:** before and after a feature, the project tags `main` as `save-YYYY-MM-DD-<name>` (for example `save-2026-10-05-pre-cruise-v1` and `save-2026-10-05-cruise-v1`). To look at an old state: `git checkout save-2026-10-05-cruise-v1`. To list them: `git tag -l "save-*"`.
7. **Webots worlds are generated.** Track worlds come from `tracks/*.json` via `scripts/track_builder.py`, the S from `scripts/s_track.py --write-world`, and the warehouse from `scripts/warehouse_builder.py` (see [`docs/TRACKS.md`](docs/TRACKS.md), [`docs/WAREHOUSE.md`](docs/WAREHOUSE.md)). Regenerate instead of hand-editing, and never save a world from the Webots exit prompt.
8. **Never commit secrets.** Settings go in `.env` (ignored by git); `.env.example` is the template.
9. **Log the story.** When you finish a meaningful arc (or get stuck on one), add an entry to [`JOURNEY.md`](JOURNEY.md).

---

### F. Overnight optimizer (Windows)

A script you can start before bed. It runs warehouse laps one after another, changes **one** allowlisted setting at a time, and leaves `optimizer/runs/morning.md` for the morning. Webots has to be on this PC. The cloud and Linux cannot launch it.

What the overnight loop tunes, in `config/config.yaml`. These are exactly `OPTIMIZER_TUNING_KEYS` in `src/controller_limits.py`:

| Key | What it does |
| --- | --- |
| `controller.cruise_m_s` | Rowfit curve / lane cruise |
| `controller.pivot_w_max_rad_s` | In-place turn rate |
| `controller.k_steer` | How hard the steer command is |
| `controller.coast_enabled` | Coast into a turn instead of braking to a stop |
| `controller.coast_margin_m` | How early that coast starts |

`bom_build` stays `light` for the whole run. `controller.straight_m_s` is still read by the controller and is not tuned. Heavy-build confirm is off: a heavy lap is not a held-out physics check (see [`optimizer/KNOWLEDGE.md`](optimizer/KNOWLEDGE.md)). The loop does **not** edit the energy model or `config/bom/bom.json` to chase a better score. A change that looks good is checked with two fresh light laps (the value before the keep, and the kept value). Ideas for code changes are written in the report and are not committed. When the loop stops, those five tuning keys are put back to what they were at the start.

```
powershell -ExecutionPolicy Bypass -File scripts\start.ps1
powershell -ExecutionPolicy Bypass -File scripts\overnight_optimize.ps1
```

`overnight_optimize.ps1` starts the dashboard if it is down, then runs laps with `RBM_MISSION_LAPS=1` through `scripts\launch_webots_twin.ps1` (warehouse world). Defaults are 10 iterations, 8 hours, 40 minutes per lap. Shorter night:

```
powershell -ExecutionPolicy Bypass -File scripts\overnight_optimize.ps1 -MaxIterations 4 -MaxHours 6
```

In the morning open `optimizer/runs/morning.md`. The same tries are in `results.jsonl` and `results.csv`. Learnings are appended to `optimizer/KNOWLEDGE.md`. If you want a confirmed setting kept, copy it into `config.yaml` yourself.

A no-Webots smoke test (synthetic numbers, not a measurement):

```
python -m optimizer --dry-run --max-iterations 2
```

An optional later picker can shell out to a command you supply. Nothing in the repo holds an API key.

```
set OPTIMIZER_LLM_CMD=python my_picker.py
powershell -ExecutionPolicy Bypass -File scripts\overnight_optimize.ps1 -Picker llm
```

The command reads a JSON request on stdin and prints one JSON proposal (`param`, `value`, `hypothesis`, `expected_effect`). A proposal outside the allowlist is ignored and the rule picker is used instead.

---

## 6. Reading the dashboard

The page polls `GET /api/live` every 500 ms.

| On the page | Meaning |
|-------------|---------|
| **Battery** | Pack percent |
| **System draw** | Sum of channel watts, and percent of the power budget |
| **Runtime left** | How long the pack lasts at the current draw |
| **Energy left** | Watt-hours still in the pack |
| **This lap** | Badge **TWIN LIVE** or **AWAITING TWIN**. Phase, speed, position, mission time, energy used, battery change, distance, average speed. Teleop: Forward, Back, Left, Right, Stop, Lane keep, and a steer bar |
| **Proof** | Measured vs model drive watt-hours, steer source (camera / held / track), cross-track, width error, center error |

The previous engineering view is on branch `archive/engineering-dashboard`. See [`docs/ENGINEERING_DASHBOARD_ARCHIVE.md`](docs/ENGINEERING_DASHBOARD_ARCHIVE.md). That archive stays in the repo.

---

## 7. Settings you might change

All in `.env` (copied from [`.env.example`](.env.example)). After editing, restart: `.\scripts\stop.ps1` then `.\scripts\start.ps1` (or the `.sh` versions).

| Setting | Default | What it does |
|---------|---------|--------------|
| `HARDWARE_MODE` / `HARDWARE_TYPE` / `ROS2_MOCK` | `real` / `ros2` / `true` | The full realistic power loop with mocked ROS 2. **Use this with Webots.** Set `HARDWARE_MODE=simulator` for simple random numbers (dashboard check only; Webots readings are not used) |
| `DASHBOARD_PORT` | `5000` | Change if port 5000 is busy |
| `POSTGRES_PASSWORD` | `robot` | Dev-only password. Change it (and `DATABASE_URL`) for anything beyond your own laptop |
| `RBM_API_TOKEN` | empty | Optional lock on the buttons and API. If set, open `http://127.0.0.1:5000/?token=<value>` once in the browser. Webots and the scripts pick it up automatically |
| `RBM_ROUTE` | unset | Which way the robot goes at each junction, e.g. `L,S,R` |
| `RBM_LANE_MODE` | unset | Lane following is row-fit. A leftover `gap` is ignored. |

Deeper knobs live in [`config/config.yaml`](config/config.yaml) and the hardware profiles in `config/hardware_profiles/`.

### Which battery pack?

`battery_capacity_wh()` in `src/hardware_profile.py` is the pack the teleop drain, the simulation driver, and the Webots fallback use.

| `bom_build` | Pack |
| --- | --- |
| blank (the default) | Wheeled YAML pack, **480 Wh** |
| `light` | BoM pack, **240 Wh** (Bioenno 24V 10Ah LiFePO4) |

The overnight optimizer and reference laps #8–#12 use `light`. A blank `bom_build` still means the wheeled 480 Wh YAML pack.

**Security defaults:** the dashboard and database listen on `127.0.0.1` only (not reachable from other computers). `/api/demo/launch-webots` only answers local requests.

---

## 8. Common problems

| Problem | Fix |
|---------|-----|
| `start.ps1 cannot be loaded because running scripts is disabled` | Run it as `powershell -ExecutionPolicy Bypass -File scripts\start.ps1` (same for the other `.ps1` scripts), or once: `Set-ExecutionPolicy -Scope CurrentUser RemoteSigned` |
| `./scripts/start.sh: Permission denied` (Mac/Linux) | Run `chmod +x scripts/*.sh` once (or use `sh scripts/start.sh` / `bash scripts/launch_webots_twin.sh`) |
| `docker: command not found` / "Cannot connect to the Docker daemon" | Start Docker Desktop and wait for **Engine running** |
| `Port 5000 already in use` | Close the other program, or set `DASHBOARD_PORT=5001` in `.env` and open `http://127.0.0.1:5001` |
| `set POSTGRES_PASSWORD in .env` error from Docker | You have no `.env`. Run `copy .env.example .env` (Windows) or `cp .env.example .env` |
| Dashboard page doesn't load | Wait 30 s and refresh. Check `docker compose logs dashboard` |
| Many tests `skipped` with "Postgres test database not reachable" | Normal without a database. To run them, see "Optional: run the database tests too" in [B](#b-run-the-lightweight-simulator-and-tests-no-webots) |
| Badge stays **AWAITING TWIN** while Webots runs | Start the dashboard **before** Webots. The page polls `GET /api/live` every 500 ms. Check `http://127.0.0.1:5000/api/twin/schema` opens in the browser. Make sure `HARDWARE_MODE` is not `simulator` |
| Webots console: `Twin publish failed` | Same as above: the dashboard isn't reachable |
| Webots: controller fails to start / Python not found | Install Python 3.12 (64-bit) and make sure `python` works in a new terminal; in Webots check **Tools → Preferences → Python command** |
| Obstacle or warehouse worlds error on `numpy` | `pip install numpy` (and optionally `opencv-python`) for the Python that Webots uses |
| Robot falls through the floor | Reset the simulation (Ctrl+Shift+T) |
| `git status` shows a changed `.wbt` file | You saved the world on exit. Undo with `git checkout -- webots/worlds/<file>.wbt` |

---

## 9. Where things live (folder map)

| Folder / file | Contents |
|---------------|----------|
| `run_dashboard.py` | Starts the dashboard app (database → power loop → web page) |
| `robot_battery_monitor.py` | Terminal reports from the database (`--summary`, `--history Legs`, `--report`) |
| `src/` | All the Python logic: dashboard, power loop, safety, agent, lane vision, steering, obstacles, parts catalog |
| `src/mapping/` | Apartment occupancy grid, odometry, frontiers, and the exploration loop. No ground truth |
| `eval/` | Apartment scorer and the kinematic depth stand-in. This is the code allowed to read the tape |
| `src/twin/` | The Webots ↔ dashboard bridge and power model |
| `webots/` | Simulator worlds and the robot controller ([`webots/README.md`](webots/README.md)) |
| `tracks/` | Test track definitions (JSON) |
| `scripts/` | Start/stop scripts, Webots launchers, offline simulator, measurement tools, world builders |
| `optimizer/` | Overnight lap loop, knowledge notes, morning report (`runs/` is local, not committed) |
| `config/` | `config.yaml` settings and hardware profiles |
| `data/parts.db` | The parts catalog (SQLite) |
| `tests/` | Automated tests (`tests/fake_webots/` stands in for Webots) |
| `docs/` | Design notes, baselines, run history and architecture |
| `JOURNEY.md` | Dated project log: how the project was built with AI tools, what went wrong, what worked |
| `docker-compose.yml`, `Dockerfile` | The Docker setup (dashboard + Postgres, optional ROS 2) |

---

## 10. Credits and Licenses

This section lists third-party software and assets used by the project, what license applies, and what you must keep when you redistribute or ship binaries. It also records an **attribution audit** (Oct 2026): searched `requirements.txt`, Docker/CI manifests, Webots worlds, bundled images, and source comments/URLs for copied or adapted material.

There is **no** `package.json`, `pyproject.toml`, or ROS 2 `package.xml` in this repository.

### This repository

| Item | Link | License | Notes |
|------|------|---------|--------|
| **robot-battery-monitor** (this repo) | [GitHub](https://github.com/Daniel7505/robot-battery-monitor) | [MIT](LICENSE) | [`LICENSE`](LICENSE) is present (Copyright (c) 2026 Daniel). Keep the MIT notice in copies or substantial portions. |

### Python — pinned in `requirements.txt` (dashboard / Docker)

Installed by `pip install -r requirements.txt` and the [`Dockerfile`](Dockerfile). Matplotlib pulls in additional transitive packages (NumPy, Pillow, fonttools, etc.); run `pip install pip-licenses && pip-licenses --from=mixed` in your environment for the full tree.

| Package | Version (pinned) | Link | License | Redistribution notes |
|---------|------------------|------|---------|----------------------|
| [Flask](https://pypi.org/project/Flask/) | 3.1.0 | https://pypi.org/project/Flask/ | BSD-3-Clause | Retain copyright and license notices. |
| [PyYAML](https://pypi.org/project/PyYAML/) | 6.0.2 | https://pyyaml.org/ | MIT | Retain copyright and permission notice. |
| [matplotlib](https://pypi.org/project/matplotlib/) | 3.10.0 | https://matplotlib.org/ | [PSF / Matplotlib license](https://matplotlib.org/stable/project/license.html) (BSD-style) | Retain the Matplotlib Development Team copyright notice; brief change summary if you distribute a derivative of matplotlib itself. |
| [psycopg2-binary](https://pypi.org/project/psycopg2-binary/) | 2.9.10 | https://www.psycopg.org/ | **LGPL-3.0-or-later** (library) | **Copyleft (weak).** Shipping the prebuilt wheel (e.g. in a Docker image) triggers LGPL obligations: provide license text, allow relinking/replacement of the library, and document how users can substitute their own build. See the [psycopg2 license](https://www.psycopg.org/license/). |

### Python — CI and tests only

| Package | Where | Link | License | Notes |
|---------|-------|------|---------|--------|
| [pytest](https://pypi.org/project/pytest/) | [`.github/workflows/tests.yml`](.github/workflows/tests.yml) (`pip install pytest`) | https://docs.pytest.org/ | MIT | Not in `requirements.txt`; used only in CI/local test runs. |

### Python — optional on the host (Webots / scripts)

Not pinned in `requirements.txt`; install manually when using obstacle worlds, `scripts/warehouse_builder.py`, or host-side Webots controllers (see [`webots/README.md`](webots/README.md)).

| Package | Link | License | Notes |
|---------|------|---------|--------|
| [NumPy](https://pypi.org/project/numpy/) | https://numpy.org/ | BSD-3-Clause | Used by `src/stereo_depth.py`, `src/stereo_synth.py`, tests, and warehouse tooling. Also installed transitively via matplotlib in Docker. |
| [opencv-python](https://pypi.org/project/opencv-python/) | https://opencv.org/ | Apache-2.0 | Optional faster stereo backend (`StereoSGBM` in `src/stereo_depth.py`). Not required for CI (numpy fallback). |
| [Pillow](https://pypi.org/project/pillow/) | https://python-pillow.org/ | HPND (permissive) | Used only when regenerating textures/previews via `scripts/warehouse_builder.py` (`write_prop_texture`, `draw_preview`). |

### Docker images and base OS packages

| Component | Where | Link | License | Notes |
|-----------|-------|------|---------|--------|
| [python:3.12-slim](https://hub.docker.com/_/python) | [`Dockerfile`](Dockerfile) | https://www.python.org/psf/license/ | Python Software Foundation License | Base image for the dashboard container. |
| Debian packages (`libglib2.0-0`, `libsm6`, `libxext6`, `libxrender-dev`) | [`Dockerfile`](Dockerfile) `apt-get` | https://www.debian.org/legal/licenses/ | Mixed (mostly GPL/LGPL for libs) | Runtime deps for matplotlib; standard container practice is to ship with the image and honor Debian copyright files in `/usr/share/doc/`. |
| [postgres:16-alpine](https://hub.docker.com/_/postgres) | [`docker-compose.yml`](docker-compose.yml) | https://www.postgresql.org/about/licence/ | PostgreSQL License (permissive, BSD-like) | Database service; not linked into application Python code. |
| [postgres:16](https://hub.docker.com/_/postgres) | [`.github/workflows/tests.yml`](.github/workflows/tests.yml) CI service | same | same | CI-only Postgres instance. |
| [osrf/ros:rolling-ros-base](https://hub.docker.com/r/osrf/ros) | [`docker-compose.yml`](docker-compose.yml) (`profile: full`) | https://www.ros.org/ | Mostly Apache-2.0 and BSD-3-Clause per package | Optional ROS 2 peer; not used in default `docker compose up`. |

### GitHub Actions (CI)

| Action / image | Where | Link | License |
|----------------|-------|------|---------|
| [actions/checkout@v4](https://github.com/actions/checkout) | [`.github/workflows/tests.yml`](.github/workflows/tests.yml) | https://github.com/actions/checkout | MIT |
| [actions/setup-python@v5](https://github.com/actions/setup-python) | same | https://github.com/actions/setup-python | MIT |

### Webots simulator and PROTO assets

| Item | Link | License | Notes |
|------|------|---------|--------|
| [Webots](https://cyberbotics.com/) (host install, not in repo) | https://cyberbotics.com/ | [Apache-2.0](https://github.com/cyberbotics/webots/blob/master/LICENSE) (open-source distribution) | Worlds and controllers target Webots **R2023b+** (tested with **R2025a**). You install Webots separately; it is not vendored in git. |
| **Parquetry.proto** | `EXTERNPROTO` in all `webots/worlds/butlerbot*.wbt` → [Cyberbotics Webots R2025a](https://raw.githubusercontent.com/cyberbotics/webots/R2025a/projects/appearances/protos/Parquetry.proto) | Apache-2.0 (Webots project) | Loaded from GitHub at world load time; not copied into this repo. |
| **Floor.proto** | same pattern → [Floor.proto](https://raw.githubusercontent.com/cyberbotics/webots/R2025a/projects/objects/floors/protos/Floor.proto) | Apache-2.0 (Webots project) | same |

All ButlerBot `.wbt` worlds under [`webots/worlds/`](webots/worlds/) reference only these two external PROTOs, plus the local [`webots/protos/ButlerBot.proto`](webots/protos/ButlerBot.proto). That robot PROTO, the track paint, the warehouse layout, and the controllers in this repo are **original to this project** (generated or authored here).

### Bundled images and models (in git)

| Asset | Path | Origin | License |
|-------|------|--------|---------|
| Obstacle prop speckle texture | [`webots/worlds/textures/obstacle_speckle.png`](webots/worlds/textures/obstacle_speckle.png) | Generated by `scripts/warehouse_builder.py` (`write_prop_texture`, deterministic NumPy noise) | Same as this repo (**MIT**) |
| Warehouse layout preview | [`docs/warehouse_preview.png`](docs/warehouse_preview.png) | Generated by `scripts/warehouse_builder.py` (`draw_preview`) | Same as this repo (**MIT**) |
| Track calibration / paths | `webots/worlds/s_track_path.csv`, `webots/worlds/track_calibration.json`, [`tracks/*.json`](tracks/) | Project data | Same as this repo (**MIT**) |
| Parts catalog DB | [`data/parts.db`](data/parts.db) | SQLite BOM seeded from project config (`config/bom/`, `src/parts_db.py`); vendor **URLs** point to public product pages, not copied datasheets | Same as this repo (**MIT**); respect third-party site terms when visiting links |

No `.proto` files, font files, 3D mesh assets (`.obj`/`.dae`/`.stl`), or external texture packs are committed besides the PNGs above.

### Attribution audit — code, algorithms, and comments

Searched the tree for `EXTERNPROTO`, `Copyright`, `SPDX`, `adapted`, `copied`, `based on`, and `http(s)://` in source headers. **Findings:**

| Topic | Verdict |
|-------|---------|
| **Copied third-party source files** | None found. No vendored upstream modules besides Python packages installed via pip/Docker. |
| **`tests/fake_webots/controller.py`** | Original minimal stub for CI; not the Cyberbotics Webots `controller` module. |
| **`src/lane_vision.py`** | Original; stdlib-only by design. |
| **`src/rowfit_control.py` / pure pursuit** | Standard geometric path-following algorithm described in docs ([`docs/ROWFIT_LANE_VISION.md`](docs/ROWFIT_LANE_VISION.md)); **implemented in-repo**, not pasted from an external repo. |
| **`src/mapping/`** | Frontier exploration (Yamauchi, IEEE CIRA 1997), log-odds occupancy (Moravec and Elfes, ICRA 1985; Thrun, Burgard, Fox, *Probabilistic Robotics*, ch. 9), pure pursuit (Coulter, CMU-RI-TR-92-01, 1992), phase-correlation floor flow (Kuglin and Hines, 1975), a scalar-Kalman pose update, landmark-voting loop closure, and a correlative scan match used as a done-gate check (Olson, ICRA 2009). Implemented in-repo from those descriptions. |
| **`src/stereo_depth.py`** | Original pipeline; calls OpenCV `StereoSGBM` when available, else an **original** NumPy block matcher (documented in file header). |
| **Motor / battery constants** | Comments and YAML cite [Pololu](https://www.pololu.com/), [Adafruit](https://www.adafruit.com/), and similar **datasheet URLs** for engineering values — reference only, not copied code. |
| **Hardware BOM JSON** | `config/bom/bom.json` and profiles link to vendor product pages; no scraped proprietary text committed as code. |
| **Archived world snippet** | [`archives/butlerbot_track_boxes_2026-10-01.wbt`](archives/butlerbot_track_boxes_2026-10-01.wbt) is an in-repo archive of an older track representation, not third-party. |

**GPL / strong copyleft in application dependencies:** only **psycopg2-binary (LGPL-3.0)** among pinned Python wheels. No GPL-licensed application code was identified in the repo. **Unlicensed third-party material:** none identified; all bundled assets above are project-authored or generated by project scripts.

**ROS 2:** the live hardware path is [`src/hardware_ros2.py`](src/hardware_ros2.py) through [`src/ros2_bridge.py`](src/ros2_bridge.py). Default runs use `ROS2_MOCK=true` without installing ROS. Optional `osrf/ros` image noted above. The old standalone publisher `src/ros2_node.py` is gone; it duplicated the bridge and had no Cooling channel.

### People and AI tools

This repo is a **human + AI co-build** (credit where credit is due):

| Who | Role |
|-----|------|
| **Daniel** ([Daniel7505](https://github.com/Daniel7505)) | Owner — goals, visual ground truth, pacing, final say |
| **Grok Build 4.5** (xAI) | Majority of the deep build — twin bridge, teleop/ABS/stop reliability, power feed, agent, Webots controller work, tests, architecture docs |
| **Grok Bot** | Repo review, tests and CI, security hardening, navigation features (junctions, radar, warehouse), and this documentation refresh (Oct 2026 onward) |
| **Grok Web ~4.4 → 4.5** (xAI) | Project kickoff and early product framing |

For future models (and humans doing a serious review), the standing fence is:

**[`docs/REVIEW_CHECKLIST.md`](docs/REVIEW_CHECKLIST.md)** — look here first; do not silently rewrite the stop/power core; append your lineage row when you ship a meaningful arc.

---

## 11. Project journey

How this project got here (copy-paste chat sessions → repo-aware chat → a coding agent in the terminal → a full agent with GitHub and CI), with dated milestones, the problems along the way, and what each step taught: **[`JOURNEY.md`](JOURNEY.md)**.

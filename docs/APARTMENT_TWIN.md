# Apartment twin

Dan's Halcyon Castle Hills plan A2 (1 bed, 1 bath, about 730–750 sq ft) is the next ButlerBot course after the warehouse. The warehouse world stays the regression course. This world has no lane paint. It is for vision and SLAM later.

Launch it from the repo root (the dashboard twin should already be running):

```bat
powershell -ExecutionPolicy Bypass -File scripts\launch_webots_twin.ps1 -World apartment
```

That opens `webots/worlds/apartment.wbt`. The same name works as `RBM_WORLD=apartment`.

## No foresight

The world is built from Dan's tape and photos. Those numbers are **ground truth for scoring a run after the fact**. They are not a map the robot is allowed to load.

`eval/apartment_ground_truth.json` holds the measured inches, the estimated rooms, the furniture boxes, and the floor-plan frame. The ButlerBot controller may not import or open that file. It has its onboard cameras, radar, IMU, and wheel encoders only, and it has to build its own map. `tests/test_apartment_twin.py` walks the controller package and every module those files import, and fails if any of them name the file.

The `.wbt` geometry is the apartment the sensors will see. It is not a lookup table. The inch table below is for people scoring a run, same as the JSON.

## Frame

Webots R2025a, ENU, Z up.

* +X runs from the bedroom's left wall toward the living room's right wall.
* +Y runs from the kitchen back wall toward the patio / window wall.
* The origin is the interior corner of the bedroom's left wall and the back wall.
* Clear sizes are wall face to wall face. `1 in = 0.0254 m`.

ButlerBot stands in the main room at about `(6.10, 4.70, 0)`, facing +Y toward the patio wall.

## Measured (Dan's tape, 2026-10-25)

Letters match `plan_measure.png`. The test checks A–F against the world within 1 cm, and checks that the G–J openings are at least these widths.

| letter | what | inches | metres |
|---|---|---:|---:|
| A | main living room width, wall to wall | 184 | 4.6736 |
| B | living plus kitchen length, patio wall to back wall | 328 | 8.3312 |
| C | bedroom width | 144 | 3.6576 |
| D | bedroom length | 146 | 3.7084 |
| E | entry hall width, left-block wall to the kitchen cabinets (the fridge). Open floor from the front door up to door G, open to the kitchen on the east | 44 | 1.1176 |
| F | kitchen aisle, island to counter | 38 | 0.9652 |
| G | bedroom doorway at the top of the entry hall (open; turn west into the bedroom; the leaf swings toward the kitchen) | 32 | 0.8128 |
| H | bathroom doorway from the west hallway into the bath (open; no leaf in the opening) | 32 | 0.8128 |
| I | front door (closed), at the bottom of the entry hall | 36 | 0.9144 |
| J | patio door opening (closed stop), shared-wall end of the patio wall | 32 | 0.8128 |
| K | no dining table | — | — |

There is no rug.

Interior floor area of the modeled rooms is about 731 sq ft, inside the stated 730–750 sq ft. That sum uses the estimated partitions below, so it is not itself a tape measurement.

## Estimates

These were read off the plan's proportions or assumed from the photos. They are marked `estimated` in the JSON. Wall thickness is 4 in, so the clear tape lengths above do not include the partitions.

| item | estimate |
|---|---|
| interior wall thickness | 4 in (0.102 m) |
| wall height | 96 in / 8 ft (2.438 m) |
| left block | the bedroom width (C). West column is the bath, full height down to the back wall, 72 in wide, with a closed linen nook in the top-left corner. Middle column, 68 in, holds the walk-in, two closets, the west hallway, and the laundry. East of that column is the entry hall (measured E), open floor, open to the kitchen |
| bathroom | 72 in wide × 178 in deep, the full-height west column. The bedroom walkway opens into its north end, just right of the linen nook. Door H is the open passage in its east wall, from the west hallway |
| linen nook | about 20 × 28 in, closed, top-left corner of the bath under the bedroom wall |
| walk-in closet | 68 in wide × 42 in deep, top of the middle column, under the bedroom wall. Open 32 in door on its bath side |
| pantry and storage | side by side under the walk-in, each 32 in wide × 38 in deep. Closed. The south wall is a solid door line, with no leaf in the hallway |
| west hallway | 68 in wide × 40 in deep, between the closets and the laundry. Opens east into the entry hall and west through door H into the bath. This hallway is not letter E |
| entry hall | measured E, 44 in, from the left-block wall to the fridge. Open floor from the front door north to door G, and open to the kitchen and living room on the east. No wall on that east side |
| laundry | 68 in wide × 46 in deep, bottom of the middle column, beside the lower bath. Open 36 in doorway off the west hallway |
| bedroom-to-bath walkway | 32 in, open, no door leaf, at the left end of the bedroom's inner wall (about 32 to 64 in from the outer wall), just right of the linen nook. Directly above the bath. Not letter H |
| back counter run | fridge, then stove, then cabinets, from the entry hall's east edge to the far exterior wall |
| closet, pantry, storage, and laundry floors | laminate, assumed. The tape names laminate for the main room, kitchen, bathroom, and hallway, and carpet for the bedroom |
| counter depth | 25 in. The fridge is 30 in deep, at the entry end of the run |
| fridge | about 30 in wide by 30 in deep by 70 in tall. Stainless |
| stove | 30 in wide, same depth as the counter |
| island | 132 × 36 in, 36 in high, from the entry hall's east edge to 8 in off the far wall, one measured aisle (F) north of the counter. Granite top, dark cabinets, sink. One stool at the right end |
| bath fixtures | along the outer wall, top to bottom: vanity 20 × 28 in with a sink, toilet 16 × 30 in, tub 60 × 32 in along the back wall. White. The walk from the bedroom to door H stays clear of them |
| carpet lip | 0.5 in (12.7 mm) step where the bedroom carpet meets laminate, at door G and at the bath walkway. Dan described it as slightly raised and roughly flush; the 0.5 in figure is the modeled estimate |

The front door opens into the entry hall. From the top of that hall, door G is an open 32 in passage west into the bedroom. The real leaf swings toward the kitchen and is not a solid in the opening. A hallway runs west from the entry hall, between the two closed closets and the laundry, through door H into the bath. The laundry door opens off that hallway. From the bath, the bedroom walkway leads north into the bedroom. The walk-in opens off the bath. The two small closets and the linen nook are closed. The front door and the patio stop are closed.

## Furniture

Every size and position is an estimate from Dan's photos and his marked-up plan. There is no dining table and no rug. The axis-aligned boxes below are the scoring geometry (`X, Y, Z` inches). Metre coordinates are in `eval/apartment_ground_truth.json`.

Main-room `u` runs from the shared wall to the far exterior wall. `v` runs from the patio wall to the kitchen back wall. Bedroom `x` runs from the outer side wall to the shared wall. Bedroom `y` runs from the patio-side wall to the inner wall.

| piece | size (in) | where |
|---|---|---|
| loveseat | 38 × 72 × 40 | dark brown leather, flush to the shared wall near the patio door, clear of door G |
| end table | 18 × 18 × 24 | loveseat's patio-end inner corner |
| 3-seat sofa | 36 × 80 × 40 | dark brown leather, flush to the far exterior wall near the patio |
| end table | 18 × 18 × 24 | 3-seat sofa's kitchen-side end |
| desk | 24 × 42 × 29 | compact, flush to the far exterior wall below the sofa |
| office chair | 8 × 16 × 40 | black mesh chair tucked to the desk, 8 in past the desk face |
| trash can | 11 × 11 × 15 | against the far wall, south of the desk, inside the desk's depth |
| monitor stand | 12 × 12 × 42 | in the gap between the desk and the sofa, flush to the far wall |
| fridge | 30 × 30 × 70 | stainless, at the entry hall's east edge, start of the back counter |
| stove | 30 × 25 × 36 | in the counter run, after the fridge |
| cabinets | 80 × 25 × 36 | from the stove to the far exterior wall. Uppers are visual only and stay inside the base footprint |
| island | 132 × 36 × 36 | from the entry hall's east edge to 8 in off the far wall, 38 in (F) north of the counter |
| island stool | 18 × 18 × 36 | one wooden stool at the island's right end, on the living-room side, out of the aisle |
| bed | 80 × 60 × 48 | queen, outer/patio corner, long side along the patio wall. 48 in is the headboard height |
| dresser | 19 × 25 × 28 | patio-side corner by the shared wall |
| coat stand | 14 × 23 × 65 | against the outer side wall, below the bed, clear of the bath walkway |
| tub | 60 × 32 × 20 | white, along the back wall at the bath's outer corner |
| toilet | 16 × 30 × 30 | white, on the outer wall between the tub and the vanity |
| vanity | 20 × 28 × 32 | white, with a sink, on the outer wall under the linen nook |

There are no pendant lights.

A 24 in (0.610 m) clear path must remain from the front door through the entry hall into the living room, from the entry hall west through door G into the bedroom, from the entry hall through the west hallway into the bath, from the bath north through the walkway into the bedroom, from the hallway into the laundry, and through the walk-in and the kitchen aisle. The two small closets and the linen nook stay closed, so the path does not enter them. The test inflates every non-floor solid by half of that width and checks those rooms still connect. G, H, F, the entry opening, and the walkway are all wider than 24 in.

## Floors

Each zone is its own floor solid with a `contactMaterial`.

| zone | material | top |
|---|---|---|
| living, kitchen, hallway, bathroom | `laminate` | z = 0 |
| closet, pantry, storage, laundry | `laminate` (assumed) | z = 0 |
| bedroom, and the lip at door G and the bath walkway | `carpet` | z = 0.5 in |

Walls are light greige, doors and trim white, laminate a grey-brown wood, carpet a medium pile.

Webots R2025a `ContactProperties` has no rolling-resistance coefficient. `forceDependentSlip` (ODE `v = k·F`) is the stand-in. Laminate uses the same pair as the warehouse floor so the robot still drives. Carpet is higher.

| pair | coulombFriction | forceDependentSlip | documented Crr |
|---|---:|---:|---:|
| `drive_wheel` / `laminate` | 8 | 0.01 | 0.012 |
| `drive_wheel` / `carpet` | 8 | 0.04 | 0.050 (medium pile, between a rental carpet and shag) |
| `caster` / `laminate` | 0.08 | 1 | — |
| `caster` / `carpet` | 0.15 | 0.6 | — |

**Not wired into the energy model.** `src/twin_energy_cruise.py` still uses the warehouse `drive_wheel` / `default` pair and `DEFAULT_CRR` 0.015. The Crr numbers above are notes for a later hookup.

## Top-down sketch

+Y is up (patio). +X is to the right. Not to scale. Door letters are the tape; the service rooms are estimates.

```
patio wall                    J closed (shared-wall end)
+------------------------------+---------------------------------------+
| BEDROOM  carpet              | LIVING  laminate                      |
| bed               dresser    | loveseat                  sofa        |
| coat                         |                           desk flush  |
| walkway ^                 G >|                                       |
+----+-------------+-----------+                                       |
|nook| WALK-IN                 |                                       |
|van | (opens to the bath)     | entry hall (open to the kitchen, E)   |
|toi +-------------+-----------+                                       |
|    | closet | closet (closed)|                                       |
| H >| west hallway -----------+-->                                    |
|tub +-------------+-----------+ island ========================= stool|
|    | LAUNDRY (door off hall) | aisle 38 in (F)                       |
+----+-------------------------+ fridge | stove | cabinets to far wall-+
 back wall                          I closed, bottom of the entry hall
```

The entry hall is the open 44 in strip east of the laundry, closets, and walk-in. Its east side has no wall. Door G is the opening at the top of that hall. The lower opening in the same wall is the west hallway.

## Mapping run

`apartment.wbt` starts `apartment_mapper` on ButlerBot and `apartment_pose_logger` on a supervisor node under the floor. The mapper builds the grid from `oak_depth`, `oak_gyro`, `oak_imu` roll/pitch, and the wheel encoders. The logger is the only controller that calls the supervisor. How to launch and score, including the Windows command, is in the README section **Mapping the apartment**.

```powershell
powershell -ExecutionPolicy Bypass -File scripts\launch_webots_twin.ps1 -World apartment
python eval\score_apartment_map.py (Get-Content logs\mapping_runs\CURRENT).Trim()
```

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
| E | entry hall width, wall to the kitchen cabinets | 44 | 1.1176 |
| F | kitchen aisle, island to counter | 38 | 0.9652 |
| G | bedroom doorway to the main room (open; the leaf swings toward the kitchen) | 32 | 0.8128 |
| H | bathroom door from the hallway (closed door) | 32 | 0.8128 |
| I | front door (closed), in the main-room back wall at the entry | 36 | 0.9144 |
| J | patio door opening (closed stop), shared-wall end of the patio wall | 32 | 0.8128 |
| K | no dining table | — | — |

There is no rug.

Interior floor area of the modeled rooms is about 733 sq ft, inside the stated 730–750 sq ft. That sum uses the estimated partitions below, so it is not itself a tape measurement.

## Estimates

These were read off the plan's proportions or assumed from the photos. They are marked `estimated` in the JSON. Wall thickness is 4 in, so the clear tape lengths above do not include the partitions.

| item | estimate |
|---|---|
| interior wall thickness | 4 in (0.102 m) |
| wall height | 96 in / 8 ft (2.438 m) |
| bathroom | 96 in wide × 72 in deep, under the bedroom's left side. The bedroom walkway opens into it. Door H is the closed door on the hall |
| walk-in closet | 96 in wide × 48 in deep, under the bathroom, same place as before. Closed toward the hall |
| laundry | 96 in wide × 50 in deep, under the closet. The wall on the hall is gone; bifolds are a flush closed panel |
| hallway length | 178 in. The **width** is measured (E, 44 in). Laminate |
| hall-to-kitchen opening | 56 in along the hall, a cased opening so the entry joins the hall. Not a letter on the tape |
| bedroom-to-bath walkway | 32 in, open, no door leaf. Centered on the bedroom's inner wall (about x 0.4 to 0.6). Leads into the bath. Not letter H |
| pantry | 24 in deep into the main room, 30 in door on the hall, closed. Along the hall, past the entry |
| storage closet | same depth and door as the pantry, the next closed closet along the hall |
| laundry bifolds | the hall wall is removed. White bifolds are a flush closed panel in that walkway |
| back counter run | from about u = 0.28 (fridge at the entry end, then the stove) to u = 1 at the far exterior wall |
| closet and laundry floors | laminate, assumed. The tape names laminate for the main room, kitchen, bathroom, and hallway, and carpet for the bedroom |
| counter depth | 25 in |
| fridge | about 30 in wide (Dan) by 30 in deep by 70 in tall. Stainless. Entry end of the counter run, about u = 0.28 |
| stove | 30 in wide, same depth as the counter |
| island | 60 × 36 in, 36 in high, granite top, dark cabinets, sink |
| carpet lip | 0.5 in (12.7 mm) step where the bedroom carpet meets laminate, at the bedroom door and at the bath walkway. Dan described it as slightly raised and roughly flush; the 0.5 in figure is the modeled estimate |

The bedroom door is an open 32 in passage. The real leaf swings toward the kitchen and is not a solid in the opening, so the clear width stays the measured 32 in. A second opening in that same inner wall, near the outer side wall, is the bedroom-to-bath walkway: about 32 in, no door leaf. Door H is the separate closed door where the hallway meets the bathroom. The front door, the patio stop, the pantry door, and the storage door are closed. The laundry bifolds are a flush closed panel, not a thick wall.

## Furniture

Every size and position is an estimate from Dan's photos and his marked-up plan. There is no dining table and no rug. The axis-aligned boxes below are the scoring geometry (`X, Y, Z` inches). Metre coordinates are in `eval/apartment_ground_truth.json`.

Main-room `u` runs from the shared wall to the far exterior wall. `v` runs from the patio wall to the kitchen back wall. Bedroom `x` runs from the outer side wall to the shared wall. Bedroom `y` runs from the patio-side wall to the inner wall.

| piece | size (in) | where |
|---|---|---|
| loveseat | 38 × 72 × 40 | dark brown leather, shared wall near the patio door, about u 0.02–0.23, v 0.05–0.27, facing into the room |
| end table | 18 × 18 × 24 | loveseat's patio-end inner corner |
| 3-seat sofa | 36 × 80 × 40 | dark brown leather, far exterior wall near the patio, about u 0.80–1, v 0.05–0.29, facing into the room |
| end table | 18 × 18 × 24 | 3-seat sofa's kitchen-side end |
| desk | 30 × 60 × 29 | far exterior wall below the sofa, about 60×30 in, inside u 0.84–1, v 0.34–0.56 |
| office chair | 22 × 22 × 40 | black mesh chair on casters, in front of the desk |
| trash can | 11 × 11 × 15 | small black can at the desk's kitchen end |
| monitor stand | 24 × 12 × 42 | between the desk and the sofa, against the far wall |
| fridge | 30 × 30 × 70 | stainless, light grey metallic, entry end of the back counter (about u = 0.28) |
| stove | 30 × 25 × 36 | in the counter run, after the fridge |
| cabinets | 72 × 25 × 36 | from the stove to the far exterior wall. Uppers are visual only and stay inside the base footprint |
| island | 60 × 36 × 36 | granite top, dark cabinets, sink |
| island chair | 18 × 18 × 36 | one wooden chair on the living-room side |
| bed | 80 × 60 × 48 | queen, outer/patio corner, long side along the patio wall, headboard against that wall. About x 0–0.55, y 0.02–0.5. 48 in is the headboard height |
| dresser | 19 × 25 × 28 | patio-side corner by the shared wall, about x 0.85–0.98, y 0.03–0.2 |
| coat stand | 14 × 23 × 65 | against the outer side wall, about x 0–0.1, y 0.52–0.68 |

There are no pendant lights.

A 24 in (0.610 m) clear path must remain from the main room through the bedroom door, the hallway, the kitchen aisle, and the bedroom-to-bath walkway. The test inflates every non-floor solid by half of that width and checks those places still connect. G, E, F, and the walkway are all wider than 24 in.

## Floors

Each zone is its own floor solid with a `contactMaterial`.

| zone | material | top |
|---|---|---|
| living, kitchen, hallway, bathroom | `laminate` | z = 0 |
| closet, laundry | `laminate` (assumed) | z = 0 |
| bedroom, and the lip at the bedroom door and the bath walkway | `carpet` | z = 0.5 in |

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
patio wall          J closed (shared-wall end)
+---------------------------+------------------------------------------+
| BEDROOM  carpet +0.5 in   | LIVING  laminate                         |
| bed (head on patio wall)  | loveseat          3-seat sofa            |
| coat stand    dresser     | end tables                               |
| walkway (open)            |           desk chair  monitor            |
+-------------+-------------+ G bedroom door (open, toward kitchen)    |
| BATH        | H closed    |                                          |
| estimate    | HALL 44 in  |                                          |
+-------------+             | pantry + storage (closed doors on hall)  |
| WALK-IN     | laminate    |                                          |
| stays       |             | KITCHEN  laminate                        |
+-------------+ bifold panel| island + one chair                       |
| LAUNDRY     | (walkway,   | aisle 38 in                              |
| estimate    |  closed)    | fridge | stove | cabinets to far wall    |
+-------------+-------------+------+ I closed                          |
 back wall (solid)                 front door in the main room
```

Pantry and storage sit on the main-room side of the hall with closed doors. The laundry side of that hall is an opening with the bifolds drawn as a flush closed panel. The walk-in closet is unchanged, between the bath and the laundry.

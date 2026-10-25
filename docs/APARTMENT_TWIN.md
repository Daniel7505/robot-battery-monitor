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
| G | bedroom doorway (open; the leaf swings toward the kitchen) | 32 | 0.8128 |
| H | bathroom doorway (closed door) | 32 | 0.8128 |
| I | front door (closed) | 36 | 0.9144 |
| J | patio door opening (closed stop; the patio is out of scope) | 32 | 0.8128 |
| K | no dining table | — | — |

There is no rug.

Interior floor area of the modeled rooms is about 733 sq ft, inside the stated 730–750 sq ft. That sum uses the estimated partitions below, so it is not itself a tape measurement.

## Estimates

These were read off the plan's proportions or assumed from the photos. They are marked `estimated` in the JSON. Wall thickness is 4 in, so the clear tape lengths above do not include the partitions.

| item | estimate |
|---|---|
| interior wall thickness | 4 in (0.102 m) |
| wall height | 96 in / 8 ft (2.438 m) |
| bathroom | 96 in wide × 72 in deep, under the bedroom's left side. Door H is in that shared wall |
| walk-in closet | 96 in wide × 48 in deep, under the bathroom. Double bifold toward the hall, modeled as a closed wall |
| laundry | 96 in wide × 50 in deep, under the closet. Double bifold toward the hall, modeled as a closed wall |
| hallway length | 178 in. The **width** is measured (E, 44 in). Laminate |
| hall-to-kitchen opening | 66 in along the hall, a cased opening so the aisle joins the hall. Not a letter on the tape |
| closet and laundry floors | laminate, assumed. The tape names laminate for the main room, kitchen, bathroom, and hallway, and carpet for the bedroom |
| counter depth | 25 in |
| fridge | about 30 in wide (Dan) by 30 in deep by 70 in tall. First on the back wall from the entry side |
| stove | 30 in wide, same depth as the counter |
| island | 60 × 36 in, 36 in high, granite top, dark cabinets, sink |
| carpet lip | 0.5 in (12.7 mm) step where the bedroom carpet meets the hall laminate. Dan described it as slightly raised and roughly flush; the 0.5 in figure is the modeled estimate |

The bedroom door is an open 32 in passage. The real leaf swings toward the kitchen and is not a solid in the opening, so the clear width stays the measured 32 in. The bathroom door, the front door, the patio stop, and the hall bifolds are closed walls.

## Furniture

Every size and position is an estimate from Dan's photos. There is no dining table and no rug. The axis-aligned boxes below are the scoring geometry (`X, Y, Z` inches). Metre coordinates are in `eval/apartment_ground_truth.json`.

| piece | size (in) | where |
|---|---|---|
| loveseat | 38 × 60 × 40 | dark brown leather recliner along the living room's left wall |
| end table | 22 × 22 × 24 | between the loveseat and the 3-seat sofa |
| 3-seat sofa | 84 × 38 × 40 | dark brown leather recliner on the far wall, under the window |
| side table | 18 × 18 × 24 | right end of the 3-seat sofa |
| desk | 30 × 60 × 29 | about 60×30 in, right side by the patio door |
| office chair | 22 × 22 × 40 | black mesh chair on casters, at the desk |
| trash can | 11 × 11 × 15 | small black can beside the desk |
| TV / monitor stand | 12 × 24 × 42 | beside the desk (stand plus screen) |
| fridge | 30 × 30 × 70 | stainless, back wall, entry side |
| stove | 30 × 25 × 36 | between the fridge and the cabinets |
| cabinets | rest of the back wall × 25 × 36 | dark bases, granite top. Uppers are visual only and stay inside the base footprint |
| island | 60 × 36 × 36 | granite top, dark cabinets, sink |
| island chair | 18 × 18 × 36 | one wooden spindle-back chair on the living-room side |
| bed | 60 × 80 × 48 | queen assumed, on the bedroom's far wall. 48 in is the headboard height |
| nightstand | 24 × 16 × 28 | dark wood, near the bed |
| coat stand | 16 × 16 × 65 | left corner of the bedroom |

Three pendant lights hang over the island. They are visual only (no collision).

A 24 in (0.610 m) clear path must remain from the main room through the bedroom door, the hallway, and the kitchen aisle. The test inflates every non-floor solid by half of that width and checks those four places still connect. G, E, and F are all wider than 24 in (32, 44, and 38).

## Floors

Each zone is its own floor solid with a `contactMaterial`.

| zone | material | top |
|---|---|---|
| living, kitchen, hallway, bathroom | `laminate` | z = 0 |
| closet, laundry | `laminate` (assumed) | z = 0 |
| bedroom, and the lip in the bedroom doorway | `carpet` | z = 0.5 in |

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
patio / window wall                                          J closed
+---------------------------+------------------------------------------+
| BEDROOM   carpet +0.5 in  | LIVING   laminate                        |
| bed on the far wall       | loveseat  end   3-seat sofa   side       |
| nightstand     coat stand |                      desk  chair  TV     |
| H bath door (closed)      |                                          |
+-------------+-------------+ G bedroom door (open, toward kitchen)    |
| BATH        | HALL 44 in  |                                          |
| estimate    | laminate    |                                          |
+-------------+ bifolds     |                                          |
| CLOSET      | closed      | KITCHEN  laminate                        |
| estimate    |             | island + chair on the living side        |
+-------------+             | aisle 38 in                              |
| LAUNDRY     |             | fridge | stove | cabinets                |
| estimate    | I closed    +------------------------------------------+
+-------------+ front door    back wall
```

The hall's other closed door (toward the kitchen wall, past the cased opening) is drawn as a white panel on a solid wall. It is not a measured doorway.

# Engineering dashboard (archived)

The old full dashboard is saved on branch `archive/engineering-dashboard` (commit `1ec638f`). That is the last `main` before the page was cut down to battery, the lap, and proof.

## What the engineering view contained

A second view on the same page, built to look like a lab power console:

- LRU health (parts grouped into units, and whether each was inside its limit)
- Per-channel amps and thermal readings
- Forecasts (how long the battery should last, and a scripted mission timeline)
- Safety badges
- History analytics from the database
- ROS2 status
- Power allocation across channels

## Why it was retired

Most of those values were estimated or scripted. They were not measured. On a simulated lap they looked precise, and were often blank or priced from a canned mission rather than the lap that was actually running. The slim page is the default.

## When it comes back

The engineering view is meant to return when a real robot arrives. Rebuild it on real sensor data: current, voltage, and temperature sensors, and a real ROS2 connection. Keep the slim view as the default.

## How to view it

```
git checkout archive/engineering-dashboard
```

That branch is only the old page. Switch back to your working branch when you are done.

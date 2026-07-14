# Dynamic right-side yielding experiment

Build: `dynamic-yield-exp0.1`

Mod version: `3.0.1.2.50`

This branch starts directly from `main`. It does not contain the previous single-lane block or orange-pocket implementation.

## What it does

When two active AutoDrive vehicles:

- are 55–175 m apart;
- are travelling in opposite directions;
- are already on the same future blue two-way route in reverse order;
- have at least 120 m of that common route ahead;
- are not already involved in another dynamic meeting;

both right-side manoeuvres are evaluated.

The system considers:

- measured vehicle/implement width;
- tractor-train length;
- AutoDrive turning radius;
- forward road grade;
- terrain height along the temporary path;
- cross slope to the right;
- static collision checks along the right-side corridor.

The vehicle with the safer/lower-cost manoeuvre yields. A vehicle travelling uphill receives a large priority to continue; a vehicle travelling downhill is preferred for yielding when its right-side corridor is safe.

The yielding vehicle receives temporary pathfinder points forming a smooth S-shaped right move. It slows to 20 km/h and then 10 km/h, stops at maximum offset, waits for the opposing vehicle to fully pass, and rejoins the original AutoDrive route.

No permanent AutoDrive waypoints are created or saved.

## Safety status

This is an experimental build. It is disabled by default.

The first test must be performed on:

- a straight or gently curved blue two-way route at least 180–250 m long;
- level open ground;
- no ditch, water, fence, tree, pole or building on either right side;
- two short vehicles without trailers;
- AutoDrive traffic detection enabled.

Do not begin with a combine, wide header or multi-trailer road train.

## Installation

Download branch:

`feature/dynamic-yield-experiment`

Create the normal `FS25_AutoDrive.zip`, with `modDesc.xml` at the root of the archive.

The mod list must show version:

`3.0.1.2.50`

## Console commands

Enable the experiment:

`adDynamicYield on`

Show current status:

`adDynamicYield status`

Disable it:

`adDynamicYield off`

Enable rejection diagnostics:

`adDynamicYieldDebug on`

Disable verbose diagnostics:

`adDynamicYieldDebug off`

## First test

1. Place two small vehicles about 220–250 m apart on opposite ends of one long blue two-way route.
2. Give them destinations beyond each other.
3. Start both vehicles.
4. Enable the experiment before they come within 175 m:

   `adDynamicYield on`

Expected log:

```text
[AD-DY] Pair 1: <vehicle> yields right, <vehicle> continues; shared=... offset=... grade=... score=...
[AD-DY] Pair 1 holding <vehicle> ...m right of route
[AD-DY] Pair 1 releasing <vehicle> after <vehicle> passed
[AD-DY] Pair 1 cleared: yield vehicle returned to route
```

Expected movement:

1. One vehicle continues on the original blue route.
2. The other gradually moves to its right.
3. It slows and stops clear of the route.
4. The opposing vehicle passes.
5. The yielding vehicle returns smoothly and continues to its original destination.

## If no manoeuvre occurs

Enable:

`adDynamicYieldDebug on`

Common rejection reasons:

- `insufficient future route` — not enough route remains for the S manoeuvre;
- `cross grade` — terrain to the right is too steep;
- `longitudinal grade` — the temporary path has a steep step or slope;
- `obstacle in right corridor` — collision scan found an object;
- no pair message — vehicles are not yet on the same sufficiently long opposite blue section, or their headings are not opposite enough.

## Current limitations

- The first build does not model actual loaded mass or engine traction.
- It does not identify water separately from ordinary terrain.
- The temporary corridor uses conservative collision boxes and can reject usable space.
- The manoeuvre is calculated from the current route and is not yet replanned if another obstacle appears later.
- Multiplayer authority is server-side, but this version has not yet been multiplayer-tested.

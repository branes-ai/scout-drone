# Repository Layout

```text
scout-drone/
├── apps/         # the harnesses: sitl, replay and sitl_batch per baseline, and score_log, log_dump and trace_export
├── baselines/    # one file per baseline: its slot composition and its parameters
├── cmake/        # deps (anatomy), warnings, test registration, the slot registry and the baseline generator
├── docs/         # the documents: the source of this site
├── docs-site/    # the Starlight site generated from docs/, published to GitHub Pages
├── fields/       # the worlds: a start, a goal, a geofence, and obstacles listed or generated
├── scenarios/    # the runs: a field, a vehicle, a duration, a seed, the randomization and the oracle
├── sdk/          # branes::scout: the mission's world, the obstacles seen, slots, stack, configs and channels
├── sim/          # branes::scout_sim: the simulator (the obstacle oracle), the scorer, and the scored run
├── tools/        # gen_replay.sh: records the docs site's forest replays
├── tests/        # Catch2 tests, one executable per tests/<layer>/<name>.cpp; fixtures/ holds their data
└── vehicles/     # the vehicles, for now racing-drone's RP-1
```

The mission's headers are under `<branes/scout/...>`, in namespace `branes::scout`. It uses anatomy's names directly, under `<branes/anatomy/...>`.

A field file (`fields/<name>.yaml`) lists its obstacles, generates them from the run's seed, or both: `forest-60` grows a different forest for each seed.

A baseline file (`baselines/<name>.yaml`) names one implementation for each of scout's six slots, and the parameters they read. At configure time, `cmake/baselines.cmake` checks the composition against the slot registry (`cmake/slot_registry.cmake`) and generates the header that defines the baseline's stack. There are two baselines so far: `null`, a stub in every slot, and `s0-gt-pid`, which flies the true state in a straight line to the goal.

`sim/` runs a baseline in anatomy's closed loop. Its oracle reports the obstacles within range of the true position at each camera frame, and its scorer judges a run: reached the goal, collided, left the geofence, over the speed limit, or timed out.

`apps/` flies a baseline over a scenario (`scenarios/<name>.yaml`) and writes the log, replays it, scores it, dumps it as CSV, or runs a batch of seeds ([build and test](build-and-test.md)).

The [documentation map](../README.md) lists every document.

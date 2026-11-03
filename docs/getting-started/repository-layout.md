# Repository Layout

```text
scout-drone/
├── apps/         # the harnesses: sitl, replay and sitl_batch per baseline and build, and score_log, log_dump and trace_export
├── baselines/    # one file per baseline: its slot composition, its graph's bodies and its parameters
├── cmake/        # deps (anatomy), warnings, test registration, the slot and operator registries and the baseline generator
├── docs/         # the documents: the source of this site
├── docs-site/    # the Starlight site generated from docs/, published to GitHub Pages
├── fields/       # the worlds: a start, a goal, a geofence, and obstacles listed or generated
├── scenarios/    # the runs: a field, a vehicle, a duration, a seed, the randomization and the oracle
├── sdk/          # branes::scout: the mission's world, the obstacles seen, slots, stack and graph, configs and channels
├── sim/          # branes::scout_sim: the simulator (the obstacle oracle), the scorer, and the scored run
├── tools/        # gen_replay.sh: records the docs site's forest replays
├── tests/        # Catch2 tests, one executable per tests/<layer>/<name>.cpp; fixtures/ holds their data
└── vehicles/     # the vehicles, for now racing-drone's RP-1
```

The mission's headers are under `<branes/scout/...>`, in namespace `branes::scout`. It uses anatomy's names directly, under `<branes/anatomy/...>`.

A field file (`fields/<name>.yaml`) lists its obstacles, generates them from the run's seed, or both: `forest-60` grows a different forest for each seed.

A baseline file (`baselines/<name>.yaml`) names one implementation for each of scout's six slots, and the parameters they read. At configure time, `cmake/baselines.cmake` checks the composition against the slot registry (`cmake/slot_registry.cmake`) and generates the header that defines the baseline's stack. There are two baselines so far: `null`, a stub in every slot, and `s0-gt-pid`, which flies the true state in a straight line to the goal.

A baseline also has a graph build ([anatomy ADR 0004](https://github.com/branes-ai/anatomy/blob/main/docs/adr/0004-domain-flow-composition.md), step 2). The file's `class: scoreboard` and `graph:` name the body of each of the graph's six composites, from the operator registry (`cmake/flow_registry.cmake`). The graph is C++, in `<branes/scout/flow/...>`: the domains of scout's messages, the estimator's and planner's bindings, and the graph and its CPU stack (`graph::Stack`). Its harnesses are `sitl_<name>_graph`, `replay_<name>_graph` and `sitl_batch_<name>_graph`. A gate checks that each baseline's two builds write the same runs, on every scenario and in the batch, with anatomy's `log_compare`. Their logs differ only in `composition_hash`, which is the slot composition's hash in one and the graph's in the other.

`sim/` runs a baseline in anatomy's closed loop. Its oracle reports the obstacles within range of the true position at each camera frame, and its scorer judges a run: reached the goal, collided, left the geofence, over the speed limit, or timed out.

`apps/` flies a baseline over a scenario (`scenarios/<name>.yaml`) and writes the log, replays it, scores it, dumps it as CSV, or runs a batch of seeds ([build and test](build-and-test.md)).

The [documentation map](../README.md) lists every document.

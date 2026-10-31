# Repository Layout

```text
scout-drone/
├── cmake/        # deps (anatomy), warnings, test registration (compile_all)
├── docs/         # the documents: the source of this site
├── docs-site/    # the Starlight site generated from docs/, published to GitHub Pages
├── fields/       # the worlds: a start, a goal, a geofence, and obstacles listed or generated
├── sdk/          # branes::scout: the mission's world, slots, stack, configs and channels
├── tests/        # Catch2 tests, one executable per tests/<layer>/<name>.cpp; fixtures/ holds their data
└── vehicles/     # the vehicles, for now racing-drone's RP-1
```

The mission's headers are under `<branes/scout/...>`, in namespace `branes::scout`. It uses anatomy's names directly, under `<branes/anatomy/...>`.

A field file (`fields/<name>.yaml`) lists its obstacles, generates them from the run's seed, or both: `forest-60` grows a different forest for each seed. The [bring-up plan](../plan/bring-up/README.md) adds the rest as S0 lands: the baselines, the scenarios, the simulator and scorer, and the apps.

The [documentation map](../README.md) lists every document.

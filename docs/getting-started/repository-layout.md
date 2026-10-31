# Repository Layout

```text
scout-drone/
├── cmake/        # deps (anatomy), warnings, test registration (compile_all)
├── docs/         # the documents: the source of this site
├── docs-site/    # the Starlight site generated from docs/, published to GitHub Pages
├── sdk/          # branes::scout: the mission's world, slots, stack, configs and channels
└── tests/        # Catch2 tests, one executable per tests/<layer>/<name>.cpp
```

The mission's headers are under `<branes/scout/...>`, in namespace `branes::scout`. It uses anatomy's names directly, under `<branes/anatomy/...>`.

The [bring-up plan](../plan/bring-up/README.md) adds the rest as S0 lands: the field files, the baselines, the scenarios and vehicles, the simulator and scorer, and the apps.

The [documentation map](../README.md) lists every document.

#!/usr/bin/env bash
# gen_replay.sh [preset]
#
# Records the docs site's forest replays (docs/guides/forest-replay.mdx): one
# SITL run of s0-gt-pid on each of scenarios/s0-forest-collision.yaml and
# scenarios/s0-forest-near-miss.yaml, exported by trace_export as
# reflex-trace/1 to docs-site/src/traces/<scenario>.json, stamped with
# `git describe` of this tree. The preset defaults to gcc-release. Commit the
# traces with the change that produced them:
#
#   tools/gen_replay.sh
#   git add docs-site/src/traces/
#
# Run from the source root.
set -euo pipefail

preset=${1:-gcc-release}
cmake --preset "$preset" >/dev/null
cmake --build --preset "$preset" --target sitl_s0-gt-pid trace_export
commit=$(git describe --always --dirty --abbrev=7 --exclude '*')
work=$(mktemp -d)
trap 'rm -rf "$work"' EXIT
mkdir -p docs-site/src/traces
for name in s0-forest-collision s0-forest-near-miss; do
    scenario=scenarios/$name.yaml
    "build/$preset/sitl_s0-gt-pid" --scenario "$scenario" --out "$work/$name.brlog"
    "build/$preset/apps/trace_export" --log "$work/$name.brlog" --scenario "$scenario" \
        --out "docs-site/src/traces/$name.json" --commit "$commit"
done

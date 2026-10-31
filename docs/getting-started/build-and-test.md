# Build & Test

scout-drone is C++20 and builds with CMake ≥ 4.0 and Ninja, on GCC, Clang, and MSVC, using the same presets as the sibling repos.

## Build and run the tests

```bash
cmake --preset gcc-debug              # also: gcc-release, clang-debug, clang-release, msvc, msvc-ninja, ...
cmake --build --preset gcc-debug
ctest --preset gcc-debug
```

The first configure fetches anatomy, pinned in `cmake/deps.cmake`, and what anatomy builds on: yaml-cpp, reflex and Universal. The tests are built only when scout-drone is the top-level project (`SCOUT_BUILD_TESTS`).

## The docs site

This site is generated from [`docs/`](../README.md) by [docs-kit](https://github.com/branes-ai/docs-kit). To preview it:

```bash
cd docs-site
npm ci
npm run dev        # regenerates the pages from docs/, then serves them
```

Next: [repository layout](repository-layout.md).

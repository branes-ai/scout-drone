# Documentation map

These documents are published at <https://branes-ai.github.io/scout-drone/>. The site is generated from this folder on every merge to main: edit the documents here, never the site. [`index.mdx`](index.mdx) is the site's landing page, and this file is its documentation map.

| Folder | Contents |
|---|---|
| [getting-started/](getting-started/introduction.md) | [Introduction](getting-started/introduction.md), [build & test](getting-started/build-and-test.md), [repository layout](getting-started/repository-layout.md) |
| [guides/](guides/forest-replay.mdx) | [Watching a forest replay](guides/forest-replay.mdx): two SITL runs of s0-gt-pid through the forest, one that collides and one that reaches the goal, played in reflex's trace player, and how to export your own |
| [plan/](plan/bring-up/README.md) | Per-issue plans, each with its issues and PRs: [bringing up scout-drone on anatomy](plan/bring-up/README.md) |
| [adr/](adr/README.md) | Architecture decision records |
| sessions/ | Dated session records (`YYYY-MM-DD-short-topic.md`), not published |

The plans and the decisions are published on the [architecture site](https://branes-ai.github.io/architecture/scout-drone/), alongside every other repo's plans and decisions and their status, rather than here.

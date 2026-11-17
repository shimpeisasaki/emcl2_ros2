# Local maintenance

## Adopted source and ownership

The 2026-11-12 implementation starts at
`cbaa31735b0b09c6e3fc26a4a0964bc44ad7c22d` on local baseline `main`.
The locally available `upstream/main` and merge base are
`3eacea2dc70d9d29eb0d501d01eb5df001814776`; no remote refresh was performed.
That dated implementation used `refactor/implementation-20261112`; it is a historical baseline, not the current branch selection.
Retain the upstream layout, package name `emcl2`, and [COPYING](COPYING).

Three local commits preserve scan freshness/rejection (`c125a45`),
reduced diagnostic logging/removal of the separate quick-start config (`a81f4ec`),
and one sensor update per laser scan (`cbaa317`).
The [README scan-freshness section](README.md#scan-freshness) owns the public behavior.
Fresh localization recovery does not grant driving permission; the vehicle watchdog
retains that responsibility. The divergent GNSS experiment is not a replacement baseline.

## Current configuration and safe checks

[config/emcl2.param.yaml](config/emcl2.param.yaml) is the public configuration,
and [launch/emcl2.launch.py](launch/emcl2.launch.py) defaults to it.
README, [test/demo.bash](test/demo.bash), and [test/test.launch.xml](test/test.launch.xml)
now reference that existing file. Its parameter values are unchanged.
The removed quick-start file is available through Git history, not a current entry point.

For a prose/reference change, inspect the diff, validate relative paths, parse YAML/XML,
and run `bash -n test/demo.bash`. Do not execute the demo as a syntax check.
[test/test.bash](test/test.bash) starts a ROS launch and sends initial poses, service
requests and navigation goals; its wrapper name does not make it a safe offline unit test.
Simulation and graph interaction require separate authorization.

For separately authorized code validation, build the sibling `emcl2_msgs` package from
source before emcl2, and inspect the selected fixtures. The current feature's input,
lifecycle, fixture coverage and evidence are owned by [GNSS_RECOVERY.md](GNSS_RECOVERY.md).
Use explicit source paths, new build/install outputs and no more than two compilation
workers. Record `colcon test-result` separately. The 2026-11-12 documentation unit did
not run build, simulation or localization runtime; later checks have separate dated records.

## Change evidence and data

Update owning interface/configuration documentation in the same change as behavior.
Preserve historical evaluation conditions. Record full tested SHA and dirty diff,
OS/ROS/dependencies, input identity, exact commands, counts, exit status, failures,
skips and unverified conditions. Keep offline, simulation and hardware results distinct.

Inspect paths, sizes, metadata and existing summaries before recordings or large logs.
Read payloads only for a named question, bounded by files/topics/time/sample count.
Exclude generated output/data/cache from broad searches. Keep complete logs in a
generated log directory, summarize bounded causal excerpts, and preserve exit codes.
Follow [AGENTS.md](AGENTS.md); do not update upstream or move main as a documentation check.

## External recovery extension (2026-11-17)

The feature branch selectively implements [GNSS recovery](GNSS_RECOVERY.md) on the current main baseline. The historical divergent branch is not merged; scan freshness/duplicate guards remain. The new sibling `emcl2_msgs` repository must be built from source with emcl2.


## GNSS documentation review (2026-11-17)

[Documentation review](docs/validation/gnss_docs_review_20261117.md) records this prose-only
check. [Implementation validation](docs/validation/gnss_reset_20261117.md) records the
prior source build and offline tests; they were not rerun for the documentation check.

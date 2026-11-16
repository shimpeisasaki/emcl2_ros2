# Local maintenance

## Adopted source and ownership

The 2026-11-12 implementation starts at
`cbaa31735b0b09c6e3fc26a4a0964bc44ad7c22d` on local baseline `main`.
The locally available `upstream/main` and merge base are
`3eacea2dc70d9d29eb0d501d01eb5df001814776`; no remote refresh was performed.
The implementation branch is `refactor/implementation-20261112`.
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

For an authorized future code change, source ROS 2 Humble and build only `emcl2`
from the actual checkout with `colcon build --symlink-install --packages-select emcl2
--parallel-workers 2`. Review the selected CMake tests and fixtures before
`colcon test --packages-select emcl2`; record `colcon test-result` separately.
No build, simulation or localization runtime was exercised for this documentation unit.

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

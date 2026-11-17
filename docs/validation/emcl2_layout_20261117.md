# emcl2 multipackage repository migration — 2026-11-17

## Layout and compatibility

The Git repository now contains sibling ROS packages `emcl2/` and `emcl2_msgs/`.
There is no package.xml at the repository root. Package names, CMake package contents,
msg/srv definitions, executable names, runtime code, configuration, source timestamps,
QoS, topics, services and installed share paths are unchanged. The node uses the same
planar antenna TF correction. Source-relative paths moved; the owning current documents
are the [repository README](../../README.md), [package README](../../emcl2/README.md),
[interface README](../../emcl2_msgs/README.md) and [recovery contract](../../GNSS_RECOVERY.md).

## History and original repositories

Initial full source SHAs, branches, clean status, environment and per-file SHA-256
values are recorded in workspace `log/emcl2_layout_20261117/source_before.json`.
The implementation repository baseline is `edeb031dc1f869a3be6491c0b3158f7ae3fa8908`;
the interface repository baseline is `8a48cbaf69c7e77bbdab24d4b18918a5ea9a9ca2`.
The new working branch is `refactor/emcl2-multipackage`. No main branch or upstream
remote was changed. The package relocation is commit `d39b117`; the full-history
interface subtree import is commit `f908843`. Neither history was squashed or rewritten.
Both original histories are reachable in the combined repository.

The original clean interface repository (including .git and its original branch) is
preserved at workspace `log/emcl2_layout_20261117/emcl2_msgs_repository/`, outside src.
A complete verified backup is `log/emcl2_layout_20261117/emcl2_msgs_before.bundle`.
The generated log directory has COLCON_IGNORE. No interface repository remains at
`src/emcl2_msgs`; keeping it there would cause duplicate package discovery.
The imported package has no nested .git directory. Root COPYING and
`emcl2_msgs/LICENSE` retain their original contents.

Dated validation records under docs/validation, including the imported interface
records, retain their original source paths and results. They describe the layouts
used at those dates; this migration record owns the new source layout.

## Changes beyond relocation

Current README links, package source paths and maintenance rules now describe the
combined repository. The CI build step explicitly selects both packages from the
repository base path with sequential execution and at most two compilation workers.
The standalone CI provisioning skips only the three source-only integration-test
packages (`gnss2map`, `ublox_nav_sat_fix_hp_node`, `ublox_ubx_msgs`) absent from that
checkout; the legacy test.bash job does not invoke the GNSS pipeline fixture. Their
manifest declarations and all runtime dependencies are unchanged. Full workspace
validation below builds these dependencies from their actual source checkouts.
The retry count, retry job and rerun workflow remain unchanged. GitHub Actions and
its legacy simulation job were not run locally. Full integration tests require the
existing workspace gnss2map/ublox_dgnss/cat_robot sources; they were not fetched by this
migration. No dependencies were installed and no external Git/network request was made.
The subtree fetch used the local archived repository only.

## Verification evidence

The bounded static checker records runtime/interface/config byte preservation,
licenses, history reachability, local Markdown links/anchors, current package discovery,
CI retry preservation and diff/syntax checks. Full command, count, exit-code and diff
records live under workspace `log/emcl2_layout_20261117/`. They are generated evidence,
not recordings or deployable source. Initial whitespace-check failures are retained in
`initial_layout_check.txt`: relocation exposed trailing spaces in the old README;
only that documentation whitespace was normalized before committing the relocation.

Fresh build/test commands are saved as `build.sh` and `test.sh` in that generated
log directory. They are invoked from the workspace root using an environment with
only HOME, PATH and LANG, then `/opt/ros/humble/setup.bash`. The verification outputs
are new `build/emcl2_layout_20261117` and `install/emcl2_layout_20261117` directories;
old workspace install prefixes do not provide emcl2 or its interfaces. Build/test
executor is sequential; CMAKE_BUILD_PARALLEL_LEVEL=2 and CTEST_PARALLEL_LEVEL=1.
Test fixtures were inspected before checks. ROS tests use localhost domains 214/216/217
and synthetic map, TF, scan, GNSS/UBX data, never receiver drivers or motion commands.

Final results:

- Fresh build: 11 packages succeeded, `build.exit=0` (see build.log).
- Focused offline suites: 83 individual cases passed, no failures/errors/skips,
  `test.exit=0`. Counts: GNSS epoch/status 46; map conversion 4 C++ + 6 ROS;
  recovery 4 C++ + 7 ROS; launch wiring 3; navigation guard 7; waypoint route 6.
- `test_result.log` reports 91 including CTest wrappers, 0 failures/errors/skips,
  `test_result.exit=0`. Individual counts are in test_counts.json.
- Static layout/content/history/link checks: 112 passed, no failures/skips,
  `static_check.exit=0`. Exact predicates and per-file hashes are in static_checks.json.
- Both repository-only and workspace discovery find exactly one emcl2 package and
  one emcl2_msgs package at their new paths (discovery_repo/workspace.log and .exit=0).
- Install probe: 6 critical package prefixes resolve to the new install root;
  3 installed launch/config/test files match source and 4 generated interface types
  import successfully; no old workspace install prefix is present (install_probe.json,
  install_probe.exit=0).
- CI YAML/retry preservation and bash syntax were checked; CI jobs themselves were
  not executed (ci_shell_checks.json). Demo checking used bash -n only.

Only README/reference/CI/layout files changed beyond the file moves. The actual
localization, reset, conversion, driver and message/service definitions are byte-identical
to the recorded pre-migration source. Root COPYING and interface LICENSE are identical.
Historical validation records were imported/preserved unchanged, not relabeled as this
migration's results. Complete current repository/dirty diffs and final Git identities
are in emcl2_ros2_tested.patch, migration_source.json and source_after.json. No hardware,
NTRIP/CIT connection, bag replay, simulation or physical motion is authorized or
performed by this migration. RTK accuracy and real kidnapping/recovery remain unverified.

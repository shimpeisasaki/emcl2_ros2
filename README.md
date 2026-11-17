# emcl2_ros2

ROS 2 Humble localization and recovery interfaces, maintained in one Git repository.
The two ROS packages remain separate so clients can depend on interface types alone.

```text
emcl2_ros2/
├── emcl2/          # localization node, configuration, launch and tests
├── emcl2_msgs/     # health/reset msg and srv definitions
├── GNSS_RECOVERY.md
└── docs/validation/
```

| Package / document | Owns |
| --- | --- |
| [emcl2](emcl2/README.md) | MCL, scan freshness, installed launch/config and node usage |
| [emcl2_msgs](emcl2_msgs/README.md) | Localization status and requested-reset interface definitions |
| [GNSS recovery](GNSS_RECOVERY.md) | Runtime topics/services, acceptance, lifecycle, timestamps and planar antenna correction |
| [Local maintenance](LOCAL_MAINTENANCE.md) | Source ownership, build/test boundaries and historical baselines |

Public package names, executable names, topics, services and installed share paths
are unchanged. `ros2 launch emcl2 emcl2.launch.py` still resolves the installed package;
source-relative paths now start at `emcl2/`. This repository root has no package.xml.
Build from the workspace root with this repository as a colcon base path; both
packages are discovered automatically. Workspace integration also needs the existing
gnss2map, ublox_dgnss and cat_robot sources. See the
[layout migration record](docs/validation/emcl2_layout_20261117.md) for fresh-output
build/test commands, history provenance and validation limits.

The [COPYING](COPYING) license applies to emcl2. The interface package retains its
[Apache-2.0 license](emcl2_msgs/LICENSE). Historical evaluation records and calibration
conditions remain dated records; they are not current hardware acceptance.

# emcl2_msgs

Interfaces for localization consistency notifications and externally requested GNSS recovery.
The [authoritative recovery contract](../GNSS_RECOVERY.md) defines lifecycle,
QoS, timestamps, idempotency, validity and ownership. Navigation and motion permission
are outside these interfaces.

| Definition | Endpoint (default namespace) |
| --- | --- |
| [LocalizationStatus.msg](msg/LocalizationStatus.msg) | `/localization/status` |
| [RequestGnssReset.srv](srv/RequestGnssReset.srv) | `/localization/request_gnss_reset` |
| [GnssResetStatus.msg](msg/GnssResetStatus.msg) | `/localization/gnss_reset_status` |

This ROS package shares the emcl2_ros2 Git repository with the sibling `emcl2/` package.
Generation uses ROS 2 Humble; colcon orders this package before its consumers.
See the [repository entry](../README.md) and [layout migration](../docs/validation/emcl2_layout_20261117.md).
GNSS receiver status has its own [ublox definition](../../ublox_dgnss/ublox_ubx_msgs/msg/GnssStatus.msg).
[Implementation evidence](docs/validation/gnss_reset_20261117.md) and
[documentation review](docs/validation/gnss_docs_review_20261117.md) are dated records.

# emcl2_msgs

Interfaces for localization consistency notifications and externally requested GNSS recovery.
The [authoritative recovery contract](../emcl2_ros2/GNSS_RECOVERY.md) defines lifecycle,
QoS, timestamps, idempotency, validity and ownership. Navigation and motion permission
are outside these interfaces.

| Definition | Endpoint (default namespace) |
| --- | --- |
| [LocalizationStatus.msg](msg/LocalizationStatus.msg) | `/localization/status` |
| [RequestGnssReset.srv](srv/RequestGnssReset.srv) | `/localization/request_gnss_reset` |
| [GnssResetStatus.msg](msg/GnssResetStatus.msg) | `/localization/gnss_reset_status` |

Build this independent sibling package before emcl2; generation uses ROS 2 Humble.
GNSS receiver status has its own [ublox definition](../ublox_dgnss/ublox_ubx_msgs/msg/GnssStatus.msg).
[Implementation evidence](docs/validation/gnss_reset_20261117.md) and
[documentation review](docs/validation/gnss_docs_review_20261117.md) are dated records.

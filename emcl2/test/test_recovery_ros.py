"""Synthetic localhost graph only. No driver, launch, motion, bag or simulator."""
import math
import os
from pathlib import Path
import signal
import subprocess
import time
import uuid

import pytest
import rclpy
from ament_index_python.packages import get_package_prefix
from builtin_interfaces.msg import Duration
from emcl2_msgs.msg import LocalizationStatus, GnssResetStatus
from emcl2_msgs.srv import RequestGnssReset
from geometry_msgs.msg import PoseWithCovarianceStamped, TransformStamped
from nav_msgs.msg import OccupancyGrid
from sensor_msgs.msg import LaserScan
from rclpy.qos import QoSProfile, DurabilityPolicy, qos_profile_sensor_data
from tf2_msgs.msg import TFMessage


@pytest.fixture
def peer(tmp_path, request):
    if os.environ.get('GNSS_VALIDATION_LOG_DIR'):
        tmp_path = Path(os.environ['GNSS_VALIDATION_LOG_DIR']) / uuid.uuid4().hex
        tmp_path.mkdir(parents=True)
    assert os.environ['ROS_LOCALHOST_ONLY'] == '1'
    assert os.environ['ROS_DOMAIN_ID'] == '216'
    rclpy.init()
    ns = '/recovery_' + uuid.uuid4().hex
    node = rclpy.create_node('fixture', namespace=ns)
    executable = Path(get_package_prefix('emcl2')) / 'lib/emcl2/emcl2_node'
    pipeline = getattr(request, 'param', '') == 'ubx'
    log = (tmp_path / 'node.log').open('w')
    process = subprocess.Popen([str(executable), '--ros-args', '-r', '__ns:=' + ns,
        '-r', '/tf_static:=' + ns + '/tf_static', '-r', '/tf:=' + ns + '/tf',
        '-p', 'use_gnss_reset:=' + ('false' if getattr(request, 'param', '') == 'disabled' else 'true'), '-p', 'gnss_antenna_frame:=antenna',
        '-p', 'base_frame_id:=base_link', '-p', 'footprint_frame_id:=base_link',
        '-p', 'num_particles:=100', '-p', 'scan_max_age:=0.4',
        '-p', 'lost_hold_sec:=0.15', '-p', 'recovered_hold_sec:=0.4'],
        stdout=log, stderr=subprocess.STDOUT)
    processes = [process]
    logs = [log]
    if pipeline:
        config = tmp_path / 'calibration.yaml'
        config.write_text('/**:\n  ros__parameters:\n    p0: [0.2, 0.0, 0.0]\n    gnss0: [35.68, 140.02, 50.0]\n    p1: [100.2, 0.0]\n    gnss1: [35.68, 140.021105]\n')
        for package, name, arguments in [
            ('ublox_nav_sat_fix_hp_node', 'ublox_nav_sat_fix_hp',
             ['-r', 'fix:=gnss/fix', '-r', 'status:=gnss/status']),
            ('gnss2map', 'gauss_kruger_node', ['--params-file', str(config)])]:
            output = (tmp_path / (name + '.log')).open('w')
            logs.append(output)
            path = Path(get_package_prefix(package)) / 'lib' / package / name
            processes.append(subprocess.Popen([str(path), '--ros-args', '-r', '__ns:=' + ns,
                                                *arguments], stdout=output, stderr=subprocess.STDOUT))
    class Peer:
        scans = True
        gnss = True
        scan_range = 1.0
        frozen_scan_stamp = None
        def __init__(self):
            self.health, self.results, self.clouds = [], [], []
            latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
            self.map_pub = node.create_publisher(OccupancyGrid, 'map', latched)
            self.tf_pub = node.create_publisher(TFMessage, 'tf_static', latched)
            self.scan_pub = node.create_publisher(LaserScan, 'scan', 10)
            self.gnss_pub = node.create_publisher(PoseWithCovarianceStamped, 'gnss/map_pose', qos_profile_sensor_data)
            self.subs = [
                node.create_subscription(LocalizationStatus, 'localization/status', self.health.append, latched),
                node.create_subscription(GnssResetStatus, 'localization/gnss_reset_status', self.results.append, latched)]
            if pipeline:
                from ublox_ubx_msgs.msg import UBXNavHPPosLLH, UBXNavCov, UBXNavStatus, GnssStatus
                self.types = (UBXNavHPPosLLH, UBXNavCov, UBXNavStatus)
                self.ubx = [node.create_publisher(t, topic, 10) for t, topic in zip(self.types,
                    ['ubx_nav_hp_pos_llh', 'ubx_nav_cov', 'ubx_nav_status'])]
                self.reports = []
                self.subs.append(node.create_subscription(GnssStatus, 'gnss/status', self.reports.append, latched))
                self.epoch = 1000
            self.client = node.create_client(RequestGnssReset, 'localization/request_gnss_reset')
        def pump(self, seconds):
            deadline = time.monotonic() + seconds
            while time.monotonic() < deadline:
                assert process.poll() is None, (tmp_path / 'node.log').read_text()
                stamp = node.get_clock().now().to_msg()
                if self.scans:
                    scan = LaserScan()
                    scan.header.frame_id = 'laser'; scan.header.stamp = self.frozen_scan_stamp or stamp
                    self.last_scan_stamp = scan.header.stamp
                    scan.angle_min = -math.pi; scan.angle_increment = math.pi / 18
                    scan.angle_max = scan.angle_min + 35 * scan.angle_increment
                    scan.range_min = .1; scan.range_max = 30.
                    scan.ranges = [self.scan_range] * 36
                    self.scan_pub.publish(scan)
                if self.gnss:
                    pose = PoseWithCovarianceStamped()
                    pose.header.frame_id = 'map'; pose.header.stamp = stamp
                    pose.pose.pose.position.x = .2
                    pose.pose.covariance[0] = pose.pose.covariance[7] = .01
                    if not pipeline:
                        self.gnss_pub.publish(pose)
                    else:
                        pos, cov, status = (t() for t in self.types)
                        pos.lat = 356800000; pos.lon = 1400200000; pos.height = 50000
                        cov.pos_cor_valid = True
                        cov.pos_cov_nn = cov.pos_cov_ee = cov.pos_cov_dd = .01
                        status.gps_fix_ok = True; status.gps_fix.fix_type = 3
                        status.carr_soln_valid = True; status.carr_soln.status = 2
                        for pub, msg in zip(self.ubx, [pos, cov, status]):
                            msg.header.stamp = stamp; msg.header.frame_id = 'antenna'
                            msg.itow = self.epoch
                            pub.publish(msg)
                        self.epoch += 200
                rclpy.spin_once(node, timeout_sec=.02)
                time.sleep(.02)
        def call(self, identity, duration=.8):
            request = RequestGnssReset.Request(request_id=identity)
            request.timeout = Duration(sec=int(duration), nanosec=int((duration % 1) * 1e9))
            future = self.client.call_async(request)
            deadline = time.monotonic() + 3
            while not future.done() and time.monotonic() < deadline:
                self.pump(.03)
            assert future.done()
            return future.result()
    p = Peer()
    try:
        assert p.client.wait_for_service(timeout_sec=5)
        tf = []
        for parent, child in [('odom', 'base_link'), ('base_link', 'laser'), ('base_link', 'antenna')]:
            t = TransformStamped(); t.header.frame_id = parent; t.child_frame_id = child
            t.header.stamp = node.get_clock().now().to_msg(); t.transform.rotation.w = 1.
            tf.append(t)
        p.tf_pub.publish(TFMessage(transforms=tf))
        m = OccupancyGrid(); m.header.frame_id = 'map'; m.info.resolution = .1
        m.info.width = m.info.height = 100
        m.info.origin.position.x = m.info.origin.position.y = -5.
        m.info.origin.orientation.w = 1.
        # Obstacle ring at radius 2 m; short ranges do not penetrate it.
        m.data = [100 if 1.8 < math.hypot((i % 100 - 50) * .1, (i // 100 - 50) * .1) < 2.1 else 0
                  for i in range(10000)]
        p.map_pub.publish(m)
        deadline = time.monotonic() + 5
        while time.monotonic() < deadline:
            p.pump(.1)
            if p.health and p.health[-1].state == LocalizationStatus.TRACKING:
                break
        else:
            pytest.fail('No evaluated scan: ' + (tmp_path / 'node.log').read_text())
        yield p
    finally:
        for child in processes:
            child.send_signal(signal.SIGINT)
        for child in processes:
            try:
                child.wait(timeout=4)
            except subprocess.TimeoutExpired:
                child.kill(); child.wait()
        for output in logs:
            output.close()
        node.destroy_node(); rclpy.shutdown()


def test_explicit_request_and_idempotence(peer):
    assert not peer.results  # GNSS reception alone never starts a reset.
    accepted = peer.call('one', 2.)
    assert accepted.accepted
    duplicate = peer.call('one', 2.)
    assert duplicate.attempt_id == accepted.attempt_id
    busy = peer.call('two', 2.)
    assert not busy.accepted and busy.reason_code == busy.BUSY
    peer.pump(.8)
    result = peer.results[-1]
    assert result.state == GnssResetStatus.SUCCEEDED
    assert result.particles_replaced
    assert result.gnss_source_stamp.sec > 0
    replay = peer.call('one', 2.)
    assert replay.attempt_id == accepted.attempt_id


def test_deadline_continues_during_scan_loss(peer):
    accepted = peer.call('timeout', .8)
    assert accepted.accepted
    peer.scans = False
    peer.pump(1.2)
    assert peer.results[-1].state == GnssResetStatus.TIMED_OUT
    assert peer.health[-1].state == LocalizationStatus.UNAVAILABLE


def test_stale_gnss_and_invalid_request_rejected(peer):
    peer.gnss = False
    peer.pump(1.2)
    response = peer.call('stale', .8)
    assert not response.accepted and response.reason_code == response.NO_GNSS
    response = peer.call('', .8)
    assert not response.accepted and response.reason_code == response.INVALID_REQUEST


def test_lost_notification_and_scan_loss_are_distinct(peer):
    peer.scan_range = 4.0
    peer.pump(.8)
    assert any(m.state == LocalizationStatus.LOST for m in peer.health)
    peer.scans = False
    peer.pump(.6)
    assert peer.health[-1].state == LocalizationStatus.UNAVAILABLE
    assert not peer.results


@pytest.mark.parametrize('peer', ['ubx'], indirect=True)
def test_ublox_conversion_to_requested_reset(peer):
    peer.pump(.4)
    assert peer.reports[-1].position_valid
    assert peer.reports[-1].solution == peer.reports[-1].RTK_FIXED
    response = peer.call('pipeline', 2.)
    assert response.accepted
    peer.pump(.8)
    assert peer.results[-1].state == GnssResetStatus.SUCCEEDED


def test_duplicate_scan_cannot_execute_pending_reset(peer):
    peer.frozen_scan_stamp = peer.last_scan_stamp
    peer.pump(.12)  # Drain the last genuinely new scan before submitting the request.
    accepted = peer.call('new-scan-required', 2.)
    assert accepted.accepted
    peer.pump(.12)
    assert not peer.results[-1].particles_replaced
    peer.frozen_scan_stamp = None
    peer.pump(.8)
    assert peer.results[-1].particles_replaced
    assert peer.results[-1].state == GnssResetStatus.SUCCEEDED


@pytest.mark.parametrize('peer', ['disabled'], indirect=True)
def test_disabled_by_default_gate(peer):
    response = peer.call('disabled', 1.)
    assert not response.accepted and response.reason_code == response.DISABLED
    assert not peer.results

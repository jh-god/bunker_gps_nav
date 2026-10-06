"""Exercise the real bridge against a mock Nav2 server, TF and permission gate.

No controller or velocity publisher is used. Run in an isolated ROS domain.
"""
import os
from pathlib import Path
import queue
import signal
import subprocess
import threading
import time
import uuid

import pytest
import rclpy
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.qos import DurabilityPolicy, QoSProfile
from geographic_msgs.msg import GeoPoseStamped
from geometry_msgs.msg import PoseStamped, TransformStamped
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import Odometry
from robot_localization.srv import FromLL
from std_msgs.msg import Bool, String
from std_srvs.srv import Trigger
from tf2_ros import TransformBroadcaster
from visualization_msgs.msg import MarkerArray


def wait_for(predicate, timeout=5):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        if predicate():
            return
        time.sleep(0.02)
    raise AssertionError('Timed out waiting for bridge/mock state')


class Rig:
    def __init__(self, tmp_path):
        binary = os.environ.get('GOAL_BRIDGE_BINARY')
        assert binary and Path(binary).is_file(), 'Set GOAL_BRIDGE_BINARY to the built executable'
        self.ns = '/wp_test_' + uuid.uuid4().hex[:8]
        self.node = rclpy.create_node('mock_navigation', namespace=self.ns)
        self.group = ReentrantCallbackGroup()
        self.received = []
        self.commands = queue.Queue()
        self.canceled = 0
        self.allowed = True
        self.heartbeat = True
        self.reject = False
        self.ack_delay = 0
        self.ignore_cancel = False
        self.move_on_success = True
        self.robot_x = 0.0
        self.status = ''
        self.markers = None
        persistent = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.permission = self.node.create_publisher(Bool, 'motion_allowed', persistent)
        self.waypoint = self.node.create_publisher(PoseStamped, 'waypoint', 10)
        self.single = self.node.create_publisher(PoseStamped, 'goal_pose', 10)
        self.gps = self.node.create_publisher(GeoPoseStamped, 'gps_goal', 10)
        self.odom = self.node.create_publisher(Odometry, 'odometry_gps', 10)
        self.tf = TransformBroadcaster(self.node)
        self.map_frame = self.ns[1:] + '_map'
        self.base_frame = self.ns[1:] + '_base'
        self.node.create_subscription(String, 'waypoints/status', self.on_status, persistent)
        self.node.create_subscription(MarkerArray, 'waypoints/markers', self.on_markers, persistent)
        self.clients = {name: self.node.create_client(Trigger, 'waypoints/' + name)
                        for name in ('start', 'cancel', 'clear', 'remove_last')}
        self.server = ActionServer(
            self.node, NavigateToPose, 'navigate_to_pose', self.execute,
            goal_callback=self.goal, cancel_callback=self.cancel,
            callback_group=self.group)
        self.node.create_service(FromLL, 'fromLL', self.convert, callback_group=self.group)
        self.timer = self.node.create_timer(0.05, self.tick, callback_group=self.group)
        self.executor = MultiThreadedExecutor(num_threads=4)
        self.executor.add_node(self.node)
        self.thread = threading.Thread(target=self.executor.spin, daemon=True)
        self.thread.start()
        self.log = (tmp_path / 'bridge.log').open('w')
        self.process = subprocess.Popen([
            binary, '--ros-args', '-r', '__ns:=' + self.ns,
            '-p', 'map_frame:=' + self.map_frame,
            '-p', 'base_frame:=' + self.base_frame,
        ], stdout=self.log, stderr=subprocess.STDOUT)
        assert self.clients['start'].wait_for_service(timeout_sec=5)
        wait_for(lambda: self.waypoint.get_subscription_count() > 0 and self.status != '')
        time.sleep(0.2)  # Discover action endpoints and receive robot TF.

    def on_status(self, msg):
        self.status = msg.data

    def on_markers(self, msg):
        self.markers = msg

    def tick(self):
        if self.heartbeat:
            self.permission.publish(Bool(data=self.allowed))
        stamp = self.node.get_clock().now().to_msg()
        tf = TransformStamped()
        tf.header.frame_id, tf.child_frame_id = self.map_frame, self.base_frame
        tf.header.stamp = stamp
        tf.transform.translation.x = self.robot_x
        tf.transform.rotation.w = 1.0
        self.tf.sendTransform(tf)
        odom = Odometry()
        odom.header.stamp = stamp
        self.odom.publish(odom)

    def goal(self, request):
        time.sleep(self.ack_delay)
        return GoalResponse.REJECT if self.reject else GoalResponse.ACCEPT

    def cancel(self, handle):
        return CancelResponse.REJECT if self.ignore_cancel else CancelResponse.ACCEPT

    def execute(self, handle):
        self.received.append(handle.request.pose.pose.position.x)
        while rclpy.ok():
            if handle.is_cancel_requested:
                self.canceled += 1
                handle.canceled()
                return NavigateToPose.Result()
            try:
                command = self.commands.get(timeout=0.02)
            except queue.Empty:
                continue
            if command == 'success':
                if self.move_on_success:
                    self.robot_x = handle.request.pose.pose.position.x
                    self.tick()
                    time.sleep(0.1)  # Ensure new robot TF precedes the success result.
                handle.succeed()
            else:
                handle.abort()
            return NavigateToPose.Result()
        handle.abort()
        return NavigateToPose.Result()

    @staticmethod
    def convert(request, response):
        response.map_point.x = 4.0
        return response

    def pose(self, x):
        pose = PoseStamped()
        pose.header.frame_id = self.map_frame
        pose.pose.position.x = float(x)
        pose.pose.orientation.w = 1.0
        return pose

    def add(self, *xs):
        for x in xs:
            self.waypoint.publish(self.pose(x))
        wait_for(lambda: f'/ {len(xs)}' in self.status)

    def call(self, name):
        future = self.clients[name].call_async(Trigger.Request())
        wait_for(future.done)
        return future.result()

    def close(self):
        # Drain the server's result callbacks while its executor is still alive.
        # Stopping the executor first can strand an active execute callback.
        self.ignore_cancel = False
        try:
            self.call('cancel')
            wait_for(lambda: 'Paused: canceled by operator' in self.status and
                     'Waiting for Nav2' not in self.status)
        except AssertionError:
            self.commands.put('abort')
            time.sleep(0.1)
        self.process.send_signal(signal.SIGINT)
        try:
            self.process.wait(timeout=3)
        except subprocess.TimeoutExpired:
            self.process.kill()
            self.process.wait()
        self.log.close()
        # Unblock an execute callback before stopping its executor.
        self.commands.put('abort')
        self.executor.shutdown(timeout_sec=3)
        self.thread.join(timeout=3)
        self.server.destroy()
        self.node.destroy_node()


@pytest.fixture
def rig(tmp_path):
    rclpy.init()
    r = None
    try:
        r = Rig(tmp_path)
        yield r
    finally:
        if r is not None:
            r.close()
        rclpy.shutdown()


def test_ordered_route_longer_than_single_goal_limit(rig):
    rig.add(20, 40, 60)
    assert rig.call('start').success
    wait_for(lambda: len(rig.received) == 1)
    # Ordinary goals and route edits must not preempt a waypoint mission.
    rig.single.publish(rig.pose(3))
    assert not rig.call('clear').success
    assert not rig.call('remove_last').success
    for n in range(3):
        assert rig.received[n] == (n + 1) * 20
        rig.commands.put('success')
        if n < 2:
            wait_for(lambda: len(rig.received) == n + 2)
    wait_for(lambda: 'Completed' in rig.status)
    assert 'Reached 3 / 3' in rig.status
    assert not rig.call('start').success
    arrows = [m for m in rig.markers.markers if m.ns == 'waypoint_heading']
    assert len(arrows) == 3 and all(m.color.r < 0.5 for m in arrows)
    assert rig.call('clear').success
    wait_for(lambda: len(rig.markers.markers) == 1)  # DELETEALL


def test_distance_limits_and_undo(rig):
    rig.add(26)
    assert not rig.call('start').success
    assert not rig.received
    assert rig.call('clear').success
    rig.add(10, 40)
    result = rig.call('start')
    assert not result.success and 'Segment' in result.message
    assert rig.call('remove_last').success
    assert rig.call('start').success
    wait_for(lambda: rig.received == [10])


@pytest.mark.parametrize('stale', [False, True])
def test_gate_loss_cancels_and_requires_explicit_resume(rig, stale):
    rig.add(10, 20)
    assert rig.call('start').success
    wait_for(lambda: len(rig.received) == 1)
    if stale:
        rig.heartbeat = False
    else:
        rig.allowed = False
    wait_for(lambda: rig.canceled == 1)
    wait_for(lambda: 'Waiting for Nav2' not in rig.status)
    assert 'Reached 0 / 2' in rig.status
    rig.allowed, rig.heartbeat = True, True
    time.sleep(0.4)
    assert len(rig.received) == 1
    assert rig.call('start').success
    wait_for(lambda: rig.received == [10, 10])


def test_abort_and_rejection_do_not_skip_waypoints(rig):
    rig.add(10, 20)
    rig.reject = True
    assert rig.call('start').success
    wait_for(lambda: 'Nav2 rejected' in rig.status)
    assert 'Reached 0 / 2' in rig.status
    rig.reject = False
    assert rig.call('start').success
    wait_for(lambda: len(rig.received) == 1)
    rig.commands.put('abort')
    wait_for(lambda: 'Paused:' in rig.status)
    assert 'Reached 0 / 2' in rig.status
    assert rig.call('start').success
    wait_for(lambda: rig.received == [10, 10])


def test_cancel_before_goal_acknowledgment(rig):
    rig.ack_delay = 0.5
    rig.add(10, 20)
    assert rig.call('start').success
    assert rig.call('cancel').success
    assert not rig.call('clear').success
    wait_for(lambda: rig.canceled == 1)
    wait_for(lambda: 'Waiting for Nav2' not in rig.status)
    assert 'Reached 0 / 2' in rig.status
    assert rig.received == [10]
    assert rig.call('clear').success


def test_success_arriving_after_cancel_does_not_advance(rig):
    rig.ignore_cancel = True
    rig.add(10, 20)
    assert rig.call('start').success
    wait_for(lambda: len(rig.received) == 1)
    assert rig.call('cancel').success
    rig.commands.put('success')
    wait_for(lambda: 'Waiting for Nav2' not in rig.status)
    assert 'Reached 0 / 2' in rig.status
    assert rig.received == [10]


def test_actual_robot_distance_rechecked_between_waypoints(rig):
    rig.move_on_success = False
    rig.add(20, 40)
    assert rig.call('start').success
    wait_for(lambda: len(rig.received) == 1)
    rig.commands.put('success')
    wait_for(lambda: 'distance limit' in rig.status)
    assert 'Reached 1 / 2' in rig.status
    assert rig.received == [20]


def test_existing_single_goal_replacement_and_gps_conversion(rig):
    rig.single.publish(rig.pose(5))
    wait_for(lambda: rig.received == [5])
    rig.single.publish(rig.pose(6))
    wait_for(lambda: rig.received == [5, 6])
    assert rig.canceled == 1
    rig.commands.put('success')
    wait_for(lambda: 'Single goal reached' in rig.status)
    gps = GeoPoseStamped()
    gps.pose.position.latitude, gps.pose.position.longitude = 35.84, 127.13
    gps.pose.orientation.w = 1.0
    rig.gps.publish(gps)
    wait_for(lambda: rig.received == [5, 6, 4])

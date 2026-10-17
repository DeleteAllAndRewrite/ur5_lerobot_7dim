"""ROS Noetic bridge for MoveIt Servo cartesian velocity control.

This file is launched with the system Python because the capture conda
environment does not contain rospy.
"""
import json
import math
import os
import select
import sys
import traceback

import rospy
import tf2_ros
from geometry_msgs.msg import TwistStamped
from sensor_msgs.msg import JointState


JOINT_NAMES = (
    "shoulder_pan_joint",
    "shoulder_lift_joint",
    "elbow_joint",
    "wrist_1_joint",
    "wrist_2_joint",
    "wrist_3_joint",
)


def _rotation_vector(qx, qy, qz, qw):
    norm = math.sqrt(qx * qx + qy * qy + qz * qz + qw * qw)
    if norm <= 1e-12:
        raise RuntimeError("TF returned a zero-length quaternion")
    qx, qy, qz, qw = qx / norm, qy / norm, qz / norm, qw / norm
    if qw < 0:
        qx, qy, qz, qw = -qx, -qy, -qz, -qw
    sine = math.sqrt(qx * qx + qy * qy + qz * qz)
    if sine <= 1e-12:
        return [0.0, 0.0, 0.0]
    angle = 2.0 * math.atan2(sine, max(-1.0, min(1.0, qw)))
    scale = angle / sine
    return [qx * scale, qy * scale, qz * scale]


def _rotvec_quaternion(values):
    angle = _norm(values)
    if angle <= 1e-12:
        return [0.0, 0.0, 0.0, 1.0]
    scale = math.sin(angle / 2.0) / angle
    return [values[0] * scale, values[1] * scale, values[2] * scale, math.cos(angle / 2.0)]


def _quat_multiply(left, right):
    lx, ly, lz, lw = left
    rx, ry, rz, rw = right
    return [
        lw * rx + lx * rw + ly * rz - lz * ry,
        lw * ry - lx * rz + ly * rw + lz * rx,
        lw * rz + lx * ry - ly * rx + lz * rw,
        lw * rw - lx * rx - ly * ry - lz * rz,
    ]


def _orientation_error(current, target):
    current_q = _rotvec_quaternion(current)
    target_q = _rotvec_quaternion(target)
    relative = _quat_multiply(
        target_q,
        [-current_q[0], -current_q[1], -current_q[2], current_q[3]],
    )
    return _rotation_vector(relative[0], relative[1], relative[2], relative[3])


class ServoBridge:
    def __init__(self):
        ros_config = json.loads(os.environ.get("UR5_CAPTURE_ROS_CONFIG", "{}"))
        self.joint_state = None
        self.target = None
        self.enabled = False
        self.duration = 0.033
        self.last_command = rospy.get_time()
        self.tf_buffer = tf2_ros.Buffer(cache_time=rospy.Duration(10.0))
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        self.publisher = rospy.Publisher(
            ros_config.get("twist_topic", "/servo_server/delta_twist_cmds"),
            TwistStamped,
            queue_size=1,
        )
        self.joint_topic = ros_config.get("joint_topic", "/joint_states")
        self.base_frame = ros_config.get("base_frame", "base_link")
        self.tool_frame = ros_config.get("tool_frame", "tool0")
        self.max_linear = 0.05
        self.max_angular = 0.3
        self.command_timeout = 0.15
        rospy.Subscriber(self.joint_topic, JointState, self._joint_cb, queue_size=1)

    def _joint_cb(self, message):
        values = dict(zip(message.name, message.position))
        if all(name in values for name in JOINT_NAMES):
            self.joint_state = [float(values[name]) for name in JOINT_NAMES]
        elif len(message.position) >= 6:
            self.joint_state = [float(value) for value in message.position[:6]]

    def state(self):
        transform = self.tf_buffer.lookup_transform(
            self.base_frame, self.tool_frame, rospy.Time(0), rospy.Duration(0.05)
        )
        translation = transform.transform.translation
        rotation = transform.transform.rotation
        pose = [
            float(translation.x),
            float(translation.y),
            float(translation.z),
            *_rotation_vector(rotation.x, rotation.y, rotation.z, rotation.w),
        ]
        if self.joint_state is None:
            raise RuntimeError("No joint_states received")
        return pose, self.joint_state

    def command(self, message):
        target = message.get("target")
        if not isinstance(target, list) or len(target) != 6:
            raise ValueError("ROS Servo target must contain 6 values")
        self.target = [float(value) for value in target]
        self.duration = max(float(message.get("duration", 0.033)), 1e-3)
        self.enabled = bool(message.get("enabled"))
        self.max_linear = max(float(message.get("max_translation_speed", 0.05)), 1e-6)
        self.max_angular = max(float(message.get("max_rotation_speed", 0.3)), 1e-6)
        self.command_timeout = max(float(message.get("command_timeout_s", 0.15)), 0.01)
        self.last_command = rospy.get_time()

    def stop(self):
        self.enabled = False
        self.target = None
        self._publish_zero()

    def _publish_zero(self):
        message = TwistStamped()
        message.header.stamp = rospy.Time.now()
        message.header.frame_id = self.base_frame
        self.publisher.publish(message)

    def publish(self, pose):
        message = TwistStamped()
        message.header.stamp = rospy.Time.now()
        message.header.frame_id = self.base_frame
        if self.enabled and self.target is not None:
            if rospy.get_time() - self.last_command > self.command_timeout:
                self.stop()
                return
            linear = [self.target[i] - pose[i] for i in range(3)]
            angular = _orientation_error(pose[3:6], self.target[3:6])
            linear_scale = min(1.0, self.max_linear * self.duration / max(_norm(linear), 1e-9))
            angular_scale = min(1.0, self.max_angular * self.duration / max(_norm(angular), 1e-9))
            message.twist.linear.x = linear[0] / self.duration * linear_scale
            message.twist.linear.y = linear[1] / self.duration * linear_scale
            message.twist.linear.z = linear[2] / self.duration * linear_scale
            message.twist.angular.x = angular[0] / self.duration * angular_scale
            message.twist.angular.y = angular[1] / self.duration * angular_scale
            message.twist.angular.z = angular[2] / self.duration * angular_scale
        self.publisher.publish(message)


def _norm(values):
    return math.sqrt(sum(value * value for value in values))


def _write(message):
    sys.stdout.write(json.dumps(message, separators=(",", ":")) + "\n")
    sys.stdout.flush()


def main():
    rospy.init_node("ur5_capture_servo_bridge", anonymous=True, disable_signals=True)
    bridge = ServoBridge()
    rate = rospy.Rate(125)
    ready = False
    while not rospy.is_shutdown():
        try:
            pose, joints = bridge.state()
            if not ready:
                _write({"ready": True})
                ready = True
            bridge.publish(pose)
            _write({"device_time": rospy.get_time(), "pose": pose, "joints": joints})
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException,
                tf2_ros.ExtrapolationException, rospy.ROSException, RuntimeError):
            if ready:
                bridge._publish_zero()
        if select.select([sys.stdin], [], [], 0)[0]:
            line = sys.stdin.readline()
            if not line:
                bridge.stop()
                return
            try:
                message = json.loads(line)
                kind = message.get("kind")
                if kind == "command":
                    bridge.command(message)
                    _write({"ok": True})
                elif kind == "stop":
                    bridge.stop()
                    _write({"ok": True})
                elif kind == "quit":
                    bridge.stop()
                    return
                else:
                    raise ValueError(f"Unknown command kind: {kind!r}")
            except Exception as exc:
                _write({"error": f"{type(exc).__name__}: {exc}"})
        rate.sleep()


if __name__ == "__main__":
    try:
        main()
    except BaseException as exc:
        _write({"error": f"{type(exc).__name__}: {exc}", "traceback": traceback.format_exc()})
        raise

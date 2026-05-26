"""
teleop_node.py — Quest2ROS controller → FR5 cobot teleoperation (ROS2).

Pipeline:
  Quest app → ROS-TCP-Endpoint → /q2r_<hand>_hand_pose (+_inputs)
            → this node: clutch-gated delta-Cartesian → IK → ServoJ (Fairino SDK)
            → also publishes /joint_states for RViz2.

Deadman/clutch model (see frame_transform.py): the arm only tracks while the
grip (press_middle) is held. On press we snapshot controller + robot reference
poses; on release the arm holds. Re-press anywhere to reposition without moving
the robot — like lifting a mouse.

Actuation goes through the Fairino SDK directly (proven ServoJ + IK path), NOT
ros2_control — the frcobot ros2_control stack is sim-only on this hardware.
"""

import math
import threading
import time

import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor

from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import JointState
from quest2ros_msgs.msg import OVR2ROSInputs

from fr5_quest_teleop import config as C
from fr5_quest_teleop.fr5_driver import FR5Driver
from fr5_quest_teleop.frame_transform import (
    DeltaCartesianMapper, rate_limit_joints, clamp_joint_limits,
)
from fr5_quest_teleop.gripper import GripperController


class FR5QuestTeleop(Node):
    def __init__(self):
        super().__init__("fr5_quest_teleop")

        # ── parameters (override config.py at launch) ──────────────────────────
        p = self.declare_parameter
        self.fr5_ip = p("fr5_ip", C.FR5_IP).value
        self.servo_vel = float(p("servo_vel", C.FR5_SERVO_VEL).value)
        self.filter_t = float(p("filter_t", C.FR5_FILTER_T).value)
        self.loop_hz = float(p("loop_hz", C.LOOP_HZ).value)
        self.active_hand = p("active_hand", C.ACTIVE_HAND).value
        self.position_scale = float(p("position_scale", C.POSITION_SCALE).value)
        self.rotation_scale = float(p("rotation_scale", C.ROTATION_SCALE).value)
        self.control_orientation = bool(p("control_orientation", C.CONTROL_ORIENTATION).value)
        self.gripper_enable = bool(p("gripper_enable", C.GRIPPER_ENABLE).value)

        hand = "right" if self.active_hand == "right" else "left"
        pose_topic = f"/q2r_{hand}_hand_pose"
        inputs_topic = f"/q2r_{hand}_hand_inputs"

        # ── latest messages (written by callbacks, read by control thread) ──────
        self._lock = threading.Lock()
        self._pose = None        # PoseStamped
        self._inputs = None      # OVR2ROSInputs

        cb = ReentrantCallbackGroup()
        self.create_subscription(PoseStamped, pose_topic, self._on_pose, 10, callback_group=cb)
        self.create_subscription(OVR2ROSInputs, inputs_topic, self._on_inputs, 10, callback_group=cb)
        self._js_pub = self.create_publisher(JointState, "/joint_states", 10)

        # ── mapper / driver / gripper ──────────────────────────────────────────
        self.mapper = DeltaCartesianMapper(
            axis_map=C.AXIS_MAP,
            pos_scale=self.position_scale,
            rot_scale=self.rotation_scale,
            max_pos_mm=C.MAX_DELTA_POS_MM,
            max_rot_deg=C.MAX_DELTA_ROT_DEG,
            euler_conv=C.EULER_CONVENTION,
            control_orientation=self.control_orientation,
        )
        self.driver = FR5Driver(self.fr5_ip, self.servo_vel, self.filter_t)
        self.gripper = None

        self._prev_joints = [0.0] * 6
        self._engaged = False
        self._cycle = 0
        self._stop = threading.Event()

        self.get_logger().info(
            f"Subscribing: {pose_topic}, {inputs_topic}  | hand={hand}  "
            f"orientation={'on' if self.control_orientation else 'off'}"
        )

    # ── subscription callbacks ────────────────────────────────────────────────

    def _on_pose(self, msg: PoseStamped):
        with self._lock:
            self._pose = msg

    def _on_inputs(self, msg: OVR2ROSInputs):
        with self._lock:
            self._inputs = msg

    def _latest(self):
        with self._lock:
            return self._pose, self._inputs

    # ── hardware bring-up ─────────────────────────────────────────────────────

    def connect_robot(self):
        self.get_logger().info(f"Connecting to FR5 at {self.fr5_ip} ...")
        self.driver.connect()
        self._prev_joints = self.driver.get_joint_positions()
        if self.gripper_enable:
            self.gripper = GripperController(
                self.driver,
                index=C.GRIPPER_INDEX, gtype=C.GRIPPER_TYPE,
                open_pct=C.GRIPPER_OPEN_PCT, close_pct=C.GRIPPER_CLOSE_PCT,
                vel_pct=C.GRIPPER_VEL_PCT, force_pct=C.GRIPPER_FORCE_PCT,
                maxtime_ms=C.GRIPPER_MAXTIME_MS,
                open_thr=C.GRIPPER_OPEN_THRESHOLD, close_thr=C.GRIPPER_CLOSE_THRESHOLD,
            )
            if not self.gripper.start():
                self.gripper = None
        self.driver.start_servo_mode()
        self.get_logger().info(
            "Servo mode ready. HOLD GRIP (press_middle) to drive; release to hold. "
            "Ctrl-C to quit."
        )

    # ── control loop (runs in its own thread) ─────────────────────────────────

    def control_loop(self):
        period = 1.0 / self.loop_hz
        deadman_field = C.DEADMAN_FIELD
        trigger_field = C.GRIPPER_TRIGGER_FIELD

        while not self._stop.is_set() and rclpy.ok():
            t0 = time.monotonic()
            try:
                pose, inputs = self._latest()

                # Gripper handshake (pauses servo if a state change is pending)
                if self.gripper is not None:
                    if inputs is not None:
                        self.gripper.update_trigger(getattr(inputs, trigger_field))
                    if self.gripper.wants_pause():
                        self.gripper.pause_and_send()
                        self._prev_joints = self.driver.get_joint_positions()

                deadman = (
                    inputs is not None
                    and getattr(inputs, deadman_field) >= C.DEADMAN_THRESHOLD
                )

                if deadman and pose is not None:
                    ctrl_pos = (pose.pose.position.x, pose.pose.position.y, pose.pose.position.z)
                    q = pose.pose.orientation
                    ctrl_quat = (q.x, q.y, q.z, q.w)

                    if not self._engaged:
                        # rising edge — snapshot references
                        self._prev_joints = self.driver.get_joint_positions()
                        self.mapper.engage(ctrl_pos, ctrl_quat, self.driver.get_eef_pose())
                        self._engaged = True
                        self.get_logger().info("[CLUTCH] engaged — tracking")

                    target_eef = self.mapper.compute_target(ctrl_pos, ctrl_quat)
                    joints = self.driver.get_inverse_kin(target_eef, self._prev_joints)
                    joints = clamp_joint_limits(joints, C.FR5_JOINT_LIMITS)
                    joints = rate_limit_joints(joints, self._prev_joints, C.MAX_DELTA_PER_JOINT)
                    self.driver.servo_j(joints)
                    self._prev_joints = joints
                else:
                    if self._engaged:
                        self.mapper.release()
                        self._engaged = False
                        self.get_logger().info("[CLUTCH] released — holding")
                    # keep servo mode fed while holding
                    self.driver.servo_j(self._prev_joints)

                self._publish_joint_states()
                self._cycle += 1

            except Exception as exc:  # noqa: BLE001 — one bad cycle must not kill teleop
                if self._cycle % 50 == 0:
                    self.get_logger().warn(f"cycle error: {exc!r}")

            sleep = period - (time.monotonic() - t0)
            if sleep > 0:
                time.sleep(sleep)

    def _publish_joint_states(self):
        js = JointState()
        js.header.stamp = self.get_clock().now().to_msg()
        js.name = list(C.JOINT_NAMES)
        js.position = [math.radians(j) for j in self._prev_joints]
        self._js_pub.publish(js)

    # ── shutdown ──────────────────────────────────────────────────────────────

    def shutdown(self):
        self._stop.set()
        if self.gripper is not None:
            self.gripper.stop()
        try:
            self.driver.stop_servo_mode()
            self.driver.stop()
            self.driver.disconnect()
        except Exception:
            pass


def main(args=None):
    rclpy.init(args=args)
    node = FR5QuestTeleop()

    executor = MultiThreadedExecutor()
    executor.add_node(node)
    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()

    try:
        node.connect_robot()
        node.control_loop()
    except KeyboardInterrupt:
        pass
    except Exception as exc:  # noqa: BLE001
        node.get_logger().error(f"fatal: {exc!r}")
    finally:
        node.shutdown()
        executor.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()

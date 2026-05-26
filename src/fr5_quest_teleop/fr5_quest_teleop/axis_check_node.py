"""
axis_check_node.py — calibrate AXIS_MAP without touching the robot.

The old project guessed the Quest→robot axis mapping and got it wrong, so the
end-effector moved in the wrong directions. This tool removes the guessing:

  1. Run it (robot can be on or off — it is never commanded).
  2. Put on the headset, hold the grip on the active controller.
  3. Move the controller a clear ~20 cm along ONE direction at a time:
        push AWAY from you, then pull toward you
        move RIGHT, then LEFT
        move UP, then DOWN
  4. For each, read which raw Quest axis dominates the delta, and what the
     current AXIS_MAP turns it into (FR5 base delta).
  5. Edit AXIS_MAP in config.py until "push away" → FR5 +X (or whatever your
     mounting requires), "right" → FR5 +Y, "up" → FR5 +Z.

Only -1/0/+1 entries, and AXIS_MAP must stay a proper rotation (the node warns
if your current matrix is not).
"""

import threading

import numpy as np
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from quest2ros_msgs.msg import OVR2ROSInputs

from fr5_quest_teleop import config as C
from fr5_quest_teleop.frame_transform import DeltaCartesianMapper


class AxisCheck(Node):
    def __init__(self):
        super().__init__("fr5_quest_axis_check")
        hand = "right" if C.ACTIVE_HAND == "right" else "left"
        self._lock = threading.Lock()
        self._pose = None
        self._inputs = None
        self._ref = None     # reference position captured on grip press

        try:
            self._mapper = DeltaCartesianMapper(
                C.AXIS_MAP, 1.0, 1.0, 1e9, 1e9, C.EULER_CONVENTION
            )
            self.get_logger().info("AXIS_MAP is a valid proper rotation.")
        except ValueError as e:
            self._mapper = None
            self.get_logger().error(f"AXIS_MAP INVALID: {e}")

        self.create_subscription(PoseStamped, f"/q2r_{hand}_hand_pose", self._on_pose, 10)
        self.create_subscription(OVR2ROSInputs, f"/q2r_{hand}_hand_inputs", self._on_inputs, 10)
        self.create_timer(0.1, self._tick)   # 10 Hz console readout
        self.get_logger().info(
            f"Axis check on /q2r_{hand}_hand_*. Hold GRIP and move the controller "
            "along one axis at a time."
        )

    def _on_pose(self, msg):
        with self._lock:
            self._pose = msg

    def _on_inputs(self, msg):
        with self._lock:
            self._inputs = msg

    def _tick(self):
        with self._lock:
            pose, inputs = self._pose, self._inputs
        if pose is None or inputs is None:
            return

        held = getattr(inputs, C.DEADMAN_FIELD) >= C.DEADMAN_THRESHOLD
        pos = np.array([pose.pose.position.x, pose.pose.position.y, pose.pose.position.z])

        if held and self._ref is None:
            self._ref = pos.copy()
            self.get_logger().info("[REF SET] move along ONE axis now")
            return
        if not held:
            self._ref = None
            return

        d_ctrl = pos - self._ref                 # m, Quest frame
        if np.linalg.norm(d_ctrl) < 0.03:        # ignore < 3 cm jitter
            return

        labels = ("Qx", "Qy", "Qz")
        dom = int(np.argmax(np.abs(d_ctrl)))
        msg = (f"Quest Δ(cm)=[{d_ctrl[0]*100:+5.1f} {d_ctrl[1]*100:+5.1f} "
               f"{d_ctrl[2]*100:+5.1f}]  dominant={labels[dom]}{'+' if d_ctrl[dom]>0 else '-'}")
        if self._mapper is not None:
            d_base = self._mapper.map_position_delta(d_ctrl)   # mm, base frame
            msg += (f"  →  FR5 base Δ(mm)=[{d_base[0]:+6.1f} {d_base[1]:+6.1f} "
                    f"{d_base[2]:+6.1f}]")
        self.get_logger().info(msg)


def main(args=None):
    rclpy.init(args=args)
    node = AxisCheck()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()

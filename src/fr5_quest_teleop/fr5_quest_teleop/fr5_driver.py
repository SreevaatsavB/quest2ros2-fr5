"""
fr5_driver.py — FR5 cobot control via the Fairino SDK (ServoJ + IK).

This is the actuation path that is known to work on the real robot. It is a
thin, thread-safe wrapper over the vendor `fairino` SDK (XML-RPC). The teleop
node owns one instance and calls it from a single control thread; the gripper
runs on a background thread, so every RPC is serialised behind one lock
(xmlrpc.client is not thread-safe).

Imported lazily: `fairino` only exists on the Linux box with the vendor wheel
installed, so importing this module on a dev machine without the SDK will fail
only when FR5Driver() is constructed, not at import time.
"""

import threading
import time


class FR5Driver:
    def __init__(self, ip: str, servo_vel: float, filter_t: float):
        # Import here so the module can be imported on machines without the SDK
        # (e.g. for unit-testing the pure-math helpers).
        from fairino import Robot

        self._Robot = Robot
        self._ip = ip
        self._servo_vel = servo_vel
        self._filter_t = filter_t
        self._robot = None
        self._rpc_lock = threading.Lock()

    # ── lifecycle ──────────────────────────────────────────────────────────────

    def connect(self):
        self._robot = self._Robot.RPC(self._ip)
        self._Robot.RPC.is_conect = True   # force XML-RPC mode

        self._robot.StopMove()
        self._robot.ResetAllError()
        time.sleep(0.3)
        self._robot.Mode(0)
        self._robot.RobotEnable(1)
        time.sleep(0.5)   # servo drives need ~500 ms to energise after enable

    def start_servo_mode(self):
        time.sleep(0.1)
        with self._rpc_lock:
            err = self._robot.ServoMoveStart()
        if err not in (0, None):
            raise IOError(f"ServoMoveStart failed with error {err}")

    def stop_servo_mode(self):
        try:
            with self._rpc_lock:
                self._robot.ServoMoveEnd()
        except Exception:
            pass

    def reset_errors(self):
        with self._rpc_lock:
            self._robot.ResetAllError()

    def enable(self):
        with self._rpc_lock:
            self._robot.RobotEnable(1)

    def disconnect(self):
        self.stop_servo_mode()
        self._robot = None

    # ── state reads ─────────────────────────────────────────────────────────────

    def get_joint_positions(self) -> list[float]:
        with self._rpc_lock:
            raw = self._robot.GetActualJointPosDegree(0)
        return self._unwrap(raw, "GetActualJointPosDegree")

    def get_eef_pose(self) -> list[float]:
        """[x_mm, y_mm, z_mm, rx_deg, ry_deg, rz_deg] — TCP pose in base frame."""
        with self._rpc_lock:
            raw = self._robot.GetActualTCPPose(0)
        return self._unwrap(raw, "GetActualTCPPose")

    def get_joint_velocities(self) -> list[float]:
        with self._rpc_lock:
            raw = self._robot.GetActualJointSpeedsDegree(0)
        return self._unwrap(raw, "GetActualJointSpeedsDegree")

    def get_inverse_kin(self, eef_pose: list[float], ref_joints: list[float]) -> list[float]:
        """
        IK for a target TCP pose, seeded by ref_joints to pick the nearest
        solution (avoids elbow/wrist flips between cycles).

        eef_pose   — [x_mm, y_mm, z_mm, rx_deg, ry_deg, rz_deg]
        ref_joints — reference joint angles (deg)
        """
        eef_pose = [float(v) for v in eef_pose]
        ref_joints = [float(v) for v in ref_joints]
        with self._rpc_lock:
            raw = self._robot.GetInverseKinRef(0, eef_pose, ref_joints)
        return self._unwrap(
            raw, "GetInverseKinRef", err_hint=" (target pose may be unreachable)"
        )

    # ── motion ──────────────────────────────────────────────────────────────────

    def servo_j(self, joints_deg: list[float]):
        # xmlrpc.client cannot marshal numpy.float64 — coerce at the boundary.
        joints_deg = [float(j) for j in joints_deg]
        with self._rpc_lock:
            err = self._robot.ServoJ(
                joints_deg, [0] * 6, self._servo_vel, 0, 0.008, self._filter_t, 0
            )
        if err not in (0, None):
            raise IOError(f"ServoJ failed with error {err}")

    def stop(self):
        try:
            with self._rpc_lock:
                self._robot.StopMotion()
        except Exception:
            pass

    # ── gripper ───────────────────────────────────────────────────────────────

    def activate_gripper(self, index: int) -> int:
        with self._rpc_lock:
            return self._robot.ActGripper(index, 1)

    def send_gripper(self, index, pct, vel, force, maxtime, blocking, gtype) -> int:
        with self._rpc_lock:
            return self._robot.MoveGripper(
                index, int(pct), int(vel), int(force), int(maxtime), blocking, gtype, 0, 0, 0
            )

    # ── helpers ─────────────────────────────────────────────────────────────────

    @staticmethod
    def _unwrap(raw, name: str, err_hint: str = "") -> list[float]:
        """Fairino RPCs return [err_code, payload]. Validate and unwrap."""
        if not isinstance(raw, (list, tuple)) or len(raw) != 2:
            raise IOError(
                f"{name} returned unexpected value: {raw!r} "
                "(CNDE may not be connected — kill any stale session and retry)"
            )
        err, payload = raw
        if err != 0:
            raise IOError(f"{name} failed with error {err}{err_hint}")
        return [float(v) for v in payload]

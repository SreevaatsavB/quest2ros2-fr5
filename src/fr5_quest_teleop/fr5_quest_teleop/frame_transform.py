"""
frame_transform.py — clutch-based delta-Cartesian mapping, done correctly.

WHY THIS EXISTS / what broke before
-----------------------------------
The previous WebXR attempt mapped controller *rotation* into the robot frame by
rotating only the rotation-vector axis (`R @ rotvec`). That is wrong: to express
a rotation ΔR that is defined in frame A inside frame B you must *conjugate* it,
  ΔR_B = M · ΔR_A · Mᵀ        (M = rotation A→B)
Rotating just the axis throws away the consistency between axis and angle and
makes the end-effector tumble unpredictably — exactly the "absolute bullshit"
behaviour. Here we use scipy Rotation and conjugate properly.

MODEL
-----
Clutch (deadman) based, like lifting and repositioning a mouse:
  * On clutch engage we snapshot the controller pose (ctrl_ref) AND the robot
    TCP pose (fr5_ref). From then on we only ever apply the *delta* of the
    controller since engage — so the robot never jumps, regardless of where the
    controller happens to be.
  * Release → hold. Re-engage → fresh snapshot (reposition without moving robot).

M (POS/ROT axis map) maps the Quest controller frame to the FR5 base frame. It
MUST be a proper rotation (orthonormal, det = +1) because both frames are
right-handed; a reflection (det = -1) would invert rotation sense. We validate
this at construction. Calibrate M empirically with the axis_check node.
"""

import numpy as np
from scipy.spatial.transform import Rotation


def _as_rotation_or_raise(matrix3x3) -> Rotation:
    M = np.asarray(matrix3x3, dtype=np.float64).reshape(3, 3)
    det = np.linalg.det(M)
    ortho_err = np.abs(M @ M.T - np.eye(3)).max()
    if ortho_err > 1e-6:
        raise ValueError(
            f"AXIS_MAP is not orthonormal (max |M·Mᵀ−I| = {ortho_err:.2e}). "
            "Use only proper rotations: axis permutations with an even number of "
            "sign flips, or 90/180° rotations."
        )
    if det < 0:
        raise ValueError(
            f"AXIS_MAP has det = {det:+.3f} (a reflection). Two right-handed "
            "frames are related by a proper rotation (det = +1). Flip an even "
            "number of axes, or swap two axes and flip one."
        )
    return Rotation.from_matrix(M)


class DeltaCartesianMapper:
    """
    Maps Quest controller pose deltas → FR5 TCP target pose, clutch-based.

    Parameters
    ----------
    axis_map : 3x3 proper rotation (Quest frame → FR5 base frame)
    pos_scale : EEF mm per metre of controller travel (1000 = 1:1)
    rot_scale : multiplier on controller rotation magnitude (1.0 = 1:1)
    max_pos_mm : clamp on |target − ref| per axis (workspace guard)
    max_rot_deg : clamp on rotation delta magnitude (deg)
    euler_conv : scipy euler sequence matching the FR5 TCP angle convention
    """

    def __init__(
        self,
        axis_map,
        pos_scale: float,
        rot_scale: float,
        max_pos_mm: float,
        max_rot_deg: float,
        euler_conv: str = "xyz",
        control_orientation: bool = True,
    ):
        self._M = _as_rotation_or_raise(axis_map)
        self._pos_scale = float(pos_scale)
        self._rot_scale = float(rot_scale)
        self._max_pos_mm = float(max_pos_mm)
        self._max_rot_deg = float(max_rot_deg)
        self._euler_conv = euler_conv
        self._control_orientation = control_orientation

        # references captured on engage()
        self._ctrl_ref_pos = None    # np(3) metres, Quest frame
        self._ctrl_ref_rot = None    # Rotation, Quest frame
        self._fr5_ref_xyz = None     # np(3) mm, base frame
        self._fr5_ref_rot = None     # Rotation, base frame

    # ── clutch ─────────────────────────────────────────────────────────────────

    def engage(self, ctrl_pos, ctrl_quat_xyzw, fr5_eef_pose):
        """Snapshot controller and robot reference poses at clutch press."""
        self._ctrl_ref_pos = np.asarray(ctrl_pos, dtype=np.float64)
        self._ctrl_ref_rot = Rotation.from_quat(np.asarray(ctrl_quat_xyzw, dtype=np.float64))
        self._fr5_ref_xyz = np.asarray(fr5_eef_pose[:3], dtype=np.float64)
        self._fr5_ref_rot = Rotation.from_euler(
            self._euler_conv, fr5_eef_pose[3:6], degrees=True
        )

    @property
    def engaged(self) -> bool:
        return self._ctrl_ref_pos is not None

    def release(self):
        self._ctrl_ref_pos = None
        self._ctrl_ref_rot = None
        self._fr5_ref_xyz = None
        self._fr5_ref_rot = None

    # ── per-cycle mapping ────────────────────────────────────────────────────────

    def compute_target(self, ctrl_pos, ctrl_quat_xyzw):
        """
        Return target FR5 TCP pose [x_mm, y_mm, z_mm, rx_deg, ry_deg, rz_deg].
        Must be called only while engaged.
        """
        if not self.engaged:
            raise RuntimeError("compute_target() called before engage()")

        ctrl_pos = np.asarray(ctrl_pos, dtype=np.float64)
        ctrl_rot = Rotation.from_quat(np.asarray(ctrl_quat_xyzw, dtype=np.float64))

        # ── position: rotate the controller travel into the base frame ──────────
        d_ctrl_m = ctrl_pos - self._ctrl_ref_pos            # metres, Quest frame
        d_base_mm = self._M.apply(d_ctrl_m) * 1000.0 * self._pos_scale
        d_base_mm = np.clip(d_base_mm, -self._max_pos_mm, self._max_pos_mm)
        target_xyz = self._fr5_ref_xyz + d_base_mm

        # ── orientation: conjugate the delta rotation into the base frame ───────
        if self._control_orientation:
            dR_ctrl = ctrl_rot * self._ctrl_ref_rot.inv()    # Quest frame
            dR_base = self._M * dR_ctrl * self._M.inv()       # CONJUGATION (the fix)

            # scale + clamp the rotation magnitude
            rotvec = dR_base.as_rotvec()
            angle = np.linalg.norm(rotvec)
            if angle > 1e-9:
                scaled = angle * self._rot_scale
                scaled = float(np.clip(scaled, -np.radians(self._max_rot_deg),
                                       np.radians(self._max_rot_deg)))
                dR_base = Rotation.from_rotvec(rotvec / angle * scaled)
            else:
                dR_base = Rotation.identity()

            target_rot = dR_base * self._fr5_ref_rot
            target_euler = target_rot.as_euler(self._euler_conv, degrees=True)
        else:
            target_euler = self._fr5_ref_rot.as_euler(self._euler_conv, degrees=True)

        return [
            float(target_xyz[0]), float(target_xyz[1]), float(target_xyz[2]),
            float(target_euler[0]), float(target_euler[1]), float(target_euler[2]),
        ]

    # ── used by the axis_check tool ──────────────────────────────────────────────

    def map_position_delta(self, d_ctrl_m):
        """Raw position-delta mapping (m, Quest) → (mm, base), no clamp/scale."""
        return self._M.apply(np.asarray(d_ctrl_m, dtype=np.float64)) * 1000.0


def rate_limit_joints(target_deg, prev_deg, per_joint_limit_deg, scale=1.0):
    """Clamp |target − prev| per joint to per_joint_limit_deg * scale (deg/cycle)."""
    out = []
    for t, p, lim in zip(target_deg, prev_deg, per_joint_limit_deg):
        d = float(np.clip(t - p, -lim * scale, lim * scale))
        out.append(float(p + d))
    return out


def clamp_joint_limits(joints_deg, limits):
    """Clamp each joint to its (lo, hi) hard limit."""
    return [max(lo, min(hi, float(j))) for j, (lo, hi) in zip(joints_deg, limits)]

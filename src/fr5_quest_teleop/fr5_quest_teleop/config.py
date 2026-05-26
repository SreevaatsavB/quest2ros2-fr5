"""
config.py — tunables for Quest2ROS → FR5 teleop.

Every value here is also exposed as a ROS2 parameter on the teleop node (see
teleop_node.py declare_parameter calls), so you can override at launch without
editing this file. These are the defaults.
"""

# ── FR5 connection ────────────────────────────────────────────────────────────
FR5_IP = "192.168.58.2"          # Fairino controller IP
FR5_SERVO_VEL = 15.0             # ServoJ velocity % — start low, tune up
FR5_FILTER_T = 0.04              # ServoJ trajectory filter (s) — smooths motion

# ── control loop ──────────────────────────────────────────────────────────────
LOOP_HZ = 125.0                  # ServoJ must run 60–1000 Hz
JOINT_NAMES = ["j1", "j2", "j3", "j4", "j5", "j6"]   # match frcobot URDF

# ── Quest2ROS topics (published by the Quest app via ROS-TCP-Endpoint) ─────────
ACTIVE_HAND = "right"            # "right" | "left"
POSE_TOPIC = "/q2r_right_hand_pose"        # geometry_msgs/PoseStamped
INPUTS_TOPIC = "/q2r_right_hand_inputs"    # quest2ros/OVR2ROSInputs
HAPTIC_TOPIC = "/q2r_right_hand_haptic_feedback"  # quest2ros/OVR2ROSHapticFeedback

# ── clutch / inputs ───────────────────────────────────────────────────────────
# The arm only tracks while the clutch is engaged; otherwise it holds. Two modes:
#   "hold"   — engaged WHILE CLUTCH_FIELD (analog) >= CLUTCH_THRESHOLD. Safest for
#              a real cobot; default uses the grip (press_middle). Release = hold.
#   "toggle" — CLUTCH_FIELD is a bool button; each press flips engaged on/off.
#              This is the official Quest2ROS convention (button_lower) AND what the
#              SimulationInput "fake Quest" drives — set this to test WITHOUT a headset.
#
# OVR2ROSInputs fields: button_upper(B,bool) button_lower(A,bool)
#                       thumb_stick_horizontal/vertical press_index(trigger) press_middle(grip)
CLUTCH_MODE = "hold"             # "hold" | "toggle"
CLUTCH_FIELD = "press_middle"    # hold: press_middle/press_index ; toggle: button_lower/button_upper
CLUTCH_THRESHOLD = 0.6           # engaged when analog value >= this (hold mode)

# Gripper input:
#   "analog" — GRIPPER_FIELD (press_index trigger) hysteresis open/close
#   "toggle" — GRIPPER_FIELD is a bool button (button_upper); each press flips state
GRIPPER_MODE = "analog"          # "analog" | "toggle"
GRIPPER_FIELD = "press_index"

# ── delta-Cartesian mapping ───────────────────────────────────────────────────
# AXIS_MAP: Quest controller frame → FR5 base frame. MUST be a proper rotation
# (det = +1). Default is identity; CALIBRATE with the axis_check node and replace
# with the rows that make each controller axis move the expected FR5 base axis.
# Rows are FR5 axes, columns are Quest axes; use only -1/0/+1 entries.
AXIS_MAP = [
    [1.0, 0.0, 0.0],   # FR5 +X  ←  Quest ...
    [0.0, 1.0, 0.0],   # FR5 +Y  ←  Quest ...
    [0.0, 0.0, 1.0],   # FR5 +Z  ←  Quest ...
]

POSITION_SCALE = 1.0             # EEF mm per mm of hand travel (1.0 = 1:1)
ROTATION_SCALE = 1.0             # rotation multiplier (1.0 = 1:1)
CONTROL_ORIENTATION = True       # False = position-only (lock wrist orientation)

# FR5 TCP Euler convention used by GetActualTCPPose / GetInverseKinRef.
# Fairino reports rx,ry,rz; "xyz" (intrinsic, degrees) is the working default.
# If wrist orientation feels rotated, this is the first thing to check.
EULER_CONVENTION = "xyz"

# ── safety ────────────────────────────────────────────────────────────────────
# Max EEF offset from the clutch reference (per axis) — workspace guard.
MAX_DELTA_POS_MM = 500.0
MAX_DELTA_ROT_DEG = 90.0

# Per-joint rate limit (deg/cycle) for [J1..J6]. At 125 Hz, 0.16 ≈ 20 deg/s.
MAX_DELTA_PER_JOINT = [0.16, 0.12, 0.12, 0.30, 0.08, 0.20]

# Hard joint limits with margin, from GetJointSoftLimitDeg() on this controller.
FR5_JOINT_LIMITS = [
    (-170, 170),   # J1
    (-260,  80),   # J2
    (-155, 155),   # J3
    (-260,  80),   # J4
    (-170, 170),   # J5
    (-350, 350),   # J6
]

# ── DH AG-160-95 gripper (via Fairino SDK) ────────────────────────────────────
GRIPPER_ENABLE = True
GRIPPER_INDEX = 1
GRIPPER_TYPE = 0                 # 0 = parallel
GRIPPER_OPEN_PCT = 100
GRIPPER_CLOSE_PCT = 0
GRIPPER_VEL_PCT = 50
GRIPPER_FORCE_PCT = 50
GRIPPER_MAXTIME_MS = 5000
# Trigger (press_index) hysteresis → gripper state
GRIPPER_CLOSE_THRESHOLD = 0.7    # trigger >= → close
GRIPPER_OPEN_THRESHOLD = 0.2     # trigger <= → open

# Quest2ROS → Fairino FR5 Teleoperation (ROS2)

Real-time teleop of a **Fairino FR5** cobot using a **Meta Quest** controller via
the **[Quest2ROS](https://quest2ros.github.io/)** app. The controller's pose drives
the FR5 end-effector in Cartesian space (clutch-based delta → IK → ServoJ at
125 Hz), with RViz2 visualisation and optional DH gripper control.

This is an **independent rewrite**. The earlier WebXR attempt mapped controller
rotation incorrectly (it rotated only the rotation-vector *axis* instead of
conjugating the rotation into the robot frame), which made the end-effector tumble
unpredictably. Here the input comes from the purpose-built Quest2ROS app as clean
ROS-standard poses, and the rotation delta is conjugated properly
(`ΔR_base = M · ΔR_ctrl · Mᵀ`). The Quest→robot axis mapping is no longer guessed —
calibrate it empirically with the `axis_check` tool.

---

## Architecture

```
Meta Quest (Quest2ROS app)
   │  ROS-TCP (set PC IP + port in the app)
   ▼
ros_tcp_endpoint  ──►  /q2r_right_hand_pose   (geometry_msgs/PoseStamped)
                       /q2r_right_hand_inputs (quest2ros/OVR2ROSInputs)
   ▼
fr5_quest_teleop (this package)
   ├─ clutch-gated delta-Cartesian  (frame_transform.py)
   ├─ FR5 GetInverseKinRef → ServoJ  (fr5_driver.py, Fairino SDK)
   ├─ DH gripper from trigger        (gripper.py)
   └─ publishes /joint_states  ──►  robot_state_publisher ──► RViz2
```

Actuation goes through the **Fairino SDK** (ServoJ + IK), *not* ros2_control — the
frcobot ros2_control stack is sim-only on this hardware.

### Controller mapping (configurable — `CLUTCH_MODE` / `GRIPPER_MODE`)
Default (`hold` / `analog`) — recommended on the real cobot:
| Input (`OVR2ROSInputs`) | Physical | Role |
|---|---|---|
| `press_middle` | grip | **clutch** — hold to drive, release to hold (safest) |
| `press_index` | trigger | gripper open/close (analog hysteresis) |

`toggle` mode — matches the official Quest2ROS convention **and** the `SimulationInput`
fake-Quest, so you can test with no headset:
| Input | Physical | Role |
|---|---|---|
| `button_lower` | A | press to toggle tracking on/off (re-anchors on each engage) |
| `button_upper` | B | press to toggle gripper open/closed |

---

## Packages
| Package | Type | What |
|---|---|---|
| `quest2ros` | ament_cmake | `OVR2ROSInputs`, `OVR2ROSHapticFeedback` — package name **must** be `quest2ros` to match the app's registered type names |
| `fr5_quest_teleop` | ament_python | teleop node, axis_check node, launch file |

---

## Setup (Ubuntu 22.04 + ROS2 Humble)

### 1. Prerequisite workspace (FR5 URDF for RViz)
Clone the FR5 description into your ROS2 workspace so RViz has a robot model:
```bash
mkdir -p ~/ros2_teleop_ws/src && cd ~/ros2_teleop_ws/src
git clone https://github.com/FAIR-INNOVATION/frcobot_ros2.git
```

### 2. ROS-TCP-Endpoint (the bridge the Quest app talks to)
```bash
cd ~/ros2_teleop_ws/src
git clone -b ROS2 https://github.com/Unity-Technologies/ROS-TCP-Endpoint.git ros_tcp_endpoint
```

### 3. This package
```bash
cp -r /path/to/quest2ros2-fr5/src/quest2ros        ~/ros2_teleop_ws/src/
cp -r /path/to/quest2ros2-fr5/src/fr5_quest_teleop ~/ros2_teleop_ws/src/
# NOTE: if you also clone the official Quest2ROS2 repo, do NOT end up with two
# "quest2ros" message packages — use only one (they are identical).
```

### 4. Fairino SDK + python deps
```bash
pip install numpy scipy
pip install /path/to/fairino-*.whl     # vendor wheel, not on PyPI
```

### 5. Build
```bash
cd ~/ros2_teleop_ws
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
source install/setup.bash
```

### 6. Network
- Set the Linux NIC on the robot side to the `192.168.58.x/24` subnet (FR5 = `192.168.58.2`).
- Put the Quest **and** the Linux PC on the same Wi-Fi.
- Install the Quest2ROS app on the headset (see https://quest2ros.github.io/), then
  in the app set **IP = this PC's Wi-Fi IP** and **Port = the endpoint port** (default 10000).

---

## Test without a headset (recommended first)

The official Quest2ROS2 repo ships `SimulationInput` — a "fake Quest" that publishes
the exact same topics (it holds `button_lower` and toggles `button_upper`). It lets
you exercise the input → mapping → IK → robot path **without a headset**.

```bash
# 1) fake-Quest publishes /q2r_right_hand_{pose,inputs}  (a slow circle + button toggles)
ros2 run q2r2_bringup SimulationInput

# 2) teleop in toggle mode (matches the simulator's buttons)
ros2 launch fr5_quest_teleop teleop.launch.py \
    clutch_mode:=toggle clutch_field:=button_lower \
    gripper_mode:=toggle gripper_field:=button_upper servo_vel:=8.0
```
Press the simulator's `button_lower` equivalent (it's held in sim) → the FR5 should
trace the circle; `button_upper` toggles the gripper. RViz mirrors it via `/joint_states`.

> **Caveat — this still needs the FR5 connected.** Unlike the reference (which uses TF
> + a Cartesian controller), this node anchors on the robot's *actual* TCP pose and
> drives it over the Fairino SDK, so it connects to `FR5_IP` on startup. There is no
> robot-free dry-run yet; if you want one, that's a follow-up (mock driver + PyBullet IK).

> The `clutch_*` / `gripper_*` launch args mirror the config defaults. On the real
> headset, omit them to get the safe `hold`/`analog` defaults (grip = clutch).

## Running (separate terminals, each after `source install/setup.bash`)

**T1 — TCP endpoint** (Quest connects here):
```bash
ros2 launch ros_tcp_endpoint endpoint.py ROS_IP:=0.0.0.0
```
Confirm topics appear once the app connects:
```bash
ros2 topic echo /q2r_right_hand_pose --once
```

**T2 — calibrate the axis map (do this once, no robot motion):**
```bash
ros2 run fr5_quest_teleop axis_check
```
Hold grip, move the controller ~20 cm along one direction at a time, and read the
output. Edit `AXIS_MAP` in `fr5_quest_teleop/config.py` until:
push-away → FR5 **+X**, move-right → FR5 **+Y**, move-up → FR5 **+Z** (adjust to your
mounting). Keep entries to `-1/0/+1`; the tool warns if the matrix isn't a proper
rotation. Rebuild after editing (`colcon build`).

**T3 — teleop + RViz** (escalate carefully):
```bash
# position-only first — safest, isolates the position mapping:
ros2 launch fr5_quest_teleop teleop.launch.py control_orientation:=false servo_vel:=8.0

# then full 6-DOF once position tracking looks right:
ros2 launch fr5_quest_teleop teleop.launch.py control_orientation:=true servo_vel:=15.0
```
In RViz: add **RobotModel** (Description Topic `/robot_description`) and set **Fixed
Frame** to `base_link`. The model tracks the live robot via `/joint_states`.

**Drive:** hold **grip** → arm tracks your hand; release → arm holds; re-grip
anywhere to reposition without moving the robot. **Trigger** = gripper.
**Ctrl-C** = clean stop (always; never `kill -9` — the FR5 CNDE session needs the
clean shutdown frame or it blocks new connections for ~30 s).

---

## Tuning (`fr5_quest_teleop/config.py`, or launch args)
| Param | Default | Notes |
|---|---|---|
| `AXIS_MAP` | identity | **calibrate with `axis_check`** — the #1 thing to get right |
| `POSITION_SCALE` | 1.0 | EEF mm per mm of hand travel (1.0 = 1:1) |
| `ROTATION_SCALE` | 1.0 | rotation multiplier |
| `CONTROL_ORIENTATION` | true | false = lock wrist, position-only |
| `EULER_CONVENTION` | "xyz" | FR5 TCP angle convention — flip here if wrist feels rotated |
| `MAX_DELTA_PER_JOINT` | see file | per-joint rate cap (deg/cycle @125 Hz) |
| `FR5_SERVO_VEL` | 15 | ServoJ velocity % — start low |

---

## Troubleshooting
- **No `/q2r_*` topics:** Quest app not connected — check the IP/port in the app match
  this PC and the endpoint, same Wi-Fi, firewall open on the endpoint port.
- **EEF moves in the wrong direction:** fix `AXIS_MAP` via `axis_check`.
- **Wrist orientation is off but position is fine:** try a different `EULER_CONVENTION`
  (e.g. `"ZYX"`), or run with `control_orientation:=false` to confirm position is good.
- **`GetInverseKinRef failed ... unreachable`:** target outside workspace — reduce
  `POSITION_SCALE` or re-grip closer to a reachable region.
- **`fairino` import error:** SDK wheel not installed (Linux only; cannot run on macOS).

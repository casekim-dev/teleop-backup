# Quest Teleop Bridge Notes

This workspace receives Quest/WebXR hand/head pose data and publishes ROS2 commands for robot arm, waist, head, and hand control.

The current main runtime is:

```bash
python3 finger1.py
```

The Quest browser page is:

```bash
index.html
```

Serve it from this directory as usual:

```bash
python3 -m http.server 8012
```

The WebSocket receiver inside `finger1.py` listens on:

```text
0.0.0.0:8765
```

`index.html` currently connects to:

```text
ws://192.168.50.3:8765
```

Update that IP in `index.html` if the teleop PC IP changes.

## Main Files

### `finger1.py`

Integrated Quest teleop runtime.

It handles:

- WebSocket JSON receive from Quest/WebXR
- foot pedal clutch input
- left/right arm twist control
- head and waist control
- Quest hand tracking to IGRIS hand targets
- debug logging for arm orientation commands

Run:

```bash
python3 finger1.py
```

Important ROS publishers:

```text
/left_servo/left_servo/delta_twist_cmds     geometry_msgs/msg/TwistStamped
/right_servo/right_servo/delta_twist_cmds   geometry_msgs/msg/TwistStamped
/head_controller/joint_trajectory           trajectory_msgs/msg/JointTrajectory
/waist_delta_cmds                           trajectory_msgs/msg/JointTrajectory
/igris_c/hand/targets                       std_msgs/msg/Float32MultiArray
/igris/hand/status                          sensor_msgs/msg/JointState
```

Important ROS subscribers:

```text
/igris/hand/joint_states                    sensor_msgs/msg/JointState
/igris_c/hand/status                        std_msgs/msg/String
```

TF frames used by arm control are parameterized:

```text
base_frame      default: base_link
left_ee_frame   default: Left_Hand
right_ee_frame  default: Right_Hand
```

Run with different TF names if needed:

```bash
python3 finger1.py --ros-args \
  -p base_frame:=base_link \
  -p left_ee_frame:=Left_Hand \
  -p right_ee_frame:=Right_Hand
```

If TF is missing, arm control cannot capture robot EE zero. Check:

```bash
ros2 topic list | grep tf
ros2 topic echo /tf --once
ros2 topic echo /tf_static --once
```

### `index.html`

Operational Quest/WebXR page.

It uses WebXR hand tracking, not Unity C# APIs.

Hand pose source:

```javascript
const referenceSpace = renderer.xr.getReferenceSpace();
const wristPose = frame.getJointPose(hand.get('wrist'), referenceSpace);
```

The pose sent for each hand is the WebXR `wrist` joint pose in the WebXR reference space:

```javascript
pos: wristPose.transform.position
rot: wristPose.transform.orientation
```

This is not `transform.localPosition` or `transform.localRotation` from Unity. It is WebXR pose relative to the current XR reference space.

### `quest_handcmd_bridge.cpp`

ROS2-to-DDS bridge for the physical IGRIS hand.

It subscribes:

```text
/igris_c/hand/targets  std_msgs/msg/Float32MultiArray
```

It publishes DDS:

```text
rt/handcmd
```

Default motor ID order:

```text
[11, 12, 13, 14, 15, 16, 21, 22, 23, 24, 25, 26]
```

Meaning:

```text
11 right thumb bend
12 right index
13 right middle
14 right ring
15 right pinky
16 right thumb rotation/spread
21 left thumb bend
22 left index
23 left middle
24 left ring
25 left pinky
26 left thumb rotation/spread
```

### `finger_only.py`

Hand-only test node. Use this when debugging fingers without arm/waist motion.

### `orientation_axis_probe.py`

Direct arm angular-axis probe. This bypasses Quest and sends pure angular twist to the servo topic.

Use this to determine what robot motion each command axis actually produces:

```bash
python3 orientation_axis_probe.py --side left --axis x --value 0.15 --duration 0.5
python3 orientation_axis_probe.py --side left --axis y --value 0.15 --duration 0.5
python3 orientation_axis_probe.py --side left --axis z --value 0.15 --duration 0.5
```

Right arm:

```bash
python3 orientation_axis_probe.py --side right --axis x --value 0.15 --duration 0.5
```

This publishes pure angular commands:

```text
[linear_x, linear_y, linear_z, angular_x, angular_y, angular_z]
[0,        0,        0,        wx,        0,         0]
[0,        0,        0,        0,         wy,        0]
[0,        0,        0,        0,         0,         wz]
```

## Data Flow

```text
Quest WebXR index.html
  -> WebSocket ws://teleop_pc:8765
  -> finger1.py
  -> arm TwistStamped topics
  -> waist/head JointTrajectory topics
  -> /igris_c/hand/targets
  -> quest_handcmd_bridge.cpp
  -> DDS rt/handcmd
  -> robot hand controller
```

## Arm Control Details

`finger1.py` receives WebXR wrist pose for each hand.

Position conversion from WebXR to ROS-like basis:

```python
ros_x = -vr_z
ros_y = -vr_x
ros_z =  vr_y
```

Arm linear command is P-control velocity in `base_frame`:

```text
linear unit: m/s
```

On clutch rising edge:

1. `finger1.py` looks up current robot EE TF in `base_frame`.
2. It stores robot EE zero.
3. It stores current Quest/WebXR wrist pose as human zero.
4. While clutch is held, target robot position is:

```text
robot_zero + (current_vr_pos - human_zero)
```

Then command is:

```text
linear = (target_robot_pos - current_robot_pos) * p_gain
```

Current `p_gain` is `4.0`.

### Body-Yaw Coupled Arm Target

Current behavior is intended to be body-relative, not fixed world-relative.

When the clutch is pressed, `finger1.py` stores:

```text
arm_zero_head_yaw[side] = latest_head_yaw
```

While the clutch is held, it computes:

```python
body_yaw_delta = -(latest_head_yaw - arm_zero_head_yaw[side])
```

The arm position target is then rotated in the base XY plane by `body_yaw_delta` before P-control. This means:

```text
hand already extended forward + user/waist turns 90 deg
  -> robot arm target should rotate with the body/waist instead of staying at the old world direction
```

Switches:

```text
rotate_arm_with_head_yaw          default True
rotate_orientation_with_head_yaw  default True
```

If the arm follows the body yaw in the wrong direction, first try flipping the sign of `body_yaw_delta` in `process_arm()`.

## Arm Orientation Details

Current WebXR wrist quaternion path:

```text
q_vr_wrist
  -> map_vr_to_ros_quat()
  -> q_ros_wrist
  -> apply_wrist_to_ee_offset()
  -> q_robot_target
```

When `rotate_orientation_with_head_yaw` is enabled, the same body yaw delta used for arm position is also applied to orientation before angular velocity is computed:

```python
q_body_yaw = quat_from_rpy(0.0, 0.0, body_yaw_delta)
target_rot = q_body_yaw * q_robot_target
```

This is the current intended behavior: wrist/EE orientation should rotate with the user's body/waist heading, not remain locked to the old world heading. Fine tuning is still expected around wrist-to-EE offset and sign conventions.

Angular velocity is computed in base/world frame:

```text
q_delta = q_curr * inverse(q_prev)
angular = rotvec(q_delta) / dt
```

Then published directly as:

```text
TwistStamped.twist.angular.x/y/z
angular unit: rad/s
frame_id: base_frame
```

Current angular gain is:

```python
a_gain = 2.0
```

Orientation offset parameters:

```text
wrist_to_ee_roll   default 0.0 rad
wrist_to_ee_pitch  default 0.0 rad
wrist_to_ee_yaw    default 0.0 rad
wrist_to_ee_order  default post
```

Default multiplication:

```text
q_robot_target = q_ros_wrist * q_offset_wrist_to_ee
```

If `wrist_to_ee_order:=pre`:

```text
q_robot_target = q_offset_wrist_to_ee * q_ros_wrist
```

Use `orientation_axis_probe.py` and RViz/robot observation to determine the correct wrist-to-EE offset.

## Orientation Debug Log

`finger1.py` currently logs arm orientation commands at 10 Hz:

```text
arm_orientation ang_base L=[wx, wy, wz] R=[wx, wy, wz] dom L/R=rot_about_base_y+(...) / ... mode L/R=... clutch L/R=...
```

Meaning:

```text
wx = angular velocity about base_frame X axis
wy = angular velocity about base_frame Y axis
wz = angular velocity about base_frame Z axis
```

This is not hand translation direction. It is rotation about the robot/base frame axes.

To test axis mapping:

1. Use only one hand/clutch at a time.
2. Keep hand position fixed as much as possible.
3. Rotate wrist in one intended direction only.
4. Check which `rot_about_base_*` axis dominates.
5. Compare against `orientation_axis_probe.py` direct robot-axis behavior.

If the dominant axis is consistent but wrong, tune `wrist_to_ee_roll/pitch/yaw` or `wrist_to_ee_order`.

## Waist and Head

Waist command topic:

```text
/waist_delta_cmds
```

Waist behavior:

- clutch-gated
- no initial 3-second zero capture
- uses delta from previous head pose while clutch is held
- yaw and pitch only
- roll fixed to `0.0`
- yaw/pitch signs are inverted at publish time

Current waist output order:

```python
[-yaw, 0.0, -pitch]
```

## Finger Control

Finger control is also clutch-gated by side.

If left clutch is off, left hand targets are not updated from VR.
If right clutch is off, right hand targets are not updated from VR.

Target array order published to `/igris_c/hand/targets`:

```text
[ R_thumb_bend,
  R_index,
  R_middle,
  R_ring,
  R_pinky,
  R_thumb_spread,
  L_thumb_bend,
  L_index,
  L_middle,
  L_ring,
  L_pinky,
  L_thumb_spread ]
```

## Common Issues

### No arm motion, TF warning

Example:

```text
TF lookup failed for base_link->Left_Hand
```

Check:

```bash
ros2 topic list | grep tf
ros2 topic echo /tf --once
```

Make sure robot bringup/TF publisher is running in the same ROS domain.

### Quest connects but no XR data

Check the IP in `index.html`:

```javascript
const socket = new WebSocket('ws://192.168.50.3:8765');
```

Check that `finger1.py` is running and listening on port 8765.

Useful checks:

```bash
ps -ef | grep 'python3 finger1.py'
ss -tanp | grep ':8765'
ROS_DOMAIN_ID=94 ros2 node list | grep finger_footpose_node
```

If `ss` shows `ESTABLISHED` from the Quest IP, the WebSocket connection is alive. If debug logs show `ws_count` increasing and `xr L/R=True`, data is arriving.

### Hand moves but robot fingers do not

Make sure the DDS hand bridge is running and subscribed to `/igris_c/hand/targets`.

Check:

```bash
ros2 topic info /igris_c/hand/targets -v
```

## GitHub Backup

This directory is backed up to:

```text
https://github.com/casekim-dev/teleop-backup
```

Current important documentation files:

```text
README.md
TELEOP_BACKUP_NOTES.md
```

## 2026-05-29 Handoff Notes

Latest tested direction:

- Quest/WebXR connection recovered and `finger1.py` can receive wrist/head/hand data over WebSocket `8765`.
- Arm translation now follows the user's current body/head yaw: if the user turns and then reaches forward, the robot reaches forward relative to the turned body.
- Additional fix added after testing: if the user is already reaching forward and then rotates the body/waist, the arm target is rotated with the body yaw instead of remaining at the old world direction.
- Orientation now applies the same body yaw delta before angular velocity calculation. This is directionally correct but still needs fine tuning.
- Remaining tuning knobs are mainly `body_yaw_delta` sign, `wrist_to_ee_roll/pitch/yaw`, `wrist_to_ee_order`, and angular gain `a_gain`.

Important runtime reminder:

- Edits to `finger1.py` do not affect an already-running process. Restart `python3 finger1.py` after code changes.
- Normal Quest page serving remains `python3 -m http.server 8012`; do not switch to the old HTTPS experiments unless explicitly needed.
- For physical hand motion, `quest_handcmd_bridge` must also be running; otherwise `/igris_c/hand/targets` may have publisher but no subscriber.

Quick validation sequence for the next session:

1. Start/confirm robot TF and IK side.
2. Serve the page: `python3 -m http.server 8012`.
3. Run bridge: `python3 finger1.py`.
4. Confirm WebSocket: `ss -tanp | grep ':8765'` should show Quest IP `ESTABLISHED`.
5. Press clutch and test arm translation with head/body yaw at 0 deg and 90 deg.
6. Test holding the arm extended while rotating the body/waist; arm target should rotate with the body.
7. Test wrist orientation after body yaw; expect correct qualitative direction, but tune offsets/gains if axes feel rotated or scaled.

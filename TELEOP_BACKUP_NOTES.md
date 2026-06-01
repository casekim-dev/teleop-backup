# Quest Teleop Backup Notes

Date: 2026-05-28
Workspace: `/home/casekim/teleop`

## Current Working Setup

The current integrated runtime file is `finger1.py`.
It combines:

- Quest WebXR pose input over WebSocket `0.0.0.0:8765`
- foot pedal clutch input from `/dev/input/by-id/usb-PCsensor_FootSwitch-event-kbd`
- left/right arm Cartesian velocity control
- head/waist control
- Quest hand tracking to IGRIS hand targets

The Quest page is `index.html`. Serve it separately with the existing HTTP flow:

```bash
python3 -m http.server 8012
```

Do not use the calibration/instruction page for normal operation. `index_joint_dof.html` is only for hand DOF calibration/debugging.

## Main Runtime Commands

Run the integrated Python bridge:

```bash
python3 finger1.py
```

Run the ROS-to-DDS hand bridge separately so `/igris_c/hand/targets` reaches the robot hand:

```bash
quest_handcmd_bridge
```

If launching the built binary directly, use the installed/built executable from the ROS workspace as appropriate.

## Arm Control

`finger1.py` publishes arm commands:

- `/left_servo/left_servo/delta_twist_cmds`
- `/right_servo/right_servo/delta_twist_cmds`

Arm behavior:

- side-specific clutch gates control
- on clutch rising edge, robot EE zero is captured from TF
- VR hand zero is captured at the same time
- linear output is P-control velocity in `base_link`, unit `m/s`
- angular output is quaternion-differentiated velocity, unit `rad/s`

Frames:

- left EE frame: `Left_Hand`
- right EE frame: `Right_Hand`
- TF lookup target frame: `base_link`

## Waist and Head Control

Head is still published to:

- `/head_controller/joint_trajectory`

Waist is published to:

- `/waist_delta_cmds`

Waist behavior:

- clutch-gated like the arms
- no initial 3 second zero capture
- delta is based on the previous head pose while clutch is held
- yaw and pitch only
- roll is fixed to `0.0`
- yaw and pitch signs are inverted at publish time because the robot direction was reversed

Current waist output order:

```python
[float(-yaw), 0.0, float(-pitch)]
```

## Finger Control

Quest hand tracking is integrated into `finger1.py` and publishes:

- `/igris_c/hand/targets` (`std_msgs/msg/Float32MultiArray`)

Finger behavior:

- side-specific clutch gated, same as arms
- if left clutch is off, left hand targets are not updated from VR
- if right clutch is off, right hand targets are not updated from VR
- WebXR 25 hand joints are used when present
- fallback distance-based mapping remains only for missing joint data
- 1 Hz debug log prints target values, WebXR finger/joint visibility, thumb bend/spread, and retarget mode

The 12 target values are ordered for the DDS hand motor IDs:

| Index | Motor ID | Meaning |
| --- | ---: | --- |
| 0 | 11 | right thumb bend |
| 1 | 12 | right index |
| 2 | 13 | right middle |
| 3 | 14 | right ring |
| 4 | 15 | right pinky |
| 5 | 16 | right thumb rotation/spread |
| 6 | 21 | left thumb bend |
| 7 | 22 | left index |
| 8 | 23 | left middle |
| 9 | 24 | left ring |
| 10 | 25 | left pinky |
| 11 | 26 | left thumb rotation/spread |

Current hand retarget parameters in `finger1.py`:

- `thumb_bend_open_deg = 166.0`
- `thumb_bend_closed_deg = 122.0`
- `thumb_spread_open_deg = -38.0`
- `thumb_spread_closed_deg = -14.0`
- `finger_open_deg = 172.0`
- `finger_closed_deg = 105.0`
- `thumb_bend_gain = 1.0`
- `thumb_spread_gain = 1.0`

## DDS Hand Bridge

`quest_handcmd_bridge.cpp` subscribes:

- `/igris_c/hand/targets`

It publishes DDS:

- `rt/handcmd`

It also listens for diagnostics/status:

- `rt/handstate`
- `rt/service/hand_init/response`

Default motor IDs:

```cpp
{11, 12, 13, 14, 15, 16, 21, 22, 23, 24, 25, 26}
```

Useful debug features:

- `HAND_MOTOR_IDS=...` overrides the 12 motor IDs
- `HAND_ID_PROBE=1` enables one-ID-at-a-time probing
- `HAND_ID_PROBE_IDS=16,26` limits probe IDs
- `HAND_ID_PROBE_Q=0.35` changes probe command value

## Finger Debug Files

`finger_only.py` is the isolated hand-only test node. It was used to validate Quest finger detection and `/igris_c/hand/targets` publishing without arms/waist.

`spread_channel_test.py` directly publishes spread/bend channels and confirmed that the physical thumb spread channels and DDS bridge path can work.

`hand_dof_probe.py` and `index_joint_dof.html` were used to collect WebXR joint data and derive the current joint-based mapping.

## Verified Checks

The integrated file was syntax checked:

```bash
python3 -m py_compile finger1.py
```

Recent functional status from testing:

- arms work
- waist works after yaw/pitch sign inversion
- fingers work in isolated test
- fingers are now integrated with arm control in `finger1.py`
- fingers are now clutch-gated like arms

## Files Worth Backing Up

Core runtime and bridge:

- `finger1.py`
- `finger_only.py`
- `quest_handcmd_bridge.cpp`
- `index.html`
- `index_joint_dof.html`
- `spread_channel_test.py`
- `hand_dof_probe.py`
- `CMakeLists.txt`
- `package.xml`

Important supporting/legacy files:

- `delta_footpose.py`
- `foot_purepose.py`
- `purepose.py`
- `debug_pedal.py`
- `common_arm_ik.py`
- `ik_solver.py`
- `cyclonedds_eno1.xml`

Excluded from git backup:

- `build/`, `install/`, `log/`
- SDK/vendor clones under `igris_c_sdk_public/`, `igris_sdk_robot/`, `televuer/`
- local TLS/key files such as `*.pem`

## 2026-05-29 Update: Body-Relative Arm and Orientation

Today the remaining body-relative behavior was addressed in `finger1.py`.

Observed issue:

- Reaching forward after turning the head/body worked.
- But if the hand was already extended forward and the user rotated from yaw 0 deg to yaw 90 deg, the robot arm target stayed near the old world direction instead of rotating with the body.

Implemented behavior:

- On clutch rising edge, the node stores `arm_zero_head_yaw[side]`.
- While clutch is held, it computes `body_yaw_delta = -(latest_head_yaw - arm_zero_head_yaw[side])`.
- Arm XY target is rotated by this body yaw delta before P-control.
- Orientation target is also pre-multiplied by `q_body_yaw` before angular velocity calculation.

New/important parameters:

- `rotate_arm_with_head_yaw = True`
- `rotate_orientation_with_head_yaw = True`
- `wrist_to_ee_roll/pitch/yaw` and `wrist_to_ee_order` remain the main orientation tuning knobs.

Current status:

- Directional behavior appears correct.
- Fine tuning is still needed for orientation feel and possibly sign/gain.
- If body yaw coupling is reversed, flip the sign of `body_yaw_delta` in `process_arm()`.

Next-session checks:

```bash
python3 -m http.server 8012
python3 finger1.py
ss -tanp | grep ':8765'
ROS_DOMAIN_ID=94 ros2 topic info /left_servo/left_servo/delta_twist_cmds -v
ROS_DOMAIN_ID=94 ros2 topic info /right_servo/right_servo/delta_twist_cmds -v
```

For physical fingers, also run/check `quest_handcmd_bridge`; `/igris_c/hand/targets` needs a subscriber.

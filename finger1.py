import asyncio
import json
import math
import os
import struct
import threading
import time

import rclpy
from geometry_msgs.msg import TwistStamped
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float32MultiArray, String
from tf2_ros import Buffer, TransformListener
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
import websockets


shared_xr_data = {"head": None, "left": None, "right": None}
shared_ws_count = 0
shared_ws_last_time = 0.0
shared_foot_clutch = {"left": False, "right": False}


def foot_pedal_monitor(node_logger):
    global shared_foot_clutch
    import fcntl

    device_path = "/dev/input/by-id/usb-PCsensor_FootSwitch-event-kbd"

    if not os.path.exists(device_path):
        node_logger.error(f"Foot pedal device not found: {device_path}")
        return

    event_format = "QQHHi"
    event_size = struct.calcsize(event_format)
    EVIOCGRAB = 0x40044590

    node_logger.info(f"Foot pedal monitor started: {device_path}")

    try:
        with open(device_path, "rb") as f:
            try:
                fcntl.ioctl(f.fileno(), EVIOCGRAB, 1)
                node_logger.info("Foot pedal grabbed exclusively")
            except Exception as e:
                node_logger.warn(f"Failed to grab foot pedal exclusively: {e}")

            while True:
                data = f.read(event_size)
                if not data:
                    break

                _, _, ev_type, ev_code, ev_value = struct.unpack(event_format, data)

                if ev_type == 1:
                    is_pressed = ev_value > 0
                    old_left = shared_foot_clutch["left"]
                    old_right = shared_foot_clutch["right"]

                    if ev_code == 30:
                        shared_foot_clutch["left"] = is_pressed
                    elif ev_code == 46:
                        shared_foot_clutch["right"] = is_pressed
                    elif ev_code == 48:
                        shared_foot_clutch["left"] = is_pressed
                        shared_foot_clutch["right"] = is_pressed

                    if old_left != shared_foot_clutch["left"] or old_right != shared_foot_clutch["right"]:
                        node_logger.info(
                            f"Clutch State: L={shared_foot_clutch['left']}, "
                            f"R={shared_foot_clutch['right']}"
                        )

    except PermissionError:
        node_logger.error("Permission denied. Run with sudo to read the foot pedal.")
    except Exception as e:
        node_logger.error(f"Foot pedal error: {e}")


async def ws_handler(websocket):
    global shared_xr_data, shared_ws_count, shared_ws_last_time
    try:
        async for message in websocket:
            shared_xr_data = json.loads(message)
            shared_ws_count += 1
            shared_ws_last_time = time.time()
    except websockets.exceptions.ConnectionClosed:
        pass


async def main_ws():
    async with websockets.serve(ws_handler, "0.0.0.0", 8765):
        await asyncio.Future()


def start_ws_server():
    asyncio.run(main_ws())


class FingerFootPoseNode(Node):
    def __init__(self):
        super().__init__("finger_footpose_node")
        self.dt = 0.02

        self.prev_clutch = {"left": False, "right": False}
        self.human_zero = {"left": None, "right": None}
        self.rot_zero = {"left": None, "right": None}
        self.robot_zero = {"left": None, "right": None}
        self.prev_state = {"left": None, "right": None}
        self.arm_zero_head_yaw = {"left": 0.0, "right": 0.0}
        self.latest_head_yaw = 0.0
        self.rotate_arm_with_head_yaw = bool(self.declare_parameter("rotate_arm_with_head_yaw", True).value)
        self.rotate_orientation_with_head_yaw = bool(
            self.declare_parameter("rotate_orientation_with_head_yaw", True).value
        )
        self.last_arm_cmd = {
            "left": {"linear": (0.0, 0.0, 0.0), "angular": (0.0, 0.0, 0.0), "clutch": False, "mode": "init"},
            "right": {"linear": (0.0, 0.0, 0.0), "angular": (0.0, 0.0, 0.0), "clutch": False, "mode": "init"},
        }

        self.base_frame = str(self.declare_parameter("base_frame", "base_link").value)
        self.left_ee_frame = str(self.declare_parameter("left_ee_frame", "Left_Hand").value)
        self.right_ee_frame = str(self.declare_parameter("right_ee_frame", "Right_Hand").value)

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.left_pub = self.create_publisher(TwistStamped, "/left_servo/left_servo/delta_twist_cmds", 10)
        self.right_pub = self.create_publisher(TwistStamped, "/right_servo/right_servo/delta_twist_cmds", 10)
        self.head_pub = self.create_publisher(JointTrajectory, "/head_controller/joint_trajectory", 10)
        self.waist_pub = self.create_publisher(JointTrajectory, "/waist_delta_cmds", 10)
        self.left_hand_pub = self.create_publisher(JointTrajectory, "/left_hand_controller/joint_trajectory", 10)
        self.right_hand_pub = self.create_publisher(JointTrajectory, "/right_hand_controller/joint_trajectory", 10)
        self.hand_target_pub = self.create_publisher(Float32MultiArray, "/igris_c/hand/targets", 10)
        self.joint_state_pub = self.create_publisher(JointState, "/igris/hand/status", 10)
        self.teleop_sub = self.create_subscription(
            JointState,
            "/igris/hand/joint_states",
            self.teleop_callback,
            10,
        )
        self.status_sub = self.create_subscription(
            String,
            "/igris_c/hand/status",
            self.status_callback,
            10,
        )

        self.finger_lock = threading.Lock()
        self.fing_th_L = 0.0
        self.fing_all_L = 0.0
        self.fing_th_R = 0.0
        self.fing_all_R = 0.0
        self.gripper_L = 0.0
        self.gripper_R = 0.0
        self.right_targets = {
            "thumb_up": 0.0,
            "index": 0.0,
            "middle": 0.0,
            "ring": 0.0,
            "pinky": 0.0,
            "thumb_down": 0.0,
        }
        self.left_targets = {
            "thumb_up": 0.0,
            "index": 0.0,
            "middle": 0.0,
            "ring": 0.0,
            "pinky": 0.0,
            "thumb_down": 0.0,
        }
        self.last_targets = [0.0] * 12
        self.target_publish_count = 0
        self.last_thumb_features = {"left": {}, "right": {}}
        self.last_retarget_mode = {"left": "none", "right": "none"}
        self.thumb_spread_hold = {"left": None, "right": None}

        self.thumb_bend_gain = float(self.declare_parameter("thumb_bend_gain", 1.0).value)
        self.thumb_spread_gain = float(self.declare_parameter("thumb_spread_gain", 1.0).value)
        self.thumb_right_spread_invert = bool(self.declare_parameter("thumb_right_spread_invert", False).value)
        self.thumb_left_spread_invert = bool(self.declare_parameter("thumb_left_spread_invert", False).value)
        self.force_thumb_spread = float(self.declare_parameter("force_thumb_spread", -1.0).value)
        self.thumb_bend_open_deg = float(self.declare_parameter("thumb_bend_open_deg", 166.0).value)
        self.thumb_bend_closed_deg = float(self.declare_parameter("thumb_bend_closed_deg", 122.0).value)
        self.thumb_spread_open_deg = float(self.declare_parameter("thumb_spread_open_deg", -38.0).value)
        self.thumb_spread_closed_deg = float(self.declare_parameter("thumb_spread_closed_deg", -14.0).value)
        self.finger_open_deg = float(self.declare_parameter("finger_open_deg", 172.0).value)
        self.finger_closed_deg = float(self.declare_parameter("finger_closed_deg", 105.0).value)

        self.wrist_to_ee_roll = float(self.declare_parameter("wrist_to_ee_roll", 0.0).value)
        self.wrist_to_ee_pitch = float(self.declare_parameter("wrist_to_ee_pitch", 0.0).value)
        self.wrist_to_ee_yaw = float(self.declare_parameter("wrist_to_ee_yaw", 0.0).value)
        self.wrist_to_ee_order = str(self.declare_parameter("wrist_to_ee_order", "post").value)
        self.wrist_to_ee_offset = self.quat_from_rpy(
            self.wrist_to_ee_roll,
            self.wrist_to_ee_pitch,
            self.wrist_to_ee_yaw,
        )

        self.joint_names = [
            "Fing_all_R",
            "joint_2",
            "joint_3",
            "joint_4",
            "joint_5",
            "Fing_th_R",
            "Fing_all_L",
            "joint_8",
            "joint_9",
            "joint_10",
            "joint_11",
            "Fing_th_L",
        ]

        self.prev_waist_clutch = False
        self.prev_waist_state = None

        self.timer = self.create_timer(self.dt, self.timer_callback)
        self.debug_timer = self.create_timer(0.1, self.debug_callback)
        self.get_logger().info("Finger foot pose node started")
        self.get_logger().info("WebSocket input: 0.0.0.0:8765")
        self.get_logger().info("Hand bridge: /igris/hand/joint_states -> /igris_c/hand/targets")
        self.get_logger().info("Hand status: /igris_c/hand/status -> /igris/hand/status")

    def map_vr_to_ros(self, pos):
        return {"x": -pos["z"], "y": -pos["x"], "z": pos["y"]}

    def map_vr_rotvec_to_ros(self, rotvec):
        return {"x": -rotvec["z"], "y": -rotvec["x"], "z": rotvec["y"]}

    def normalize_quat(self, q):
        norm = math.sqrt(q["x"] ** 2 + q["y"] ** 2 + q["z"] ** 2 + q["w"] ** 2)
        if norm < 1e-9:
            return {"x": 0.0, "y": 0.0, "z": 0.0, "w": 1.0}
        return {k: q[k] / norm for k in ("x", "y", "z", "w")}

    def quat_inverse(self, q):
        q = self.normalize_quat(q)
        return {"x": -q["x"], "y": -q["y"], "z": -q["z"], "w": q["w"]}

    def quat_multiply(self, a, b):
        return {
            "x": a["w"] * b["x"] + a["x"] * b["w"] + a["y"] * b["z"] - a["z"] * b["y"],
            "y": a["w"] * b["y"] - a["x"] * b["z"] + a["y"] * b["w"] + a["z"] * b["x"],
            "z": a["w"] * b["z"] + a["x"] * b["y"] - a["y"] * b["x"] + a["z"] * b["w"],
            "w": a["w"] * b["w"] - a["x"] * b["x"] - a["y"] * b["y"] - a["z"] * b["z"],
        }

    def quat_delta_to_rotvec(self, q_zero, q_current):
        q_zero = self.normalize_quat(q_zero)
        q_current = self.normalize_quat(q_current)

        if q_zero["x"] * q_current["x"] + q_zero["y"] * q_current["y"] + q_zero["z"] * q_current["z"] + q_zero["w"] * q_current["w"] < 0.0:
            q_current = {k: -q_current[k] for k in ("x", "y", "z", "w")}

        q_delta = self.normalize_quat(self.quat_multiply(q_current, self.quat_inverse(q_zero)))
        v_norm = math.sqrt(q_delta["x"] ** 2 + q_delta["y"] ** 2 + q_delta["z"] ** 2)

        if v_norm < 1e-9:
            return {"x": 0.0, "y": 0.0, "z": 0.0}

        angle = 2.0 * math.atan2(v_norm, q_delta["w"])
        if angle > math.pi:
            angle -= 2.0 * math.pi
        elif angle < -math.pi:
            angle += 2.0 * math.pi

        scale = angle / v_norm
        return {"x": q_delta["x"] * scale, "y": q_delta["y"] * scale, "z": q_delta["z"] * scale}

    def euler_from_quaternion(self, q):
        x, y, z, w = q["x"], q["y"], q["z"], q["w"]
        t0 = 2.0 * (w * x + y * z)
        t1 = 1.0 - 2.0 * (x * x + y * y)
        roll = math.atan2(t0, t1)
        t2 = 2.0 * (w * y - z * x)
        t2 = max(-1.0, min(1.0, t2))
        pitch = math.asin(t2)
        t3 = 2.0 * (w * z + x * y)
        t4 = 1.0 - 2.0 * (y * y + z * z)
        yaw = math.atan2(t3, t4)
        return roll, pitch, yaw

    def quat_to_matrix(self, q):
        q = self.normalize_quat(q)
        x, y, z, w = q["x"], q["y"], q["z"], q["w"]
        return [
            [1.0 - 2.0 * (y * y + z * z), 2.0 * (x * y - z * w), 2.0 * (x * z + y * w)],
            [2.0 * (x * y + z * w), 1.0 - 2.0 * (x * x + z * z), 2.0 * (y * z - x * w)],
            [2.0 * (x * z - y * w), 2.0 * (y * z + x * w), 1.0 - 2.0 * (x * x + y * y)],
        ]

    def matrix_to_quat(self, m):
        trace = m[0][0] + m[1][1] + m[2][2]
        if trace > 0.0:
            s = math.sqrt(trace + 1.0) * 2.0
            return self.normalize_quat({
                "w": 0.25 * s,
                "x": (m[2][1] - m[1][2]) / s,
                "y": (m[0][2] - m[2][0]) / s,
                "z": (m[1][0] - m[0][1]) / s,
            })
        if m[0][0] > m[1][1] and m[0][0] > m[2][2]:
            s = math.sqrt(1.0 + m[0][0] - m[1][1] - m[2][2]) * 2.0
            return self.normalize_quat({
                "w": (m[2][1] - m[1][2]) / s,
                "x": 0.25 * s,
                "y": (m[0][1] + m[1][0]) / s,
                "z": (m[0][2] + m[2][0]) / s,
            })
        if m[1][1] > m[2][2]:
            s = math.sqrt(1.0 + m[1][1] - m[0][0] - m[2][2]) * 2.0
            return self.normalize_quat({
                "w": (m[0][2] - m[2][0]) / s,
                "x": (m[0][1] + m[1][0]) / s,
                "y": 0.25 * s,
                "z": (m[1][2] + m[2][1]) / s,
            })
        s = math.sqrt(1.0 + m[2][2] - m[0][0] - m[1][1]) * 2.0
        return self.normalize_quat({
            "w": (m[1][0] - m[0][1]) / s,
            "x": (m[0][2] + m[2][0]) / s,
            "y": (m[1][2] + m[2][1]) / s,
            "z": 0.25 * s,
        })

    def matmul3(self, a, b):
        return [[sum(a[i][k] * b[k][j] for k in range(3)) for j in range(3)] for i in range(3)]

    def mattranspose3(self, m):
        return [[m[j][i] for j in range(3)] for i in range(3)]

    def quat_from_rpy(self, roll, pitch, yaw):
        cr, sr = math.cos(roll * 0.5), math.sin(roll * 0.5)
        cp, sp = math.cos(pitch * 0.5), math.sin(pitch * 0.5)
        cy, sy = math.cos(yaw * 0.5), math.sin(yaw * 0.5)
        return self.normalize_quat({
            "x": sr * cp * cy - cr * sp * sy,
            "y": cr * sp * cy + sr * cp * sy,
            "z": cr * cp * sy - sr * sp * cy,
            "w": cr * cp * cy + sr * sp * sy,
        })

    def map_vr_to_ros_quat(self, rot):
        basis = [
            [0.0, 0.0, -1.0],
            [-1.0, 0.0, 0.0],
            [0.0, 1.0, 0.0],
        ]
        r_vr = self.quat_to_matrix(rot)
        r_ros = self.matmul3(self.matmul3(basis, r_vr), self.mattranspose3(basis))
        return self.matrix_to_quat(r_ros)

    def apply_wrist_to_ee_offset(self, q_ros_wrist):
        if self.wrist_to_ee_order == "pre":
            return self.normalize_quat(self.quat_multiply(self.wrist_to_ee_offset, q_ros_wrist))
        return self.normalize_quat(self.quat_multiply(q_ros_wrist, self.wrist_to_ee_offset))

    def rotate_xy(self, x, y, yaw):
        c = math.cos(yaw)
        s = math.sin(yaw)
        return c * x - s * y, s * x + c * y

    def publish_zero_twist(self, twist_pub, side=None, mode="zero"):
        msg = TwistStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.base_frame
        twist_pub.publish(msg)
        if side is not None:
            self.last_arm_cmd[side] = {
                "linear": (0.0, 0.0, 0.0),
                "angular": (0.0, 0.0, 0.0),
                "clutch": shared_foot_clutch[side],
                "mode": mode,
            }

    def calc_angular_velocity(self, q1, q2, dt):
        q1 = self.normalize_quat(q1)
        q2 = self.normalize_quat(q2)
        dot = q1["x"] * q2["x"] + q1["y"] * q2["y"] + q1["z"] * q2["z"] + q1["w"] * q2["w"]
        if dot < 0.0:
            q2 = {k: -q2[k] for k in ("x", "y", "z", "w")}

        # base_link/world-frame angular delta: R_delta = R_curr * R_prev^T.
        q_delta = self.normalize_quat(self.quat_multiply(q2, self.quat_inverse(q1)))
        v_norm = math.sqrt(q_delta["x"] ** 2 + q_delta["y"] ** 2 + q_delta["z"] ** 2)
        if v_norm < 1e-9 or dt <= 0.0:
            return 0.0, 0.0, 0.0

        angle = 2.0 * math.atan2(v_norm, q_delta["w"])
        if angle > math.pi:
            angle -= 2.0 * math.pi
        elif angle < -math.pi:
            angle += 2.0 * math.pi

        scale = angle / (v_norm * dt)
        return q_delta["x"] * scale, q_delta["y"] * scale, q_delta["z"] * scale

    def process_arm(self, side, xr_data, twist_pub):
        global shared_foot_clutch

        curr_clutch = shared_foot_clutch[side]
        vr_pos = self.map_vr_to_ros(xr_data["pos"])
        vr_rot = self.apply_wrist_to_ee_offset(self.map_vr_to_ros_quat(xr_data["rot"]))
        ee_frame = self.left_ee_frame if side == "left" else self.right_ee_frame

        if curr_clutch and not self.prev_clutch[side]:
            try:
                trans = self.tf_buffer.lookup_transform(self.base_frame, ee_frame, rclpy.time.Time())
                self.robot_zero[side] = trans.transform.translation
                self.human_zero[side] = vr_pos
                self.arm_zero_head_yaw[side] = self.latest_head_yaw
                self.rot_zero[side] = vr_rot
                self.prev_state[side] = {"rot": vr_rot}
                self.publish_zero_twist(twist_pub, side, "clutch_on_zero")
            except Exception as e:
                self.get_logger().warn(
                    f"TF lookup failed for {self.base_frame}->{ee_frame}: {e}",
                    throttle_duration_sec=1.0,
                )
                self.last_arm_cmd[side]["mode"] = "tf_missing"
                self.last_arm_cmd[side]["clutch"] = curr_clutch
                self.prev_clutch[side] = True
                return
            self.prev_clutch[side] = curr_clutch
            return

        if not curr_clutch:
            if self.prev_clutch[side]:
                self.publish_zero_twist(twist_pub, side, "clutch_off_zero")
            else:
                self.last_arm_cmd[side]["clutch"] = False
                self.last_arm_cmd[side]["mode"] = "clutch_off"
            self.prev_clutch[side] = curr_clutch
            return

        if self.robot_zero[side] is None or self.human_zero[side] is None:
            self.prev_clutch[side] = curr_clutch
            return

        try:
            curr_trans = self.tf_buffer.lookup_transform(self.base_frame, ee_frame, rclpy.time.Time())
            curr_pos = curr_trans.transform.translation

            dx = vr_pos["x"] - self.human_zero[side]["x"]
            dy = vr_pos["y"] - self.human_zero[side]["y"]
            dz = vr_pos["z"] - self.human_zero[side]["z"]

            base_x = self.robot_zero[side].x + dx
            base_y = self.robot_zero[side].y + dy
            target_z = self.robot_zero[side].z + dz

            body_yaw_delta = -(self.latest_head_yaw - self.arm_zero_head_yaw[side])
            if self.rotate_arm_with_head_yaw:
                # Waist command is -head yaw, so rotate the arm target with the same commanded body yaw.
                target_x, target_y = self.rotate_xy(base_x, base_y, body_yaw_delta)
            else:
                target_x, target_y = base_x, base_y

            target_rot = vr_rot
            if self.rotate_orientation_with_head_yaw:
                q_body_yaw = self.quat_from_rpy(0.0, 0.0, body_yaw_delta)
                target_rot = self.normalize_quat(self.quat_multiply(q_body_yaw, vr_rot))

            p_gain = 4.0
            msg = TwistStamped()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = self.base_frame
            msg.twist.linear.x = (target_x - curr_pos.x) * p_gain
            msg.twist.linear.y = (target_y - curr_pos.y) * p_gain
            msg.twist.linear.z = (target_z - curr_pos.z) * p_gain

            prev_rot = self.prev_state[side]["rot"]
            wx, wy, wz = self.calc_angular_velocity(prev_rot, target_rot, self.dt)
            a_gain = 2.0
            msg.twist.angular.x = float(wx * a_gain)
            msg.twist.angular.y = float(wy * a_gain)
            msg.twist.angular.z = float(wz * a_gain)

            twist_pub.publish(msg)
            self.last_arm_cmd[side] = {
                "linear": (msg.twist.linear.x, msg.twist.linear.y, msg.twist.linear.z),
                "angular": (msg.twist.angular.x, msg.twist.angular.y, msg.twist.angular.z),
                "clutch": curr_clutch,
                "mode": "active",
            }
            self.prev_state[side]["rot"] = target_rot
        except Exception:
            pass

        self.prev_clutch[side] = curr_clutch

    def process_head_and_waist(self, head_data):
        is_clutch_active = shared_foot_clutch["left"] or shared_foot_clutch["right"]
        mapped_q = self.map_vr_to_ros_quat(head_data["rot"])
        roll, pitch, yaw = self.euler_from_quaternion(mapped_q)
        self.latest_head_yaw = yaw

        head_msg = JointTrajectory()
        head_msg.header.stamp = self.get_clock().now().to_msg()
        head_msg.joint_names = ["29_Joint_Neck_Yaw", "30_Joint_Neck_Pitch"]
        head_point = JointTrajectoryPoint()
        head_point.positions = [0.0, 0.3]
        head_point.time_from_start.nanosec = 50000000
        head_msg.points.append(head_point)
        self.head_pub.publish(head_msg)

        if not is_clutch_active:
            self.prev_waist_clutch = False
            self.prev_waist_state = None
            return

        current_state = {"roll": roll, "pitch": pitch, "yaw": yaw}

        if not self.prev_waist_clutch or self.prev_waist_state is None:
            self.prev_waist_clutch = True
            self.prev_waist_state = current_state
            self.publish_waist_delta(0.0, 0.0)
            return

        delta_pitch = pitch - self.prev_waist_state["pitch"]
        delta_yaw = yaw - self.prev_waist_state["yaw"]
        self.publish_waist_delta(delta_yaw, delta_pitch)
        self.prev_waist_state = current_state

    def publish_waist_delta(self, yaw, pitch):
        waist_msg = JointTrajectory()
        waist_msg.header.stamp = self.get_clock().now().to_msg()
        waist_msg.joint_names = ["0_Joint_Waist_Yaw", "1_Joint_Waist_Roll", "2_Joint_Waist_Pitch"]
        waist_point = JointTrajectoryPoint()
        waist_point.positions = [float(-yaw), 0.0, float(-pitch)]
        waist_point.time_from_start.nanosec = 50000000
        waist_msg.points.append(waist_point)
        self.waist_pub.publish(waist_msg)

    def normalize_position(self, value):
        return max(0.0, min(1.0, float(value)))

    def distance_to_angle(self, dist):
        min_dist = 0.05
        max_dist = 0.18
        angle = (max_dist - dist) / (max_dist - min_dist) * 1.57
        return max(0.0, min(1.57, angle))

    def angle_to_target(self, angle):
        return self.normalize_position(angle / 1.57)

    def pos(self, joints, name):
        item = joints.get(name)
        if not item:
            return None
        p = item.get("pos") or {}
        try:
            return (float(p["x"]), float(p["y"]), float(p["z"]))
        except Exception:
            return None

    def vsub(self, a, b):
        return (a[0] - b[0], a[1] - b[1], a[2] - b[2])

    def vdot(self, a, b):
        return a[0] * b[0] + a[1] * b[1] + a[2] * b[2]

    def vcross(self, a, b):
        return (
            a[1] * b[2] - a[2] * b[1],
            a[2] * b[0] - a[0] * b[2],
            a[0] * b[1] - a[1] * b[0],
        )

    def vnorm(self, a):
        return math.sqrt(self.vdot(a, a))

    def vscale(self, a, k):
        return (a[0] * k, a[1] * k, a[2] * k)

    def joint_angle_deg(self, a, b, c):
        v1 = self.vsub(a, b)
        v2 = self.vsub(c, b)
        n1 = self.vnorm(v1)
        n2 = self.vnorm(v2)
        if n1 < 1e-6 or n2 < 1e-6:
            return None
        value = max(-1.0, min(1.0, self.vdot(v1, v2) / (n1 * n2)))
        return math.degrees(math.acos(value))

    def project_to_plane(self, v, normal):
        n2 = self.vdot(normal, normal)
        if n2 < 1e-9:
            return None
        return self.vsub(v, self.vscale(normal, self.vdot(v, normal) / n2))

    def signed_angle_on_plane(self, v1, v2, normal):
        p1 = self.project_to_plane(v1, normal)
        p2 = self.project_to_plane(v2, normal)
        if p1 is None or p2 is None:
            return None
        n1 = self.vnorm(p1)
        n2 = self.vnorm(p2)
        nn = self.vnorm(normal)
        if n1 < 1e-6 or n2 < 1e-6 or nn < 1e-6:
            return None
        return math.degrees(math.atan2(self.vdot(normal, self.vcross(p1, p2)) / nn, self.vdot(p1, p2)))

    def range_to_target(self, value, open_value, closed_value):
        denom = open_value - closed_value
        if abs(denom) < 1e-6:
            return 0.0
        return self.normalize_position((open_value - value) / denom)

    def range_to_target_increasing(self, value, open_value, closed_value):
        denom = closed_value - open_value
        if abs(denom) < 1e-6:
            return 0.0
        return self.normalize_position((value - open_value) / denom)

    def finger_bend_from_joints(self, joints, chain):
        pts = [self.pos(joints, name) for name in chain]
        angles = []
        for i in range(1, len(pts) - 1):
            if pts[i - 1] and pts[i] and pts[i + 1]:
                angle = self.joint_angle_deg(pts[i - 1], pts[i], pts[i + 1])
                if angle is not None:
                    angles.append(angle)
        if not angles:
            return None
        avg_angle = sum(angles) / len(angles)
        return self.range_to_target(avg_angle, self.finger_open_deg, self.finger_closed_deg)

    def joint_based_targets(self, side, joints):
        wrist = self.pos(joints, "wrist")
        thumb_meta = self.pos(joints, "thumb-metacarpal")
        thumb_prox = self.pos(joints, "thumb-phalanx-proximal")
        thumb_distal = self.pos(joints, "thumb-phalanx-distal")
        thumb_tip = self.pos(joints, "thumb-tip")
        index_meta = self.pos(joints, "index-finger-metacarpal")
        middle_meta = self.pos(joints, "middle-finger-metacarpal")
        pinky_meta = self.pos(joints, "pinky-finger-metacarpal")

        required = [wrist, thumb_meta, thumb_prox, thumb_distal, thumb_tip, index_meta, middle_meta, pinky_meta]
        if any(point is None for point in required):
            return None

        thumb_mcp = self.joint_angle_deg(thumb_meta, thumb_prox, thumb_distal)
        thumb_ip = self.joint_angle_deg(thumb_prox, thumb_distal, thumb_tip)
        thumb_bend_angle = thumb_mcp
        thumb_bend = 0.0 if thumb_bend_angle is None else self.range_to_target(
            thumb_bend_angle, self.thumb_bend_open_deg, self.thumb_bend_closed_deg
        )

        palm_normal = self.vcross(self.vsub(index_meta, wrist), self.vsub(pinky_meta, wrist))
        spread_angle = self.signed_angle_on_plane(self.vsub(middle_meta, wrist), self.vsub(thumb_tip, thumb_meta), palm_normal)
        spread_raw = 0.0 if spread_angle is None else self.range_to_target_increasing(
            spread_angle, self.thumb_spread_open_deg, self.thumb_spread_closed_deg
        )

        if (side == "right" and self.thumb_right_spread_invert) or (side == "left" and self.thumb_left_spread_invert):
            spread_raw = 1.0 - spread_raw if spread_raw > 0.0 else 0.0

        thumb_spread = spread_raw

        chains = {
            "index": ["index-finger-metacarpal", "index-finger-phalanx-proximal", "index-finger-phalanx-intermediate", "index-finger-phalanx-distal", "index-finger-tip"],
            "middle": ["middle-finger-metacarpal", "middle-finger-phalanx-proximal", "middle-finger-phalanx-intermediate", "middle-finger-phalanx-distal", "middle-finger-tip"],
            "ring": ["ring-finger-metacarpal", "ring-finger-phalanx-proximal", "ring-finger-phalanx-intermediate", "ring-finger-phalanx-distal", "ring-finger-tip"],
            "pinky": ["pinky-finger-metacarpal", "pinky-finger-phalanx-proximal", "pinky-finger-phalanx-intermediate", "pinky-finger-phalanx-distal", "pinky-finger-tip"],
        }
        targets = {
            "thumb_up": self.normalize_position(thumb_bend * self.thumb_bend_gain),
            "index": self.finger_bend_from_joints(joints, chains["index"]),
            "middle": self.finger_bend_from_joints(joints, chains["middle"]),
            "ring": self.finger_bend_from_joints(joints, chains["ring"]),
            "pinky": self.finger_bend_from_joints(joints, chains["pinky"]),
            "thumb_down": self.normalize_position(
                self.force_thumb_spread if self.force_thumb_spread >= 0.0 else thumb_spread * self.thumb_spread_gain
            ),
        }
        if any(targets[name] is None for name in ("index", "middle", "ring", "pinky")):
            return None

        self.last_thumb_features[side] = {
            "mcp": None if thumb_mcp is None else round(thumb_mcp, 1),
            "ip": None if thumb_ip is None else round(thumb_ip, 1),
            "bend_angle": None if thumb_bend_angle is None else round(thumb_bend_angle, 1),
            "spread_angle": None if spread_angle is None else round(spread_angle, 1),
            "spread_raw": round(spread_raw, 3),
        }
        return targets

    def process_fingers(self, side, xr_data, hand_pub=None):
        if not shared_foot_clutch[side]:
            self.last_retarget_mode[side] = "clutch_off"
            return

        if "fingers" not in xr_data or not xr_data["fingers"]:
            return

        fingers = xr_data["fingers"]
        joints = xr_data.get("joints") or {}
        hand_targets = self.joint_based_targets(side, joints) if joints else None
        self.last_retarget_mode[side] = "joint" if hand_targets is not None else "fallback"

        if hand_targets is None:
            xr_joint_map = {
                "thumb": "thumb-tip",
                "index": "index-finger-tip",
                "middle": "middle-finger-tip",
                "ring": "ring-finger-tip",
                "pinky": "pinky-finger-tip",
            }
            angles = {}
            for local_name, xr_name in xr_joint_map.items():
                dist = fingers.get(xr_name)
                if dist is None:
                    continue
                angles[local_name] = self.distance_to_angle(float(dist))
            if "thumb" not in angles:
                return
            non_thumb = [angles[name] for name in ("index", "middle", "ring", "pinky") if name in angles]
            if not non_thumb:
                return
            thumb_value = self.angle_to_target(angles.get("thumb", 0.0))
            hand_targets = {
                "thumb_up": thumb_value,
                "index": self.angle_to_target(angles.get("index", 0.0)),
                "middle": self.angle_to_target(angles.get("middle", 0.0)),
                "ring": self.angle_to_target(angles.get("ring", 0.0)),
                "pinky": self.angle_to_target(angles.get("pinky", 0.0)),
                "thumb_down": self.normalize_position(
                    self.force_thumb_spread if self.force_thumb_spread >= 0.0 else thumb_value
                ),
            }

        thumb_value = max(hand_targets["thumb_up"], hand_targets["thumb_down"])
        all_value = sum(hand_targets[name] for name in ("index", "middle", "ring", "pinky")) / 4.0

        with self.finger_lock:
            if side == "left":
                self.left_targets = hand_targets
                self.fing_th_L = thumb_value
                self.fing_all_L = all_value
            else:
                self.right_targets = hand_targets
                self.fing_th_R = thumb_value
                self.fing_all_R = all_value

        self.publish_hand_targets()

    def teleop_callback(self, msg):
        updated = False

        with self.finger_lock:
            for i, name in enumerate(msg.name):
                if i >= len(msg.position):
                    break

                if name == "Fing_all_L":
                    value = self.normalize_position(msg.position[i])
                    self.fing_all_L = value
                    for key in ("index", "middle", "ring", "pinky"):
                        self.left_targets[key] = value
                    updated = True
                elif name == "Fing_th_L":
                    value = self.normalize_position(msg.position[i])
                    self.fing_th_L = value
                    self.left_targets["thumb_up"] = value
                    self.left_targets["thumb_down"] = value
                    updated = True
                elif name == "Fing_all_R":
                    value = self.normalize_position(msg.position[i])
                    self.fing_all_R = value
                    for key in ("index", "middle", "ring", "pinky"):
                        self.right_targets[key] = value
                    updated = True
                elif name == "Fing_th_R":
                    value = self.normalize_position(msg.position[i])
                    self.fing_th_R = value
                    self.right_targets["thumb_up"] = value
                    self.right_targets["thumb_down"] = value
                    updated = True
                elif name == "Gripper_L":
                    self.gripper_L = self.normalize_position(msg.position[i] * -1.0)
                    updated = True
                elif name == "Gripper_R":
                    self.gripper_R = self.normalize_position(msg.position[i])
                    updated = True

        if updated:
            self.publish_hand_targets()

    def publish_hand_targets(self):
        with self.finger_lock:
            targets = [
                self.right_targets["thumb_up"],
                self.right_targets["index"],
                self.right_targets["middle"],
                self.right_targets["ring"],
                self.right_targets["pinky"],
                self.right_targets["thumb_down"],
                self.left_targets["thumb_up"],
                self.left_targets["index"],
                self.left_targets["middle"],
                self.left_targets["ring"],
                self.left_targets["pinky"],
                self.left_targets["thumb_down"],
            ]

        msg = Float32MultiArray()
        msg.data = [float(t) for t in targets]
        self.hand_target_pub.publish(msg)
        self.last_targets = list(msg.data)
        self.target_publish_count += 1

    def status_callback(self, msg):
        try:
            raw_data = msg.data
            if raw_data.startswith("data: '"):
                raw_data = raw_data[7:-1]

            parsed_data = json.loads(raw_data)
            if "targets" not in parsed_data:
                return

            joint_state_msg = JointState()
            joint_state_msg.header.stamp = self.get_clock().now().to_msg()
            joint_state_msg.name = self.joint_names
            joint_state_msg.position = [float(val) for val in parsed_data["targets"]]
            self.joint_state_pub.publish(joint_state_msg)
        except json.JSONDecodeError:
            self.get_logger().error(f"JSON parse error. data={msg.data}")
        except Exception as e:
            self.get_logger().error(f"Unexpected hand status error: {e}")

    def timer_callback(self):
        if shared_xr_data["head"]:
            self.process_head_and_waist(shared_xr_data["head"])
        if shared_xr_data["left"]:
            self.process_arm("left", shared_xr_data["left"], self.left_pub)
            self.process_fingers("left", shared_xr_data["left"], self.left_hand_pub)
        else:
            self.last_arm_cmd["left"]["mode"] = "no_xr"
            self.last_arm_cmd["left"]["clutch"] = shared_foot_clutch["left"]

        if shared_xr_data["right"]:
            self.process_arm("right", shared_xr_data["right"], self.right_pub)
            self.process_fingers("right", shared_xr_data["right"], self.right_hand_pub)
        else:
            self.last_arm_cmd["right"]["mode"] = "no_xr"
            self.last_arm_cmd["right"]["clutch"] = shared_foot_clutch["right"]

    def dominant_rotation_label(self, values, threshold=0.2):
        names = ("rot_about_base_x", "rot_about_base_y", "rot_about_base_z")
        idx = max(range(3), key=lambda i: abs(values[i]))
        value = values[idx]
        if abs(value) < threshold:
            return "none"
        return f"{names[idx]}{'+' if value >= 0.0 else '-'}({value:.4f})"

    def debug_callback(self):
        l_ang = self.last_arm_cmd["left"]["angular"]
        r_ang = self.last_arm_cmd["right"]["angular"]
        left_ang = [round(v, 4) for v in l_ang]
        right_ang = [round(v, 4) for v in r_ang]
        ws_age = None if shared_ws_last_time <= 0.0 else round(time.time() - shared_ws_last_time, 2)
        has_left = bool(shared_xr_data.get("left"))
        has_right = bool(shared_xr_data.get("right"))
        self.get_logger().info(
            "teleop_status ws_count=%d ws_age=%s xr L/R=%s/%s clutch_raw L/R=%s/%s "
            "arm_orientation ang_base L=%s R=%s dom L/R=%s/%s mode L/R=%s/%s"
            % (
                shared_ws_count,
                ws_age,
                has_left,
                has_right,
                shared_foot_clutch["left"],
                shared_foot_clutch["right"],
                left_ang,
                right_ang,
                self.dominant_rotation_label(l_ang),
                self.dominant_rotation_label(r_ang),
                self.last_arm_cmd["left"]["mode"],
                self.last_arm_cmd["right"]["mode"],
            )
        )


def main(args=None):
    rclpy.init(args=args)
    node = FingerFootPoseNode()

    foot_thread = threading.Thread(target=foot_pedal_monitor, args=(node.get_logger(),), daemon=True)
    foot_thread.start()

    ws_thread = threading.Thread(target=start_ws_server, daemon=True)
    ws_thread.start()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()

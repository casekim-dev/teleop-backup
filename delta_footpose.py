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
from tf2_ros import Buffer, TransformListener
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
import websockets


shared_xr_data = {"head": None, "left": None, "right": None}
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
    global shared_xr_data
    try:
        async for message in websocket:
            shared_xr_data = json.loads(message)
    except websockets.exceptions.ConnectionClosed:
        pass


async def main_ws():
    async with websockets.serve(ws_handler, "0.0.0.0", 8765):
        await asyncio.Future()


def start_ws_server():
    asyncio.run(main_ws())


class DeltaFootPoseNode(Node):
    def __init__(self):
        super().__init__("delta_footpose_node")
        self.dt = 0.02

        self.prev_clutch = {"left": False, "right": False}
        self.human_zero = {"left": None, "right": None}
        self.rot_zero = {"left": None, "right": None}
        self.robot_zero = {"left": None, "right": None}
        self.prev_state = {"left": None, "right": None}

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.left_pub = self.create_publisher(TwistStamped, "/left_servo/left_servo/delta_twist_cmds", 10)
        self.right_pub = self.create_publisher(TwistStamped, "/right_servo/right_servo/delta_twist_cmds", 10)
        self.head_pub = self.create_publisher(JointTrajectory, "/head_controller/joint_trajectory", 10)
        self.waist_pub = self.create_publisher(JointTrajectory, "/waist_controller/joint_trajectory", 10)
        self.left_hand_pub = self.create_publisher(JointTrajectory, "/left_hand_controller/joint_trajectory", 10)
        self.right_hand_pub = self.create_publisher(JointTrajectory, "/right_hand_controller/joint_trajectory", 10)

        self.head_first_msg_time = None
        self.head_zero = None

        self.timer = self.create_timer(self.dt, self.timer_callback)
        self.get_logger().info("Delta foot pose node started")

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

    def map_vr_to_ros_quat(self, rot):
        return {"x": -rot["z"], "y": -rot["x"], "z": rot["y"], "w": rot["w"]}

    def publish_zero_twist(self, twist_pub):
        msg = TwistStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "base_link"
        twist_pub.publish(msg)

    def calc_angular_velocity(self, q1, q2, dt):
        dot = q1["x"] * q2["x"] + q1["y"] * q2["y"] + q1["z"] * q2["z"] + q1["w"] * q2["w"]
        sign = 1.0 if dot >= 0 else -1.0
        qdx = (q2["x"] * sign - q1["x"]) / dt
        qdy = (q2["y"] * sign - q1["y"]) / dt
        qdz = (q2["z"] * sign - q1["z"]) / dt
        qdw = (q2["w"] * sign - q1["w"]) / dt
        wx = 2.0 * (qdx * q1["w"] - qdw * q1["x"] - qdy * q1["z"] + qdz * q1["y"])
        wy = 2.0 * (qdy * q1["w"] - qdw * q1["y"] + qdx * q1["z"] - qdz * q1["x"])
        wz = 2.0 * (qdz * q1["w"] - qdw * q1["z"] - qdx * q1["y"] + qdy * q1["x"])
        return wx, wy, wz

    def process_arm(self, side, xr_data, twist_pub):
        global shared_foot_clutch

        curr_clutch = shared_foot_clutch[side]
        vr_pos = self.map_vr_to_ros(xr_data["pos"])
        vr_rot = xr_data["rot"]
        ee_frame = "Left_Hand" if side == "left" else "Right_Hand"

        if curr_clutch and not self.prev_clutch[side]:
            try:
                trans = self.tf_buffer.lookup_transform("base_link", ee_frame, rclpy.time.Time())
                self.robot_zero[side] = trans.transform.translation
                self.human_zero[side] = vr_pos
                self.rot_zero[side] = vr_rot
                self.prev_state[side] = {"rot": vr_rot}
                self.publish_zero_twist(twist_pub)
            except Exception as e:
                self.get_logger().warn(f"TF lookup failed for {ee_frame}: {e}")
                curr_clutch = False
            self.prev_clutch[side] = curr_clutch
            return

        if not curr_clutch:
            if self.prev_clutch[side]:
                self.publish_zero_twist(twist_pub)
            self.prev_clutch[side] = curr_clutch
            return

        if self.robot_zero[side] is None or self.human_zero[side] is None:
            self.prev_clutch[side] = curr_clutch
            return

        try:
            curr_trans = self.tf_buffer.lookup_transform("base_link", ee_frame, rclpy.time.Time())
            curr_pos = curr_trans.transform.translation

            target_x = self.robot_zero[side].x + (vr_pos["x"] - self.human_zero[side]["x"])
            target_y = self.robot_zero[side].y + (vr_pos["y"] - self.human_zero[side]["y"])
            target_z = self.robot_zero[side].z + (vr_pos["z"] - self.human_zero[side]["z"])

            p_gain = 4.0
            msg = TwistStamped()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = "base_link"
            msg.twist.linear.x = (target_x - curr_pos.x) * p_gain
            msg.twist.linear.y = (target_y - curr_pos.y) * p_gain
            msg.twist.linear.z = (target_z - curr_pos.z) * p_gain

            prev_rot = self.prev_state[side]["rot"]
            wx, wy, wz = self.calc_angular_velocity(prev_rot, vr_rot, self.dt)
            a_gain = 1.0
            msg.twist.angular.x = float(-wz * a_gain)
            msg.twist.angular.y = float(-wx * a_gain)
            msg.twist.angular.z = float(wy * a_gain)

            twist_pub.publish(msg)
            self.prev_state[side]["rot"] = vr_rot
        except Exception:
            pass

        self.prev_clutch[side] = curr_clutch

    def process_head_and_waist(self, head_data):
        is_clutch_active = shared_foot_clutch["left"] or shared_foot_clutch["right"]
        mapped_q = self.map_vr_to_ros_quat(head_data["rot"])
        roll, pitch, yaw = self.euler_from_quaternion(mapped_q)

        if self.head_zero is None:
            current_time = time.time()
            if self.head_first_msg_time is None:
                self.head_first_msg_time = current_time
                self.get_logger().info("Head tracking detected. Calibrating zero in 3 seconds.")
                return
            if current_time - self.head_first_msg_time < 3.0:
                return
            self.head_zero = {"roll": roll, "pitch": pitch, "yaw": yaw}
            self.get_logger().info("Head zero calibrated")
            return

        final_roll = roll - self.head_zero["roll"]
        final_pitch = pitch - self.head_zero["pitch"]
        final_yaw = yaw - self.head_zero["yaw"]

        head_msg = JointTrajectory()
        head_msg.header.stamp = self.get_clock().now().to_msg()
        head_msg.joint_names = ["29_Joint_Neck_Yaw", "30_Joint_Neck_Pitch"]
        head_point = JointTrajectoryPoint()
        head_point.positions = [0.0, 0.3]
        head_point.time_from_start.nanosec = 50000000
        head_msg.points.append(head_point)
        self.head_pub.publish(head_msg)

        if is_clutch_active:
            waist_msg = JointTrajectory()
            waist_msg.header.stamp = self.get_clock().now().to_msg()
            waist_msg.joint_names = ["0_Joint_Waist_Yaw", "1_Joint_Waist_Roll", "2_Joint_Waist_Pitch"]
            waist_point = JointTrajectoryPoint()
            waist_point.positions = [float(final_yaw), float(final_roll), float(final_pitch)]
            waist_point.time_from_start.nanosec = 50000000
            waist_msg.points.append(waist_point)
            self.waist_pub.publish(waist_msg)

    def process_fingers(self, side, xr_data, hand_pub):
        if "fingers" not in xr_data or not xr_data["fingers"]:
            return

        fingers = xr_data["fingers"]
        suffix = "Left" if side == "left" else "Right"
        joint_msg = JointTrajectory()
        joint_msg.header.stamp = self.get_clock().now().to_msg()
        joint_msg.joint_names = (
            [
                f"31_Joint_Thumb_{suffix}",
                f"32_Joint_Index_{suffix}",
                f"33_Joint_Middle_{suffix}",
                f"34_Joint_Ring_{suffix}",
                f"35_Joint_Pinky_{suffix}",
            ]
            if side == "left"
            else [
                f"36_Joint_Thumb_{suffix}",
                f"37_Joint_Index_{suffix}",
                f"38_Joint_Middle_{suffix}",
                f"39_Joint_Ring_{suffix}",
                f"40_Joint_Pinky_{suffix}",
            ]
        )

        min_dist = 0.05
        max_dist = 0.18
        xr_joint_map = {
            "thumb": "thumb-tip",
            "index": "index-finger-tip",
            "middle": "middle-finger-tip",
            "ring": "ring-finger-tip",
            "pinky": "pinky-finger-tip",
        }

        positions = []
        for f_key in ["thumb", "index", "middle", "ring", "pinky"]:
            dist = fingers.get(xr_joint_map[f_key])
            if dist is None:
                positions.append(0.0)
                continue
            angle = (max_dist - dist) / (max_dist - min_dist) * 1.57
            positions.append(float(max(0.0, min(1.57, angle))))

        point = JointTrajectoryPoint()
        point.positions = positions
        point.time_from_start.nanosec = 50000000
        joint_msg.points.append(point)
        hand_pub.publish(joint_msg)

    def timer_callback(self):
        if shared_xr_data["head"]:
            self.process_head_and_waist(shared_xr_data["head"])
        if shared_xr_data["left"]:
            self.process_arm("left", shared_xr_data["left"], self.left_pub)
            self.process_fingers("left", shared_xr_data["left"], self.left_hand_pub)
        if shared_xr_data["right"]:
            self.process_arm("right", shared_xr_data["right"], self.right_pub)
            self.process_fingers("right", shared_xr_data["right"], self.right_hand_pub)


def main(args=None):
    rclpy.init(args=args)
    node = DeltaFootPoseNode()

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

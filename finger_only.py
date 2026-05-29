#!/usr/bin/env python3
import asyncio
import json
import math
import threading

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float32MultiArray, String
import websockets


shared_xr_data = {"head": None, "left": None, "right": None}
shared_ws_debug = {
    "count": 0,
    "top_keys": [],
    "left_keys": [],
    "right_keys": [],
    "left_has_fingers": False,
    "right_has_fingers": False,
    "left_has_joints": False,
    "right_has_joints": False,
}


async def ws_handler(websocket):
    global shared_xr_data, shared_ws_debug
    print("WebSocket client connected", flush=True)
    try:
        async for message in websocket:
            shared_xr_data = json.loads(message)
            left = shared_xr_data.get("left") or {}
            right = shared_xr_data.get("right") or {}
            shared_ws_debug = {
                "count": shared_ws_debug["count"] + 1,
                "top_keys": sorted(shared_xr_data.keys()),
                "left_keys": sorted(left.keys()) if isinstance(left, dict) else [],
                "right_keys": sorted(right.keys()) if isinstance(right, dict) else [],
                "left_has_fingers": isinstance(left, dict) and bool(left.get("fingers")),
                "right_has_fingers": isinstance(right, dict) and bool(right.get("fingers")),
                "left_has_joints": isinstance(left, dict) and bool(left.get("joints")),
                "right_has_joints": isinstance(right, dict) and bool(right.get("joints")),
            }
    except websockets.exceptions.ConnectionClosed:
        pass


async def main_ws(logger=None):
    if logger:
        logger.info("Starting WebSocket server on 0.0.0.0:8765")
    async with websockets.serve(ws_handler, "0.0.0.0", 8765):
        if logger:
            logger.info("WebSocket server listening on 0.0.0.0:8765")
        await asyncio.Future()


def start_ws_server(logger=None):
    try:
        asyncio.run(main_ws(logger))
    except Exception as e:
        if logger:
            logger.error(f"WebSocket server failed: {e}")
        else:
            raise


class FingerOnlyNode(Node):
    def __init__(self):
        super().__init__("finger_only_node")

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
        self.last_left_fingers = None
        self.last_right_fingers = None
        self.last_targets = [0.0] * 12
        self.target_publish_count = 0
        self.last_thumb_dist = {"left": None, "right": None}
        self.thumb_open_dist = {"left": None, "right": None}
        self.last_thumb_features = {"left": {}, "right": {}}
        self.last_retarget_mode = {"left": "none", "right": "none"}
        self.thumb_spread_hold = {"left": None, "right": None}

        # Prefer WebXR's 25 hand joints when available. The older wrist-to-tip
        # distance path remains as a fallback for the previous browser payload.
        self.thumb_open_deadzone = float(self.declare_parameter("thumb_open_deadzone", 0.20).value)
        self.thumb_spread_complete_at = float(self.declare_parameter("thumb_spread_complete_at", 0.25).value)
        self.thumb_bend_start = float(self.declare_parameter("thumb_bend_start", 0.0).value)
        self.thumb_bend_gain = float(self.declare_parameter("thumb_bend_gain", 1.0).value)
        self.thumb_spread_gain = float(self.declare_parameter("thumb_spread_gain", 1.0).value)
        self.thumb_right_spread_invert = bool(self.declare_parameter("thumb_right_spread_invert", False).value)
        self.thumb_left_spread_invert = bool(self.declare_parameter("thumb_left_spread_invert", False).value)
        self.force_thumb_spread = float(self.declare_parameter("force_thumb_spread", -1.0).value)
        self.thumb_close_dist = float(self.declare_parameter("thumb_close_dist", 0.05).value)
        self.thumb_default_open_dist = float(self.declare_parameter("thumb_default_open_dist", 0.12).value)
        self.thumb_min_range = float(self.declare_parameter("thumb_min_range", 0.045).value)
        self.thumb_bend_open_deg = float(self.declare_parameter("thumb_bend_open_deg", 166.0).value)
        self.thumb_bend_closed_deg = float(self.declare_parameter("thumb_bend_closed_deg", 122.0).value)
        self.thumb_spread_hold_start = float(self.declare_parameter("thumb_spread_hold_start", 2.0).value)
        self.thumb_spread_hold_release = float(self.declare_parameter("thumb_spread_hold_release", 1.5).value)
        self.thumb_spread_hold_follow = float(self.declare_parameter("thumb_spread_hold_follow", 1.0).value)
        self.thumb_spread_open_deg = float(self.declare_parameter("thumb_spread_open_deg", -38.0).value)
        self.thumb_spread_closed_deg = float(self.declare_parameter("thumb_spread_closed_deg", -14.0).value)
        self.finger_open_deg = float(self.declare_parameter("finger_open_deg", 172.0).value)
        self.finger_closed_deg = float(self.declare_parameter("finger_closed_deg", 105.0).value)

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
        self.hand_target_pub = self.create_publisher(Float32MultiArray, "/igris_c/hand/targets", 10)
        self.joint_state_pub = self.create_publisher(JointState, "/igris/hand/status", 10)

        self.timer = self.create_timer(0.02, self.timer_callback)
        self.debug_timer = self.create_timer(1.0, self.debug_callback)

        self.get_logger().info("Finger-only robot hand node started")
        self.get_logger().info("WebSocket input: 0.0.0.0:8765")
        self.get_logger().info(
            "XR fingers -> /igris_c/hand/targets in DDS HandCmd order "
            "(joint-based retargeting when WebXR joints are available)"
        )
        self.get_logger().info("Sub: /igris/hand/joint_states, /igris_c/hand/status")
        self.get_logger().info("Pub: /igris_c/hand/targets, /igris/hand/status")
        self.get_logger().info("Serve the Quest page separately, e.g. python3 -m http.server 8012")

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
        # MCP is the cleanest bend signal from the calibration summary. IP moves less consistently.
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

        # Direct continuous spread mapping. Keep this simple now that 16/26 are verified.
        # Any bend/spread coupling seen here is from WebXR features, not the output channel.
        self.thumb_spread_hold[side] = None
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
            "spread_hold": None,
        }
        return targets

    def open_zero_target(self, value):
        value = self.normalize_position(value)
        deadzone = self.normalize_position(self.thumb_open_deadzone)
        if value <= deadzone:
            return 0.0
        if deadzone >= 1.0:
            return 0.0
        return self.normalize_position((value - deadzone) / (1.0 - deadzone))

    def thumb_distance_to_base(self, side, dist):
        dist = float(dist)
        self.last_thumb_dist[side] = dist

        prev_open = self.thumb_open_dist[side]
        if prev_open is None or dist > prev_open:
            self.thumb_open_dist[side] = dist

        open_dist = self.thumb_open_dist[side]
        if open_dist is None:
            open_dist = self.thumb_default_open_dist

        close_dist = min(self.thumb_close_dist, open_dist - 0.01)
        denom = max(open_dist - close_dist, self.thumb_min_range)
        raw = (open_dist - dist) / denom
        return self.open_zero_target(raw)

    def thumb_targets(self, side, dist):
        base = self.thumb_distance_to_base(side, dist)

        spread_complete_at = self.normalize_position(self.thumb_spread_complete_at)
        if spread_complete_at <= 0.0:
            spread = 1.0 if base > 0.0 else 0.0
        else:
            spread = self.normalize_position(base / spread_complete_at)

        bend_start = self.normalize_position(self.thumb_bend_start)
        if base <= bend_start or bend_start >= 1.0:
            bend = 0.0
        else:
            bend = self.normalize_position((base - bend_start) / (1.0 - bend_start))

        bend = self.normalize_position(bend * self.thumb_bend_gain)
        spread = self.normalize_position(spread * self.thumb_spread_gain)
        if (side == "right" and self.thumb_right_spread_invert) or (side == "left" and self.thumb_left_spread_invert):
            spread = 1.0 - spread if spread > 0.0 else 0.0

        return bend, spread

    def timer_callback(self):
        if shared_xr_data["left"]:
            self.process_xr_fingers("left", shared_xr_data["left"])
        if shared_xr_data["right"]:
            self.process_xr_fingers("right", shared_xr_data["right"])

    def process_xr_fingers(self, side, xr_data):
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

            thumb_bend, thumb_spread = self.thumb_targets(side, fingers.get("thumb-tip", self.thumb_default_open_dist))
            hand_targets = {
                "thumb_up": thumb_bend,
                "index": self.angle_to_target(angles.get("index", 0.0)),
                "middle": self.angle_to_target(angles.get("middle", 0.0)),
                "ring": self.angle_to_target(angles.get("ring", 0.0)),
                "pinky": self.angle_to_target(angles.get("pinky", 0.0)),
                "thumb_down": self.normalize_position(
                    self.force_thumb_spread if self.force_thumb_spread >= 0.0 else thumb_spread
                ),
            }

        thumb_value = max(hand_targets["thumb_up"], hand_targets["thumb_down"])
        all_value = sum(hand_targets[name] for name in ("index", "middle", "ring", "pinky")) / 4.0

        with self.finger_lock:
            if side == "left":
                self.left_targets = hand_targets
                self.fing_th_L = thumb_value
                self.fing_all_L = all_value
                self.last_left_fingers = dict(fingers)
            else:
                self.right_targets = hand_targets
                self.fing_th_R = thumb_value
                self.fing_all_R = all_value
                self.last_right_fingers = dict(fingers)

        self.publish_hand_targets()

    def teleop_callback(self, msg):
        updated = False

        with self.finger_lock:
            for i, name in enumerate(msg.name):
                if i >= len(msg.position):
                    break

                if name == "Fing_all_L":
                    self.fing_all_L = self.normalize_position(msg.position[i])
                    updated = True
                elif name == "Fing_th_L":
                    self.fing_th_L = self.normalize_position(msg.position[i])
                    updated = True
                elif name == "Fing_all_R":
                    self.fing_all_R = self.normalize_position(msg.position[i])
                    updated = True
                elif name == "Fing_th_R":
                    self.fing_th_R = self.normalize_position(msg.position[i])
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

        with self.finger_lock:
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

    def debug_callback(self):
        ws_debug = dict(shared_ws_debug)
        with self.finger_lock:
            values = (
                self.fing_all_L,
                self.fing_th_L,
                self.fing_all_R,
                self.fing_th_R,
                self.last_left_fingers is not None,
                self.last_right_fingers is not None,
                ws_debug["count"],
                ws_debug["left_has_fingers"],
                ws_debug["right_has_fingers"],
                ws_debug.get("left_has_joints", False),
                ws_debug.get("right_has_joints", False),
                ws_debug["top_keys"],
                ws_debug["left_keys"],
                ws_debug["right_keys"],
                self.target_publish_count,
                [round(v, 3) for v in self.last_targets],
                None if self.last_thumb_dist["left"] is None else round(self.last_thumb_dist["left"], 4),
                None if self.thumb_open_dist["left"] is None else round(self.thumb_open_dist["left"], 4),
                None if self.last_thumb_dist["right"] is None else round(self.last_thumb_dist["right"], 4),
                None if self.thumb_open_dist["right"] is None else round(self.thumb_open_dist["right"], 4),
                round(self.right_targets["thumb_up"], 3),
                round(self.right_targets["thumb_down"], 3),
                round(self.left_targets["thumb_up"], 3),
                round(self.left_targets["thumb_down"], 3),
                self.last_thumb_features["right"].get("bend_angle"),
                self.last_thumb_features["right"].get("spread_angle"),
                self.last_thumb_features["left"].get("bend_angle"),
                self.last_thumb_features["left"].get("spread_angle"),
                self.last_thumb_features["right"].get("spread_raw"),
                self.last_thumb_features["right"].get("spread_hold"),
                self.last_thumb_features["left"].get("spread_raw"),
                self.last_thumb_features["left"].get("spread_hold"),
                self.last_retarget_mode["left"],
                self.last_retarget_mode["right"],
            )

        self.get_logger().info(
            "targets L(all/th)=%.3f/%.3f R(all/th)=%.3f/%.3f "
            "xr_seen L=%s R=%s ws_count=%d ws_fingers L=%s R=%s ws_joints L=%s R=%s "
            "keys=%s left=%s right=%s pub_count=%d last_targets=%s "
            "thumb_dist/open L=%s/%s R=%s/%s "
            "thumb R(bend/spread)=%.3f/%.3f L(bend/spread)=%.3f/%.3f "
            "features R(bend_angle/spread_angle)=%s/%s L(bend_angle/spread_angle)=%s/%s "
            "spread_raw/hold R=%s/%s L=%s/%s retarget L/R=%s/%s"
            % values
        )


def main(args=None):
    rclpy.init(args=args)
    node = FingerOnlyNode()

    ws_thread = threading.Thread(target=start_ws_server, args=(node.get_logger(),), daemon=True)
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

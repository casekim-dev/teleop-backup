import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped, TransformStamped
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from moveit_msgs.srv import ServoCommandType
from tf2_ros import Buffer, TransformListener, TransformBroadcaster
import asyncio
import websockets
import json
import threading
import math
import time
import struct
import os

shared_xr_data = {"head": None, "left": None, "right": None}
# 🌟 Foot Pedal 상태 저장용 전역 변수
shared_foot_clutch = {"left": False, "right": False}

# ==========================================
# [스레드 0] 풋 페달 감시 스레드
# ==========================================
def foot_pedal_monitor(node_logger):
    global shared_foot_clutch
    import fcntl
    
    # PCsensor FootSwitch 장치 경로
    device_path = "/dev/input/by-id/usb-PCsensor_FootSwitch-event-kbd"
    
    if not os.path.exists(device_path):
        node_logger.error(f"⚠️  풋 페달 장치를 찾을 수 없습니다: {device_path}")
        return

    # input_event 구조체: time(Q, Q), type(H), code(H), value(i) -> 24 bytes
    event_format = "QQHHi"
    event_size = struct.calcsize(event_format)
    
    # EVIOCGRAB: 장치를 독점하여 터미널에 'a,b,c'가 찍히지 않게 함
    EVIOCGRAB = 0x40044590

    node_logger.info(f"🚀 풋 페달 모니터링 시작: {device_path}")
    
    try:
        with open(device_path, "rb") as f:
            # 장치 독점 시도
            try:
                fcntl.ioctl(f.fileno(), EVIOCGRAB, 1)
                node_logger.info("🔒 풋 페달 장치 독점 성공 (터미널 입력 차단됨)")
            except Exception as e:
                node_logger.warn(f"🔓 장치 독점 실패 (터미널에 글자가 찍힐 수 있음): {e}")

            while True:
                data = f.read(event_size)
                if not data:
                    break
                
                _, _, ev_type, ev_code, ev_value = struct.unpack(event_format, data)
                
                if ev_type == 1: # EV_KEY
                    is_pressed = (ev_value > 0) # 1: press, 2: repeat
                    
                    old_left = shared_foot_clutch["left"]
                    old_right = shared_foot_clutch["right"]

                    if ev_code == 30: # Pedal A -> Left
                        shared_foot_clutch["left"] = is_pressed
                    elif ev_code == 46: # Pedal C -> Right
                        shared_foot_clutch["right"] = is_pressed
                    elif ev_code == 48: # Pedal B -> Both
                        shared_foot_clutch["left"] = is_pressed
                        shared_foot_clutch["right"] = is_pressed
                    
                    # 상태 변화 시 로그 출력
                    if old_left != shared_foot_clutch["left"] or old_right != shared_foot_clutch["right"]:
                        node_logger.info(f"👟 Clutch State: L={shared_foot_clutch['left']}, R={shared_foot_clutch['right']}")

    except PermissionError:
        node_logger.error("❌ 권한 오류: 반드시 'sudo python3 foot_purepose.py'로 실행해야 합니다.")
    except Exception as e:
        node_logger.error(f"❌ 풋 페달 에러: {e}")
    finally:
        # 종료 시 독점 해제는 파일이 닫히면서 자동으로 됨
        pass

# ==========================================
# [스레드 1] 웹소켓 서버
# ==========================================
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

# ==========================================
# [스레드 2] ROS2 전신 제어 노드
# ==========================================
class FullBodyTeleopNode(Node):
    def __init__(self):
        super().__init__('full_body_teleop_node')
        self.dt = 0.02
        
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.br = TransformBroadcaster(self)

        # 상태 및 영점 저장용 변수
        self.prev_state = {"left": None, "right": None}
        self.prev_clutch = {"left": False, "right": False}
        self.human_zero = {"left": None, "right": None}
        self.robot_zero = {"left": None, "right": None}
        
        # 🌟 머리/허리 3초 캘리브레이션용 변수 추가
        self.head_first_msg_time = None
        self.head_zero = None
        
        self.left_pub = self.create_publisher(TwistStamped, '/left_servo/left_servo/delta_twist_cmds', 10)
        self.right_pub = self.create_publisher(TwistStamped, '/right_servo/right_servo/delta_twist_cmds', 10)
        self.head_pub = self.create_publisher(JointTrajectory, '/head_controller/joint_trajectory', 10)
        self.waist_pub = self.create_publisher(JointTrajectory, '/waist_controller/joint_trajectory', 10)
        
        # 🌟 5손가락 그리퍼(igris-c) 퍼블리셔 추가
        self.left_hand_pub = self.create_publisher(JointTrajectory, '/left_hand_controller/joint_trajectory', 10)
        self.right_hand_pub = self.create_publisher(JointTrajectory, '/right_hand_controller/joint_trajectory', 10)

        self.get_logger().info("⏳ 양팔 서보 스위치 대기 중...")
        
        self.srv_clients = {
            'left': self.create_client(ServoCommandType, '/left_servo/left_servo/switch_command_type'),
            'right': self.create_client(ServoCommandType, '/right_servo/right_servo/switch_command_type')
        }
        
        for name, client in self.srv_clients.items():
            while not client.wait_for_service(timeout_sec=1.0):
                self.get_logger().info(f"기다리는 중... ({name} 서비스)")
            
            req = ServoCommandType.Request()
            req.command_type = 1 # TWIST
            client.call_async(req)
        
        self.get_logger().info("✅ 양팔 MoveIt Servo [TWIST(1)] 세팅 완료!")
        self.timer = self.create_timer(self.dt, self.timer_callback)

    def calc_angular_velocity(self, q1, q2, dt):
        dot = q1['x']*q2['x'] + q1['y']*q2['y'] + q1['z']*q2['z'] + q1['w']*q2['w']
        sign = 1.0 if dot >= 0 else -1.0
        qdot_x = (q2['x']*sign - q1['x']) / dt
        qdot_y = (q2['y']*sign - q1['y']) / dt
        qdot_z = (q2['z']*sign - q1['z']) / dt
        qdot_w = (q2['w']*sign - q1['w']) / dt
        wx = 2.0 * (qdot_x*q1['w'] - qdot_w*q1['x'] - qdot_y*q1['z'] + qdot_z*q1['y'])
        wy = 2.0 * (qdot_y*q1['w'] - qdot_w*q1['y'] + qdot_x*q1['z'] - qdot_z*q1['x'])
        wz = 2.0 * (qdot_z*q1['w'] - qdot_w*q1['z'] - qdot_x*q1['y'] + qdot_y*q1['x'])
        return wx, wy, wz

    def euler_from_quaternion(self, q):
        x, y, z, w = q['x'], q['y'], q['z'], q['w']
        t0 = +2.0 * (w * x + y * z)
        t1 = +1.0 - 2.0 * (x * x + y * y)
        roll = math.atan2(t0, t1)
        t2 = +2.0 * (w * y - z * x)
        t2 = +1.0 if t2 > +1.0 else t2
        t2 = -1.0 if t2 < -1.0 else t2
        pitch = math.asin(t2)
        t3 = +2.0 * (w * z + x * y)
        t4 = +1.0 - 2.0 * (y * y + z * z)
        yaw = math.atan2(t3, t4)
        return roll, pitch, yaw

    def map_vr_to_ros(self, pos):
        return {'x': -pos['z'], 'y': -pos['x'], 'z': pos['y']}

    # 🌟 축 꼬임 해결용: VR 회전을 ROS 회전으로 변환하는 함수!
    def map_vr_to_ros_quat(self, rot):
        return {'x': -rot['z'], 'y': -rot['x'], 'z': rot['y'], 'w': rot['w']}

    # ==========================================
    # 🤖 1. 머리 & 허리 처리 로직 (다이렉트 조인트)
    # ==========================================
    def process_head_and_waist(self, head_data):
        global shared_foot_clutch
        # 🌟 이제 허리도 클러치가 밟혀있을 때만 움직입니다. (L 또는 R 페달 중 하나라도 밟으면 활성)
        is_clutch_active = shared_foot_clutch["left"] or shared_foot_clutch["right"]

        # 1. 축 매핑 적용
        mapped_q = self.map_vr_to_ros_quat(head_data['rot'])
        roll, pitch, yaw = self.euler_from_quaternion(mapped_q)

        # 2. 🌟 3초 영점 캘리브레이션 로직
        if self.head_zero is None:
            current_time = time.time()
            if self.head_first_msg_time is None:
                self.head_first_msg_time = current_time
                self.get_logger().info("⏳ AR 인식 완료! 3초 뒤에 영점을 잡습니다. 정면을 봐주세요!")
                return
            
            elapsed = current_time - self.head_first_msg_time
            if elapsed < 3.0:
                return
            else:
                self.head_zero = {'roll': roll, 'pitch': pitch, 'yaw': yaw}
                self.get_logger().info("🎯 영점 설정 완료! 이제 클러치를 밟으면 허리가 움직입니다.")
                return

        # 3. 영점 대비 이동량 계산
        final_roll = roll - self.head_zero['roll']
        final_pitch = pitch - self.head_zero['pitch']
        final_yaw = yaw - self.head_zero['yaw']

        # --- 🗣️ 머리 제어: 살짝 아래를 본 상태로 고정 ---
        head_msg = JointTrajectory()
        head_msg.header.stamp = self.get_clock().now().to_msg()
        head_msg.joint_names = ['29_Joint_Neck_Yaw', '30_Joint_Neck_Pitch'] 
        
        head_point = JointTrajectoryPoint()
        # Pitch를 약 15~20도(0.3 rad) 정도 아래를 보게 고정 (필요시 값 조절 가능)
        fixed_head_pitch = 0.3 
        head_point.positions = [0.0, float(fixed_head_pitch)] 
        head_point.time_from_start.sec = 0
        head_point.time_from_start.nanosec = 50000000 
        head_msg.points.append(head_point)
        self.head_pub.publish(head_msg)

        # --- 🩻 허리 제어: 클러치가 밟혔을 때만 모든 회전 적용 ---
        if is_clutch_active:
            waist_msg = JointTrajectory()
            waist_msg.header.stamp = self.get_clock().now().to_msg()
            waist_msg.joint_names = ['0_Joint_Waist_Yaw', '1_Joint_Waist_Roll', '2_Joint_Waist_Pitch']
            
            waist_point = JointTrajectoryPoint()
            # 머리의 Yaw(좌우), Roll(옆으로 기울기), Pitch(앞뒤)를 모두 허리로 매핑
            # 로봇 허리 사양에 따라 Yaw, Roll, Pitch 순서와 부호를 매칭
            waist_point.positions = [float(final_yaw), float(final_roll), float(final_pitch)]
            waist_point.time_from_start.sec = 0
            waist_point.time_from_start.nanosec = 50000000
            
            waist_msg.points.append(waist_point)
            self.waist_pub.publish(waist_msg)


    # ==========================================
    # 🦾 2. 팔 처리 로직 ((B)방식 P-제어)
    # ==========================================
    def process_arm(self, side, xr_data, twist_pub, ee_frame):
        global shared_foot_clutch
        # 🌟 이제 XR 데이터의 클러치 대신 풋 페달 상태를 사용합니다.
        curr_clutch = shared_foot_clutch[side]
        
        vr_rot = xr_data['rot']
        vr_pos = self.map_vr_to_ros(xr_data['pos']) 

        if curr_clutch and not self.prev_clutch[side]:
            try:
                trans = self.tf_buffer.lookup_transform('base_link', ee_frame, rclpy.time.Time())
                self.robot_zero[side] = trans.transform.translation
                self.human_zero[side] = vr_pos
                self.prev_state[side] = {'rot': vr_rot}
            except Exception as e:
                curr_clutch = False 
            self.prev_clutch[side] = curr_clutch
            return

        if curr_clutch and self.robot_zero[side] and self.human_zero[side]:
            try:
                curr_robot_trans = self.tf_buffer.lookup_transform('base_link', ee_frame, rclpy.time.Time())
                curr_robot_pos = curr_robot_trans.transform.translation

                target_x = self.robot_zero[side].x + (vr_pos['x'] - self.human_zero[side]['x'])
                target_y = self.robot_zero[side].y + (vr_pos['y'] - self.human_zero[side]['y'])
                target_z = self.robot_zero[side].z + (vr_pos['z'] - self.human_zero[side]['z'])

                p_gain = 4.0 
                msg = TwistStamped()
                msg.header.stamp = self.get_clock().now().to_msg()
                msg.header.frame_id = 'base_link' 

                msg.twist.linear.x = (target_x - curr_robot_pos.x) * p_gain
                msg.twist.linear.y = (target_y - curr_robot_pos.y) * p_gain
                msg.twist.linear.z = (target_z - curr_robot_pos.z) * p_gain

                prev_rot = self.prev_state[side]['rot']
                wx_vr, wy_vr, wz_vr = self.calc_angular_velocity(prev_rot, vr_rot, self.dt)
                
                deadband = 0.3
                if abs(wx_vr) < deadband: wx_vr = 0.0
                if abs(wy_vr) < deadband: wy_vr = 0.0
                if abs(wz_vr) < deadband: wz_vr = 0.0

                a_gain = 1.0
                msg.twist.angular.x = float(-wz_vr * a_gain)
                msg.twist.angular.y = float(-wx_vr * a_gain)
                msg.twist.angular.z = float(wy_vr * a_gain)

                twist_pub.publish(msg)
                self.prev_state[side]['rot'] = vr_rot

            except Exception as e:
                pass

        self.prev_clutch[side] = curr_clutch

    # ==========================================
    # 🖐️ 3. 손가락 처리 로직 (5손가락 igris-c)
    # ==========================================
    def process_fingers(self, side, xr_data, hand_pub):
        if 'fingers' not in xr_data or not xr_data['fingers']:
            return

        fingers = xr_data['fingers']
        joint_msg = JointTrajectory()
        joint_msg.header.stamp = self.get_clock().now().to_msg()
        
        # igris-c 5손가락 조인트 이름 (추측 기반, 실제 이름으로 수정 필요)
        suffix = "Left" if side == "left" else "Right"
        joint_msg.joint_names = [
            f'31_Joint_Thumb_{suffix}', 
            f'32_Joint_Index_{suffix}', 
            f'33_Joint_Middle_{suffix}', 
            f'34_Joint_Ring_{suffix}', 
            f'35_Joint_Pinky_{suffix}'
        ] if side == "left" else [
            f'36_Joint_Thumb_{suffix}', 
            f'37_Joint_Index_{suffix}', 
            f'38_Joint_Middle_{suffix}', 
            f'39_Joint_Ring_{suffix}', 
            f'40_Joint_Pinky_{suffix}'
        ]

        # 거리 -> 각도 매핑 (Tip-to-Wrist Distance)
        # 0.18m(최대한 폄) -> 0.0 rad, 0.05m(최대한 굽힘) -> 1.57 rad (90도)
        min_dist = 0.05
        max_dist = 0.18
        
        positions = []
        xr_joint_map = {
            'thumb': 'thumb-tip',
            'index': 'index-finger-tip',
            'middle': 'middle-finger-tip',
            'ring': 'ring-finger-tip',
            'pinky': 'pinky-finger-tip'
        }

        for f_key in ['thumb', 'index', 'middle', 'ring', 'pinky']:
            xr_name = xr_joint_map[f_key]
            if xr_name in fingers:
                dist = fingers[xr_name]
                # 선형 매핑
                angle = (max_dist - dist) / (max_dist - min_dist) * 1.57
                angle = max(0.0, min(1.57, angle))
                positions.append(float(angle))
            else:
                positions.append(0.0)

        point = JointTrajectoryPoint()
        point.positions = positions
        point.time_from_start.sec = 0
        point.time_from_start.nanosec = 50000000 # 50ms
        
        joint_msg.points.append(point)
        hand_pub.publish(joint_msg)

    def timer_callback(self):
        global shared_xr_data
        
        # 머리/허리 업데이트
        if shared_xr_data["head"]:
            self.process_head_and_waist(shared_xr_data["head"])
            
        # 양팔 업데이트
        if shared_xr_data["left"]:
            self.process_arm("left", shared_xr_data["left"], self.left_pub, "Left_Hand")
            self.process_fingers("left", shared_xr_data["left"], self.left_hand_pub)
        if shared_xr_data["right"]:
            self.process_arm("right", shared_xr_data["right"], self.right_pub, "Right_Hand")
            self.process_fingers("right", shared_xr_data["right"], self.right_hand_pub)

def main(args=None):
    rclpy.init(args=args)
    
    node = FullBodyTeleopNode()

    # 🌟 풋 페달 모니터링 스레드 시작 (노드 로거 전달)
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

if __name__ == '__main__':
    main()
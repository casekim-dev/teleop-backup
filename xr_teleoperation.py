import numpy as np
import time
import os
import sys
from multiprocessing import shared_memory
from scipy.spatial.transform import Rotation as R

# --- [ROS2 관련 패키지] ---
import rclpy
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Float64  # 하체 Z축(높이) 제어용 메시지

# 로깅 설정
import logging_mp
logging_mp.basic_config(level=logging_mp.INFO)
logger = logging_mp.get_logger(__name__)

# 경로 설정
current_dir = os.path.dirname(os.path.abspath(__file__))
parent_dir = os.path.dirname(current_dir)
sys.path.append(parent_dir)

from televuer import TeleVuerWrapper

class XRTeloperator:
    def __init__(self, left_arm_agent=None, right_arm_agent=None, left_tool_agent=None, right_tool_agent=None):
        self.left_arm_agent = left_arm_agent
        self.right_arm_agent = right_arm_agent
        self.left_tool_agent = left_tool_agent
        self.right_tool_agent = right_tool_agent
        self.frequency = 50

        # --- [스케일링 팩터] ---
        self.arm_translation_scale = 2.0  # 로봇 팔 작업 반경 확장 (위치에만 적용)
        self.base_z_scale = 1.0           # 스쿼트(높낮이) 비율 (일단 1:1 매핑)

        # --- [ROS2 노드 및 퍼블리셔 초기화] ---
        if not rclpy.ok():
            rclpy.init()
        self.ros_node = rclpy.create_node('xr_teleop_wholebody_bridge')
        
        # 양팔 6D Pose 퍼블리셔
        self.pub_left_target = self.ros_node.create_publisher(PoseStamped, '/igris/left_arm/target_pose', 10)
        self.pub_right_target = self.ros_node.create_publisher(PoseStamped, '/igris/right_arm/target_pose', 10)
        
        # 하체(Base) 높이 제어용 퍼블리셔 (SONIC이 받아서 무릎을 굽히게 될 타겟값)
        self.pub_base_z_target = self.ros_node.create_publisher(Float64, '/igris/base/target_z', 10)
        
        # 이미지 및 공유 메모리 설정
        self.img_shape = (480, 640, 3)
        self.tv_img_shm = shared_memory.SharedMemory(
            create=True, 
            size=np.prod(self.img_shape) * np.uint8().itemsize
        )
        
        # TeleVuer 초기화
        self.tv_wrapper = TeleVuerWrapper(
            binocular=(self.img_shape[1] > 640),
            use_hand_tracking=True,
            img_shape=self.img_shape,
            img_shm_name=self.tv_img_shm.name,
            return_state_data=True,
        )
        
        # --- [이전 프레임 저장 변수 (4x4 Matrix 원본)] ---
        self.prev_l_mat = None
        self.prev_r_mat = None
        self.prev_h_mat = None  # Headset(HMD) 이전 포즈

        # --- [로봇의 현재 절대 상태] ---
        # 실제 로봇의 초기 Base Z 높이 (예: 지면으로부터 0.8m) - 하드웨어 스펙에 맞춰 수정 필요
        self.robot_base_z = 0.8 
        self.is_dual_arm = self.left_arm_agent is not None and self.right_arm_agent is not None and self.left_arm_agent.id == self.right_arm_agent.id

    def _publish_ros2_pose(self, publisher, target_pos, target_rot_matrix, frame_id="base_link"):
        """계산된 Arm Target Pose를 ROS2 토픽으로 전송"""
        msg = PoseStamped()
        msg.header.stamp = self.ros_node.get_clock().now().to_msg()
        msg.header.frame_id = frame_id
        
        # Translation
        msg.pose.position.x = float(target_pos[0])
        msg.pose.position.y = float(target_pos[1])
        msg.pose.position.z = float(target_pos[2])
        
        # Rotation (Matrix -> Quaternion)
        quat = R.from_matrix(target_rot_matrix).as_quat() # [x, y, z, w]
        msg.pose.orientation.x = float(quat[0])
        msg.pose.orientation.y = float(quat[1])
        msg.pose.orientation.z = float(quat[2])
        msg.pose.orientation.w = float(quat[3])
        
        publisher.publish(msg)

    def _get_ee_matrix(self, agent, side='ee'):
        """에이전트에서 현재 로봇의 EE 포즈를 가져와 4x4 행렬로 변환"""
        if agent is None:
            return np.eye(4)
        try:
            ee_pos_dict = agent.get_ee_position()
            curr_6d = ee_pos_dict[side]
            pos = np.array(curr_6d[:3])
            rot_mat = R.from_rotvec(np.array(curr_6d[3:])).as_matrix()
            T = np.eye(4)
            T[:3, :3] = rot_mat
            T[:3, 3] = pos
            return T
        except:
            return np.eye(4)

    def run(self, task_control, socketio_instance=None, log_emit_id=None):
        logger.info("TeleOp Controller 루프 시작")
        if socketio_instance and log_emit_id:
            socketio_instance.emit(log_emit_id, {'log': 'XR Connected. Whole-body mode active.', 'type': 'stdout'})
        
        # 로봇 팔 초기 위치 확보
        robot_l_mat = self._get_ee_matrix(self.left_arm_agent, 'left' if self.is_dual_arm else 'ee')
        robot_r_mat = self._get_ee_matrix(self.right_arm_agent if not self.is_dual_arm else self.left_arm_agent, 'right' if self.is_dual_arm else 'ee')

        try:
            while not task_control.get('stop', False):
                loop_start = time.time()
                
                # 1. XR 기기 데이터 획득 (양손 + 헤드셋)
                tele_data = self.tv_wrapper.get_motion_state_data()
                curr_l_mat = tele_data.left_arm_pose
                curr_r_mat = tele_data.right_arm_pose
                
                # 주의: televuer 버전에 따라 head_pose 변수명이 다를 수 있음 (hmd_pose 등)
                curr_h_mat = getattr(tele_data, 'head_pose', getattr(tele_data, 'hmd_pose', None))

                if task_control.get('read', False):
                    if self.prev_l_mat is not None and self.prev_r_mat is not None and self.prev_h_mat is not None:
                        
                        # ==========================================================
                        # [하체 제어 파트] 헤드셋 Z축(높이) Delta 추출 및 퍼블리시
                        # ==========================================================
                        delta_z_head = (curr_h_mat[2, 3] - self.prev_h_mat[2, 3]) * self.base_z_scale
                        
                        # 노이즈 필터링 (순간적으로 30cm 이상 튀면 무시)
                        if abs(delta_z_head) < 0.3:
                            self.robot_base_z += delta_z_head
                            
                            # ROS2 하체 높이 타겟 퍼블리시 (SONIC 입력용)
                            msg_z = Float64()
                            msg_z.data = float(self.robot_base_z)
                            self.pub_base_z_target.publish(msg_z)

                        # ==========================================================
                        # [상체 제어 파트] 양손 6D Pose 연산 (안전한 회전 행렬 방식)
                        # ==========================================================
                        delta_pos_l = (curr_l_mat[:3, 3] - self.prev_l_mat[:3, 3]) * self.arm_translation_scale
                        delta_pos_r = (curr_r_mat[:3, 3] - self.prev_r_mat[:3, 3]) * self.arm_translation_scale

                        R_curr_l, R_prev_l = R.from_matrix(curr_l_mat[:3, :3]), R.from_matrix(self.prev_l_mat[:3, :3])
                        R_curr_r, R_prev_r = R.from_matrix(curr_r_mat[:3, :3]), R.from_matrix(self.prev_r_mat[:3, :3])
                        
                        delta_R_l = R_curr_l * R_prev_l.inv()
                        delta_R_r = R_curr_r * R_prev_r.inv()

                        # 튐 방지
                        if np.linalg.norm(delta_pos_l) < 0.5 and np.linalg.norm(delta_pos_r) < 0.5:
                            # 로봇 포즈 업데이트
                            robot_l_mat[:3, 3] += delta_pos_l
                            robot_r_mat[:3, 3] += delta_pos_r
                            robot_l_mat[:3, :3] = (delta_R_l * R.from_matrix(robot_l_mat[:3, :3])).as_matrix()
                            robot_r_mat[:3, :3] = (delta_R_r * R.from_matrix(robot_r_mat[:3, :3])).as_matrix()

                            # ROS2 양팔 타겟 퍼블리시 (SONIC/IK 입력용)
                            self._publish_ros2_pose(self.pub_left_target, robot_l_mat[:3, 3], robot_l_mat[:3, :3])
                            self._publish_ros2_pose(self.pub_right_target, robot_r_mat[:3, 3], robot_r_mat[:3, :3])

                            # (옵션) 기존 파이썬 객체 IK 연동 유지
                            target_l_6d = np.concatenate([robot_l_mat[:3, 3], R.from_matrix(robot_l_mat[:3, :3]).as_rotvec()])
                            target_r_6d = np.concatenate([robot_r_mat[:3, 3], R.from_matrix(robot_r_mat[:3, :3]).as_rotvec()])
                            
                            if self.is_dual_arm:
                                self.left_arm_agent.move_ee_step({'left': target_l_6d, 'right': target_r_6d})
                            elif self.left_arm_agent and self.right_arm_agent:
                                self.left_arm_agent.move_ee_step({'ee': target_l_6d})
                                self.right_arm_agent.move_ee_step({'ee': target_r_6d})

                # 이전 포즈 업데이트 (헤드셋 추가)
                self.prev_l_mat = curr_l_mat
                self.prev_r_mat = curr_r_mat
                self.prev_h_mat = curr_h_mat

                # ROS2 노드 스핀 처리
                rclpy.spin_once(self.ros_node, timeout_sec=0.0)

                # 주기 제어 (50Hz)
                elapsed = time.time() - loop_start
                time.sleep(max(0, (1.0 / self.frequency) - elapsed))

        except Exception as e:
            logger.error(f"Controller Loop Error: {e}")
        finally:
            self.cleanup()

    def cleanup(self):
        """자원 해제"""
        try:
            if self.ros_node:
                self.ros_node.destroy_node()
            if rclpy.ok():
                rclpy.shutdown()
            self.tv_img_shm.close()
            self.tv_img_shm.unlink()
            logger.info("ROS2 브릿지 클린업 완료")
        except:
            pass
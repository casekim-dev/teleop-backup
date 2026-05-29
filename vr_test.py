import rclpy
from rclpy.node import Node
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped
import numpy as np
from scipy.spatial.transform import Rotation as R
import os, sys
import time
from multiprocessing import shared_memory

# 1. 선배들의 방식대로 경로 설정 (시스템에 깔린 televuer를 우선 사용!)
# 로컬 src를 sys.path에 넣지 않음으로써 시스템 기본 패키지를 불러오게 유도함
from televuer import TeleVuerWrapper

class VRtoRVizNode(Node):
    def __init__(self):
        super().__init__('vr_to_rviz_node')
        
        self.tf_broadcaster = TransformBroadcaster(self)

        # 2. 이미지 공유 메모리 설정 (xr_teleoperation.py 그대로!)
        self.img_shape = (480, 640, 3)
        try:
            self.tv_img_shm = shared_memory.SharedMemory(
                create=True, 
                size=np.prod(self.img_shape) * np.uint8().itemsize
            )
        except:
            # 이미 있으면 기존 것 활용
            self.tv_img_shm = shared_memory.SharedMemory(name='vuer_img_shm')

        # 3. TeleVuerWrapper 초기화 (xr_teleoperation.py 1:1 복사)
        try:
            self.tv_wrapper = TeleVuerWrapper(
                binocular=False,
                use_hand_tracking=True,
                img_shape=self.img_shape,
                img_shm_name=self.tv_img_shm.name,
                return_state_data=True, # 선배들의 필승 옵션!
            )
            self.get_logger().info(f"✅ 선배들의 TeleVuer 로드 성공!")
        except Exception as e:
            self.get_logger().error(f"❌ 초기화 실패: {e}")
            sys.exit(1)

        self.timer = self.create_timer(0.02, self.timer_callback)
        self.last_print_time = self.get_clock().now()

    def broadcast_tf(self, matrix, child_frame_name):
        if matrix is None:
            return

        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'vr_world'
        t.child_frame_id = child_frame_name

        t.transform.translation.x = float(matrix[0, 3])
        t.transform.translation.y = float(matrix[1, 3])
        t.transform.translation.z = float(matrix[2, 3])

        try:
            quat = R.from_matrix(matrix[:3, :3]).as_quat()
            t.transform.rotation.x = float(quat[0])
            t.transform.rotation.y = float(quat[1])
            t.transform.rotation.z = float(quat[2])
            t.transform.rotation.w = float(quat[3])
            self.tf_broadcaster.sendTransform(t)
        except:
            pass

    def timer_callback(self):
        # 4. XR 기기 데이터 획득 (선배들이 쓰던 그 함수!)
        try:
            tele_data = self.tv_wrapper.get_motion_state_data()
        except:
            return
            
        if tele_data:
            # 선배들이 데이터 뽑아오던 필드명 그대로!
            curr_l_mat = tele_data.left_arm_pose
            curr_r_mat = tele_data.right_arm_pose
            curr_h_mat = getattr(tele_data, 'head_pose', getattr(tele_data, 'hmd_pose', None))

            if curr_h_mat is not None:
                self.broadcast_tf(curr_h_mat, 'quest_head')
                self.broadcast_tf(curr_l_mat, 'quest_left_hand')
                self.broadcast_tf(curr_r_mat, 'quest_right_hand')

                # 디버깅: 숫자가 변하는지 터미널에 실시간으로!
                now = self.get_clock().now()
                if (now - self.last_print_time).nanoseconds > 5e8: # 0.5초마다
                    self.get_logger().info(f"📊 Head Z: {curr_h_mat[2, 3]:.3f} (Wait for Enter VR...)")
                    self.last_print_time = now

def main(args=None):
    rclpy.init(args=args)
    node = VRtoRVizNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("🛑 종료")
    finally:
        node.tv_wrapper.close()
        node.tv_img_shm.close()
        node.tv_img_shm.unlink()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
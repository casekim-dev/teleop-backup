import rclpy
from rclpy.node import Node
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped, Point
from visualization_msgs.msg import Marker
import numpy as np
from scipy.spatial.transform import Rotation as R

# televuer 라이브러리 경로
import os, sys
current_dir = os.path.dirname(os.path.abspath(__file__))
parent_dir = os.path.dirname(current_dir)
sys.path.append(parent_dir)
from televuer import TeleVuerWrapper

class VRSkeletonVisualizer(Node):
    def __init__(self):
        super().__init__('vr_skeleton_visualizer')
        
        self.tf_broadcaster = TransformBroadcaster(self)
        self.marker_pub = self.create_publisher(Marker, '/vr_skeleton_marker', 10)

        # 🔥 시그니처 분석 기반 완벽한 초기화
        self.tv_wrapper = TeleVuerWrapper(
            use_hand_tracking=True, 
            binocular=False,
            display_mode='pass-through',
            return_hand_rot_data=True,
            webrtc=True
        )

        self.timer = self.create_timer(0.02, self.run_loop)
        self.get_logger().info("🚀 VR 막대기 시각화 노드 시작! 퀘스트로 접속하고 RViz2를 켜주세요.")

    def publish_tf(self, matrix, frame_name):
        if matrix is None: return
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'vr_world'
        t.child_frame_id = frame_name

        t.transform.translation.x = float(matrix[0, 3])
        t.transform.translation.y = float(matrix[1, 3])
        t.transform.translation.z = float(matrix[2, 3])

        quat = R.from_matrix(matrix[:3, :3]).as_quat()
        t.transform.rotation.x = float(quat[0])
        t.transform.rotation.y = float(quat[1])
        t.transform.rotation.z = float(quat[2])
        t.transform.rotation.w = float(quat[3])
        self.tf_broadcaster.sendTransform(t)

    def draw_skeleton_lines(self, head_mat, left_mat, right_mat):
        if head_mat is None or left_mat is None or right_mat is None: return

        marker = Marker()
        marker.header.frame_id = "vr_world"
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "vr_arms"
        marker.id = 0
        marker.type = Marker.LINE_LIST
        marker.action = Marker.ADD
        marker.scale.x = 0.05

        marker.color.r = 0.0
        marker.color.g = 1.0
        marker.color.b = 0.0
        marker.color.a = 1.0

        head_pt = Point(x=float(head_mat[0,3]), y=float(head_mat[1,3]), z=float(head_mat[2,3]))
        left_pt = Point(x=float(left_mat[0,3]), y=float(left_mat[1,3]), z=float(left_mat[2,3]))
        right_pt = Point(x=float(right_mat[0,3]), y=float(right_mat[1,3]), z=float(right_mat[2,3]))

        marker.points.extend([head_pt, left_pt, head_pt, right_pt])
        self.marker_pub.publish(marker)

    def run_loop(self):
        tele_data = self.tv_wrapper.get_tele_data()
        if tele_data:
            self.get_logger().info(f"✋ 왼손 높이(Z): {tele_data.left_wrist_pose[2, 3]:.3f}", throttle_duration_sec=0.5)
            
            h_mat = tele_data.head_pose
            l_mat = tele_data.left_wrist_pose
            r_mat = tele_data.right_wrist_pose

            # 3. RViz에 TF(좌표계 화살표) 쏘기
            self.publish_tf(h_mat, 'head')
            self.publish_tf(l_mat, 'left_hand')
            self.publish_tf(r_mat, 'right_hand')

            # 4. 형광 초록색 막대기(뼈대) 그리기
            self.draw_skeleton_lines(h_mat, l_mat, r_mat)
        else:
            # 🔥 [추가] 퀘스트가 데이터를 안 보내고 있으면 경고 띄우기!
            self.get_logger().warning("⏳ 퀘스트 데이터 대기 중...", throttle_duration_sec=1.0)


def main(args=None):
    rclpy.init(args=args)
    node = VRSkeletonVisualizer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("종료합니다.")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
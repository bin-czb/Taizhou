import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import PoseStamped
import numpy as np
from scipy.spatial.transform import Rotation as R
from collections import deque
import copy

# 新增 tf2 相关的库
import tf2_ros
from tf2_ros import TransformException

class OdomFusionNode(Node):
    def __init__(self):
        super().__init__('odom_fusion_node')
        
        # --- TF 监听模块 ---
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.body_T_base = None  # 存储 base_link 到 body 的变换矩阵
        # 设置一个定时器，每秒尝试获取一次 TF，获取成功后会自我销毁
        self.tf_timer = self.create_timer(1.0, self.get_extrinsic_tf)
        
        # 订阅与发布
        self.sub_lio = self.create_subscription(Odometry, '/fastlio2/lio_odom', self.lio_cb, 10)
        self.sub_odom = self.create_subscription(Odometry, '/odom', self.odom_cb, 50)
        
        self.pub_fused = self.create_publisher(Odometry, '/fused_odom', 10)
        self.pub_fused_path = self.create_publisher(Path, '/fused_path', 10) 
        self.pub_odom_path = self.create_publisher(Path, '/odom_path', 10)
        
        # 状态变量 
        self.odom_queue = deque(maxlen=200) 
        self.map_T_base_j = None   
        self.odom_T_base_j = None  
        self.time_j = None         
        
        # 轨迹维护
        self.fused_path_msg = Path()
        self.fused_path_msg.header.frame_id = "map"
        self.odom_path_msg = Path()
        self.odom_path_msg.header.frame_id = "map" 

    def get_extrinsic_tf(self):
        """获取传感器外参: base_link 到 body 的 TF"""
        if self.body_T_base is not None:
            return
            
        try:
            # target_frame='body', source_frame='base_link'
            # 注意：请确保你的实际 TF 树中存在这两个 frame，如果 LIO 的坐标系叫其他名字(如 camera_init)，请修改 'body'
            tf_msg = self.tf_buffer.lookup_transform('body', 'base_link', rclpy.time.Time())
            
            t = [tf_msg.transform.translation.x, tf_msg.transform.translation.y, tf_msg.transform.translation.z]
            q = [tf_msg.transform.rotation.x, tf_msg.transform.rotation.y, tf_msg.transform.rotation.z, tf_msg.transform.rotation.w]
            
            mat = np.eye(4)
            mat[:3, :3] = R.from_quat(q).as_matrix()
            mat[:3, 3] = t
            
            self.body_T_base = mat
            self.get_logger().info('✅ 成功获取 body 与 base_link 之间的静态外参 TF！')
            self.tf_timer.cancel() # 获取成功，取消定时器
            
        except TransformException as e:
            self.get_logger().info(f'等待外参 TF (body <- base_link): {e}')

    def get_time_sec(self, header):
        return header.stamp.sec + header.stamp.nanosec * 1e-9

    def pose_to_matrix(self, pose_msg):
        t = [pose_msg.position.x, pose_msg.position.y, pose_msg.position.z]
        q = [pose_msg.orientation.x, pose_msg.orientation.y, pose_msg.orientation.z, pose_msg.orientation.w]
        mat = np.eye(4)
        mat[:3, :3] = R.from_quat(q).as_matrix()
        mat[:3, 3] = t
        return mat

    def matrix_to_pose(self, mat, pose_msg):
        pose_msg.position.x, pose_msg.position.y, pose_msg.position.z = mat[:3, 3]
        q = R.from_matrix(mat[:3, :3]).as_quat()
        pose_msg.orientation.x, pose_msg.orientation.y, pose_msg.orientation.z, pose_msg.orientation.w = q
        return pose_msg

    def inv_se3(self, mat):
        inv_mat = np.eye(4)
        R_inv = mat[:3, :3].T
        inv_mat[:3, :3] = R_inv
        inv_mat[:3, 3] = -R_inv @ mat[:3, 3]
        return inv_mat

    def lio_cb(self, msg):
        """更新 j 时刻的基准位姿 (10Hz)"""
        # 如果外参还没拿到，先不处理 LIO 数据
        if self.body_T_base is None:
            return

        self.time_j = self.get_time_sec(msg.header)
        
        # 1. 获取 LIO 原始的 map_T_body
        map_T_body_j = self.pose_to_matrix(msg.pose.pose)
        
        # 2. 坐标系转换：将其转为 map_T_base
        self.map_T_base_j = map_T_body_j @ self.body_T_base
        
        if not self.odom_queue:
            return

        best_odom_mat = None
        min_dt = float('inf')
        
        for t_odom, mat in list(self.odom_queue):
            dt = abs(t_odom - self.time_j)
            if dt < min_dt:
                min_dt = dt
                best_odom_mat = mat
                
        if best_odom_mat is not None:
            if min_dt > 0.1:
                self.get_logger().warning(f'LIO与Odom时间对齐偏差较大: {min_dt:.3f}s', throttle_duration_sec=5.0)
            self.odom_T_base_j = best_odom_mat

    def odom_cb(self, msg):
        """计算并发布 i 时刻的融合结果 (50Hz)"""
        time_i = self.get_time_sec(msg.header)
        odom_T_base_i = self.pose_to_matrix(msg.pose.pose)
        self.odom_queue.append((time_i, odom_T_base_i))
        
        self.update_path(self.odom_path_msg, msg.pose.pose, msg.header.stamp, self.pub_odom_path)

        if self.map_T_base_j is None or self.odom_T_base_j is None:
            return

        # 核心计算 
        base_j_T_base_i = self.inv_se3(self.odom_T_base_j) @ odom_T_base_i
        map_T_base_i = self.map_T_base_j @ base_j_T_base_i
        
        fused_msg = copy.deepcopy(msg)
        fused_msg.header.frame_id = "map"
        fused_msg.pose.pose = self.matrix_to_pose(map_T_base_i, fused_msg.pose.pose)
        self.pub_fused.publish(fused_msg)

        self.update_path(self.fused_path_msg, fused_msg.pose.pose, msg.header.stamp, self.pub_fused_path)

    def update_path(self, path_msg, pose, stamp, publisher):
        pose_stamped = PoseStamped()
        pose_stamped.header.stamp = stamp
        pose_stamped.header.frame_id = path_msg.header.frame_id
        pose_stamped.pose = pose
        
        # 保留了你取消降采样的逻辑
        path_msg.poses.append(pose_stamped)
        path_msg.header.stamp = stamp
        publisher.publish(path_msg)

def main(args=None):
    rclpy.init(args=args)
    node = OdomFusionNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
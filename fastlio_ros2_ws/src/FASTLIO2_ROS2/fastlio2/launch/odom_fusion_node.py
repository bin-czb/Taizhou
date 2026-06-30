import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import PoseStamped, Pose
import numpy as np
from scipy.spatial.transform import Rotation as R

# 新增 tf2 相关的库
import tf2_ros
from tf2_ros import TransformException

# 新增 message_filters 用于时间戳对齐过滤
import message_filters

class OdomFusionNode(Node):
    def __init__(self):
        super().__init__('odom_fusion_node')
        
        # --- 1. TF 监听模块 ---
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.body_T_base = None  
        self.tf_timer = self.create_timer(1.0, self.get_extrinsic_tf)
        
        # --- 2. 基于 message_filters 的时间戳自动对齐 (10Hz) ---
        self.sub_lio_sync = message_filters.Subscriber(self, Odometry, '/fastlio2/lio_odom')
        self.sub_odom_sync = message_filters.Subscriber(self, Odometry, '/odom')
        
        self.ts = message_filters.ApproximateTimeSynchronizer(
            [self.sub_lio_sync, self.sub_odom_sync], queue_size=50, slop=0.1)
        self.ts.registerCallback(self.sync_cb)
        
        # --- 3. 用于高频发布的独立订阅器 (50Hz) ---
        self.sub_odom_fast = self.create_subscription(Odometry, '/odom', self.odom_fast_cb, 50)
        
        # --- 4. 发布器 ---
        self.pub_fused = self.create_publisher(Odometry, '/fused_odom', 50)
        self.pub_fused_path = self.create_publisher(Path, '/fused_path', 2) 
        self.pub_odom_path = self.create_publisher(Path, '/odom_path', 2)
        
        # LIO 相关的发布器
        self.pub_lio_path = self.create_publisher(Path, '/lio_path', 2) 
        self.pub_lio_odom = self.create_publisher(Odometry, '/lio_base_odom', 10) # 新增：LIO 转换到 base_link 的里程计
        
        # --- 5. 状态变量 ---
        self.map_T_base_j = None   
        self.odom_T_base_j = None  
        self.last_sync_time = None 
        
        # --- 6. 轨迹维护 ---
        self.fused_path_msg = Path()
        self.fused_path_msg.header.frame_id = "map"
        self.odom_path_msg = Path()
        self.odom_path_msg.header.frame_id = "map" 
        self.lio_path_msg = Path()       
        self.lio_path_msg.header.frame_id = "map" 

    def get_time_sec(self, header):
        return header.stamp.sec + header.stamp.nanosec * 1e-9

    def get_extrinsic_tf(self):
        if self.body_T_base is not None:
            return
            
        try:
            tf_msg = self.tf_buffer.lookup_transform('body', 'base_link', rclpy.time.Time())
            
            t = [tf_msg.transform.translation.x, tf_msg.transform.translation.y, tf_msg.transform.translation.z]
            q = [tf_msg.transform.rotation.x, tf_msg.transform.rotation.y, tf_msg.transform.rotation.z, tf_msg.transform.rotation.w]
            
            mat = np.eye(4)
            mat[:3, :3] = R.from_quat(q).as_matrix()
            mat[:3, 3] = t
            
            self.body_T_base = mat
            self.get_logger().info('✅ 成功获取 body 与 base_link 之间的静态外参 TF！')
            self.tf_timer.cancel() 
            
        except TransformException as e:
            self.get_logger().info(f'等待外参 TF (body <- base_link): {e}')

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

    def sync_cb(self, lio_msg, odom_msg):
        if self.body_T_base is None:
            return
            
        self.last_sync_time = self.get_time_sec(odom_msg.header)
        
        # 1. 计算出 LIO 在 base_link 下的矩阵 map_T_base_j
        map_T_body_j = self.pose_to_matrix(lio_msg.pose.pose)
        self.map_T_base_j = map_T_body_j @ self.body_T_base
        
        self.odom_T_base_j = self.pose_to_matrix(odom_msg.pose.pose)

        # =========================================================
        # 将 LIO 的 base_link 位姿还原，并发布纯 LIO 轨迹及里程计
        # =========================================================
        lio_base_pose = Pose()
        lio_base_pose = self.matrix_to_pose(self.map_T_base_j, lio_base_pose)
        
        # 1. 发布 LIO 轨迹
        self.update_path(self.lio_path_msg, lio_base_pose, lio_msg.header.stamp, self.pub_lio_path)

        # 2. 构造并发布 LIO 的 base_link 里程计 (主要用于观察朝向)
        lio_base_odom = Odometry()
        lio_base_odom.header.stamp = lio_msg.header.stamp
        lio_base_odom.header.frame_id = "map"
        lio_base_odom.child_frame_id = "base_link"
        lio_base_odom.pose.pose = lio_base_pose
        self.pub_lio_odom.publish(lio_base_odom)

    def odom_fast_cb(self, msg):
        current_time = self.get_time_sec(msg.header)
        odom_T_base_i = self.pose_to_matrix(msg.pose.pose)
        
        # 发送纯 Odom 轨迹
        self.update_path(self.odom_path_msg, msg.pose.pose, msg.header.stamp, self.pub_odom_path)

        if self.map_T_base_j is None or self.odom_T_base_j is None or self.last_sync_time is None:
            self.get_logger().warning('等待 Fast-LIO2 与 Odom 初始对齐...', throttle_duration_sec=2.0)
            return

        time_since_sync = current_time - self.last_sync_time
        if time_since_sync > 0.3:
            self.get_logger().warning(
                f'Fast-LIO2 与 Odom 已有 {time_since_sync:.2f} 秒未对齐！当前处于纯里程计漂移状态！', 
                throttle_duration_sec=1.0
            )

        base_j_T_base_i = self.inv_se3(self.odom_T_base_j) @ odom_T_base_i
        map_T_base_i = self.map_T_base_j @ base_j_T_base_i
        
        # 手动构建融合后的新消息 
        fused_msg = Odometry()
        fused_msg.header.stamp = msg.header.stamp
        fused_msg.header.frame_id = "map"
        fused_msg.child_frame_id = msg.child_frame_id
        fused_msg.twist = msg.twist 
        fused_msg.pose.pose = self.matrix_to_pose(map_T_base_i, fused_msg.pose.pose)
        
        self.pub_fused.publish(fused_msg)

        # 发送融合轨迹
        self.update_path(self.fused_path_msg, fused_msg.pose.pose, msg.header.stamp, self.pub_fused_path)

    def update_path(self, path_msg, pose, stamp, publisher):
        if len(path_msg.poses) > 0:
            last_pose = path_msg.poses[-1].pose
            dist = np.sqrt((pose.position.x - last_pose.position.x)**2 + 
                           (pose.position.y - last_pose.position.y)**2)
            if dist < 0.05:
                return
                
        if len(path_msg.poses) > 1000:
            path_msg.poses.pop(0)

        pose_stamped = PoseStamped()
        pose_stamped.header.stamp = stamp
        pose_stamped.header.frame_id = path_msg.header.frame_id
        pose_stamped.pose = pose
        
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
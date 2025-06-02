#!/usr/bin/env python3

import rospy
import tf2_ros
import geometry_msgs.msg
import sensor_msgs.msg
import math
from tf2_msgs.msg import TFMessage

# ANSI 顏色代碼
class Colors:
    RED = '\033[91m'      # 紅色
    GREEN = '\033[92m'    # 綠色
    BLUE = '\033[94m'     # 藍色
    ENDC = '\033[0m'      # 結束顏色

class RobotArmTFListener:
    def __init__(self):
        rospy.init_node('robot_arm_tf_listener', anonymous=True)
        
        # 初始化 TF 緩衝區和監聽器
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        
        # 訂閱關節狀態
        self.joint_states_sub = rospy.Subscriber(
            '/joint_states',
            sensor_msgs.msg.JointState,
            self.joint_states_callback
        )
        
        # 訂閱 TF 訊息
        self.tf_sub = rospy.Subscriber(
            '/tf',
            TFMessage,
            self.tf_callback
        )
        
        # 儲存關節狀態
        self.joint_positions = {}
        self.tf_transforms = {}  # 儲存最新的 TF 轉換
        
        # 機器手臂關節名稱
        self.joint_names = [
            'joint_1', 'joint_2', 'joint_3',
            'joint_4', 'joint_5', 'joint_6'
        ]
        
        rospy.loginfo("Robot Arm TF Listener 已啟動")

    def tf_callback(self, msg):
        """處理 TF 訊息"""
        for transform in msg.transforms:
            key = f"{transform.header.frame_id}_{transform.child_frame_id}"
            self.tf_transforms[key] = transform.transform

    def joint_states_callback(self, msg):
        """處理關節狀態訊息"""
        for i, name in enumerate(msg.name):
            if name in self.joint_names and i < len(msg.position):
                self.joint_positions[name] = msg.position[i]

    def get_joint_position(self, joint_name):
        """獲取指定關節相對於基座的位置和姿態"""
        try:
            return self.tf_buffer.lookup_transform(
                'base_link',
                f'link_{joint_name[-1]}',
                rospy.Time(0)
            )
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException,
                tf2_ros.ExtrapolationException) as e:
            rospy.logwarn(f"無法獲取關節 {joint_name} 的轉換: {e}")
            return None

    def quaternion_to_euler(self, quaternion):
        """將四元數轉換為歐拉角（RPY）"""
        x, y, z, w = quaternion.x, quaternion.y, quaternion.z, quaternion.w
        roll = math.atan2(2 * (w*x + y*z), 1 - 2 * (x*x + y*y))
        pitch = math.asin(2 * (w*y - z*x))
        yaw = math.atan2(2 * (w*z + x*y), 1 - 2 * (y*y + z*z))
        return roll, pitch, yaw

    def print_robot_info(self):
        """印出機器手臂資訊"""
        positions = []
        orientations = []
        angles = []
        
        for joint in self.joint_names:
            # 獲取關節位置和姿態
            transform = self.get_joint_position(joint)
            if transform:
                # 位置資料
                pos = transform.transform.translation
                positions.append(f"[{pos.x:.4f}, {pos.y:.4f}, {pos.z:.4f}]")
                
                # 姿態資料（歐拉角）
                roll, pitch, yaw = self.quaternion_to_euler(transform.transform.rotation)
                orientations.append(f"[{roll:.4f}, {pitch:.4f}, {yaw:.4f}]")
            
            # 角度資料
            if joint in self.joint_positions:
                angle = math.degrees(self.joint_positions[joint])
                angles.append(f"{angle:.2f}")
        
        # 印出所有資料（使用不同顏色）
        print(f"{Colors.RED}position = [{', '.join(positions)}]{Colors.ENDC}")
        print(f"{Colors.GREEN}orientation = [{', '.join(orientations)}]{Colors.ENDC}")
        print(f"{Colors.BLUE}angle = [{', '.join(angles)}]{Colors.ENDC}")

    def run(self):
        """主迴圈"""
        rate = rospy.Rate(10)  # 10Hz
        while not rospy.is_shutdown():
            self.print_robot_info()
            rate.sleep()

if __name__ == '__main__':
    try:
        listener = RobotArmTFListener()
        listener.run()
    except rospy.ROSInterruptException:
        pass 
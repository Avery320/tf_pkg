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
    YELLOW = '\033[93m'   # 黃色
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
        
        # 機器手臂完整結構（以 world 為基準）
        self.robot_structure = {
            'world': {
                'safety_floor': {'fixed': True, 'height': 0.0},  # 地面高度為 0
                'agv': {
                    'base_link': {'fixed': True, 'height': 0.325},  # AGV 基座高度
                    'robot_base': {'fixed': True, 'height': 0.0},   # 機械手臂基座（相對於 AGV）
                    'link_1': {'joint': 'joint_1', 'axis': 'Z'},
                    'link_2': {'joint': 'joint_2', 'axis': 'X'},
                    'link_3': {'joint': 'joint_3', 'axis': 'X'},
                    'link_4': {'joint': 'joint_4', 'axis': 'Y'},
                    'link_5': {'joint': 'joint_5', 'axis': 'X'},
                    'link_6': {'joint': 'joint_6', 'axis': 'Y'},
                    'flange': {'fixed': True, 'rotation': -90},
                    'gripper': {'fixed': True},
                    'tool0': {'fixed': True, 'position': [0, 0.090, 0.155], 'rotation': 135}
                }
            }
        }
        
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

    def get_transform(self, from_frame, to_frame):
        """獲取兩個框架之間的轉換"""
        try:
            return self.tf_buffer.lookup_transform(
                from_frame,
                to_frame,
                rospy.Time(0)
            )
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException,
                tf2_ros.ExtrapolationException) as e:
            rospy.logwarn(f"無法獲取從 {from_frame} 到 {to_frame} 的轉換: {e}")
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
        print(f"\n{Colors.YELLOW}=== 機器手臂狀態 ==={Colors.ENDC}")
        
        # 印出世界座標系資訊
        print(f"\n{Colors.BLUE}世界座標系:{Colors.ENDC}")
        print(f"安全地板高度: {self.robot_structure['world']['safety_floor']['height']}m")
        
        # 印出 AGV 和基座資訊
        print(f"\n{Colors.BLUE}AGV 和基座:{Colors.ENDC}")
        print(f"AGV 基座高度: {self.robot_structure['world']['agv']['base_link']['height']}m")
        
        # 印出關節資訊
        print(f"\n{Colors.BLUE}關節狀態:{Colors.ENDC}")
        for joint in self.joint_names:
            if joint in self.joint_positions:
                angle = math.degrees(self.joint_positions[joint])
                print(f"{joint}: {angle:.2f}°")
        
        # 印出末端執行器資訊
        print(f"\n{Colors.BLUE}末端執行器 (tool0):{Colors.ENDC}")
        try:
            # 獲取 tool0 相對於 world 的轉換
            transform = self.tf_buffer.lookup_transform(
                'world',
                'tool0',
                rospy.Time(0)
            )
            
            # 獲取位置
            pos = transform.transform.translation
            print(f"相對於世界座標系的位置: x={pos.x:.3f}, y={pos.y:.3f}, z={pos.z:.3f}")
            
            # 獲取旋轉（歐拉角）
            rot = transform.transform.rotation
            roll, pitch, yaw = self.quaternion_to_euler(rot)
            print(f"相對於世界座標系的旋轉: roll={math.degrees(roll):.2f}°, pitch={math.degrees(pitch):.2f}°, yaw={math.degrees(yaw):.2f}°")
            
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException,
                tf2_ros.ExtrapolationException) as e:
            print(f"{Colors.RED}無法獲取 tool0 的轉換資訊: {e}{Colors.ENDC}")

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
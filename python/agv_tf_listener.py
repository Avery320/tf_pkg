#!/usr/bin/env python3

import roslibpy
import math
import threading
import time

# ANSI 顏色代碼
class Colors:
    RED = '\033[91m'
    GREEN = '\033[92m'
    BLUE = '\033[94m'
    YELLOW = '\033[93m'
    ENDC = '\033[0m'

class RobotArmTFListener:
    def __init__(self, ros_host='localhost', ros_port=9090):
        self.client = roslibpy.Ros(host=ros_host, port=ros_port)
        self.joint_positions = {}
        self.tf_transforms = {}
        self.lock = threading.Lock()

        self.robot_structure = {
            'world': {
                'safety_floor': {'fixed': True, 'height': 0.0},
                'agv': {
                    'base_link': {'fixed': True, 'height': 0.325},
                    'robot_base': {'fixed': True, 'height': 0.0},
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
        self.joint_names = [
            'joint_1', 'joint_2', 'joint_3',
            'joint_4', 'joint_5', 'joint_6'
        ]

        self.client.run()
        print(f"{Colors.GREEN}已連接到 rosbridge！{Colors.ENDC}")

        self.joint_states_sub = roslibpy.Topic(self.client, '/joint_states', 'sensor_msgs/JointState')
        self.joint_states_sub.subscribe(self.joint_states_callback)

        self.tf_sub = roslibpy.Topic(self.client, '/tf', 'tf2_msgs/TFMessage')
        self.tf_sub.subscribe(self.tf_callback)

    def tf_callback(self, message):
        with self.lock:
            for transform in message['transforms']:
                key = f"{transform['header']['frame_id']}_{transform['child_frame_id']}"
                self.tf_transforms[key] = transform['transform']

    def joint_states_callback(self, message):
        with self.lock:
            for i, name in enumerate(message['name']):
                if name in self.joint_names and i < len(message['position']):
                    self.joint_positions[name] = message['position'][i]

    def quaternion_to_euler(self, q):
        x, y, z, w = q['x'], q['y'], q['z'], q['w']
        roll = math.atan2(2 * (w*x + y*z), 1 - 2 * (x*x + y*y))
        pitch = math.asin(2 * (w*y - z*x))
        yaw = math.atan2(2 * (w*z + x*y), 1 - 2 * (y*y + z*z))
        return roll, pitch, yaw

    def print_robot_info(self):
        with self.lock:
            print(f"\n{Colors.YELLOW}=== 機器手臂狀態 ==={Colors.ENDC}")
            print(f"\n{Colors.BLUE}世界座標系:{Colors.ENDC}")
            print(f"安全地板高度: {self.robot_structure['world']['safety_floor']['height']}m")
            print(f"\n{Colors.BLUE}AGV 和基座:{Colors.ENDC}")
            print(f"AGV 基座高度: {self.robot_structure['world']['agv']['base_link']['height']}m")
            print(f"\n{Colors.BLUE}關節狀態:{Colors.ENDC}")
            for joint in self.joint_names:
                if joint in self.joint_positions:
                    angle = math.degrees(self.joint_positions[joint])
                    print(f"{joint}: {angle:.2f}°")
            print(f"\n{Colors.BLUE}末端執行器 (tool0):{Colors.ENDC}")
            # 嘗試找出 world->tool0 的 tf
            key = 'world_tool0'
            if key in self.tf_transforms:
                pos = self.tf_transforms[key]['translation']
                print(f"相對於世界座標系的位置: x={pos['x']:.3f}, y={pos['y']:.3f}, z={pos['z']:.3f}")
                rot = self.tf_transforms[key]['rotation']
                roll, pitch, yaw = self.quaternion_to_euler(rot)
                print(f"相對於世界座標系的旋轉: roll={math.degrees(roll):.2f}°, pitch={math.degrees(pitch):.2f}°, yaw={math.degrees(yaw):.2f}°")
            else:
                print(f"{Colors.RED}無法獲取 tool0 的轉換資訊{Colors.ENDC}")

    def run(self):
        try:
            while self.client.is_connected:
                self.print_robot_info()
                time.sleep(1)
        except KeyboardInterrupt:
            print("中斷，關閉連線")
        finally:
            self.joint_states_sub.unsubscribe()
            self.tf_sub.unsubscribe()
            self.client.terminate()

if __name__ == '__main__':
    # 根據你的 rosbridge 伺服器 IP/port 修改
    listener = RobotArmTFListener(ros_host='localhost', ros_port=9090)
    listener.run() 
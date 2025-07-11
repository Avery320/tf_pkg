#!/usr/bin/env python3

import rospy
import tf2_ros
import math
from tf_pkg.msg import Tool0Pose

def quaternion_to_euler(quaternion):
    x, y, z, w = quaternion.x, quaternion.y, quaternion.z, quaternion.w
    roll = math.atan2(2 * (w*x + y*z), 1 - 2 * (x*x + y*y))
    pitch = math.asin(2 * (w*y - z*x))
    yaw = math.atan2(2 * (w*z + x*y), 1 - 2 * (y*y + z*z))
    return roll, pitch, yaw

def main():
    rospy.init_node('sub_tool0_pose', anonymous=True)
    tf_buffer = tf2_ros.Buffer()
    tf_listener = tf2_ros.TransformListener(tf_buffer)
    pub = rospy.Publisher('/tool0_pose', Tool0Pose, queue_size=10)
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        try:
            transform = tf_buffer.lookup_transform('base_link', 'tool0', rospy.Time(0))
            pos = transform.transform.translation
            rot = transform.transform.rotation
            roll, pitch, yaw = quaternion_to_euler(rot)
            msg = Tool0Pose()
            msg.x = pos.x
            msg.y = pos.y
            msg.z = pos.z
            msg.roll = roll
            msg.pitch = pitch
            msg.yaw = yaw
            pub.publish(msg)
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
            rospy.logwarn(f"無法獲取 base_link 到 tool0 的轉換: {e}")
        rate.sleep()

if __name__ == '__main__':
    main() 
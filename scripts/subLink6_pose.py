#!/usr/bin/env python3

import rospy
import tf2_ros
import math
from tf_pkg.msg import Link6Pose

def quaternion_to_euler(quaternion):
    x, y, z, w = quaternion.x, quaternion.y, quaternion.z, quaternion.w
    roll = math.atan2(2 * (w*x + y*z), 1 - 2 * (x*x + y*y))
    pitch = math.asin(2 * (w*y - z*x))
    yaw = math.atan2(2 * (w*z + x*y), 1 - 2 * (y*y + z*z))
    return roll, pitch, yaw

def main():
    rospy.init_node('sub_link6_pose', anonymous=True)
    tf_buffer = tf2_ros.Buffer()
    tf_listener = tf2_ros.TransformListener(tf_buffer)
    pub = rospy.Publisher('/link6_pose', Link6Pose, queue_size=10)
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        try:
            transform = tf_buffer.lookup_transform('world', 'link_6', rospy.Time(0))
            pos = transform.transform.translation
            rot = transform.transform.rotation
            roll, pitch, yaw = quaternion_to_euler(rot)
            msg = Link6Pose()
            msg.x = pos.x
            msg.y = pos.y
            msg.z = pos.z
            msg.roll = roll
            msg.pitch = pitch
            msg.yaw = yaw
            pub.publish(msg)
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
            rospy.logwarn(f"無法獲取 base_link 到 link_6 的轉換: {e}")
        rate.sleep()

if __name__ == '__main__':
    main() 
#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import JointState
from std_msgs.msg import Header
import time

def talker():
    pub = rospy.Publisher('joint_states', JointState, queue_size=10)
    rospy.init_node('joint_state_publisher')
    rate = rospy.Rate(10) # 10hz
    control_str = JointState()
    control_str.header = Header()
    control_str.name = ['base_to_slider', 'slide', 'rotate']
    control_str.velocity = []
    control_str.effort = []
    control_str.position = [0, 0, 0]
    control_str.header.stamp = rospy.Time.now()
    time.sleep(1.5)
    pub.publish(control_str)
    rate.sleep()
    while not rospy.is_shutdown():
      control_str.position = [float(i) for i in input("Joints positions: lateral slider, vertical slider, hummer rotation: ").split()]
      control_str.header.stamp = rospy.Time.now()
      pub.publish(control_str)
      rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass

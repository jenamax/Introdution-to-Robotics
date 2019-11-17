#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import JointState
from std_msgs.msg import Header
import time
import numpy as np
from numpy import sin, cos, sqrt, pi, transpose
from numpy.linalg import inv
from numpy import arctan2 as atan2

def rot2euler(matr):
    if matr[0, 0] != 1 and matr[0, 0] != -1:
        psi = atan2(matr[1, 0], -matr[2, 0])
        phi = atan2(matr[0, 1], matr[0, 2])
        theta = atan2(sqrt(matr[2, 0]**2 + matr[1, 0]**2), matr[0, 0])
    elif matr[0, 0] == 1:
        theta = 0
        phi = 0
        psi = atan2(-matr[1, 2], matr[1, 1])
    else:
        theta = pi
        phi = 0
        psi = atan2(matr[1, 2], matr[1, 1])
    return psi, theta, phi

def rot_matr(axis, angle):
    if axis == 'x':
        matr = np.matrix([[1, 0, 0, 0], [0, cos(angle), -sin(angle), 0], [0, sin(angle), cos(angle), 0], [0, 0, 0, 1]])
    elif axis == 'y':
        matr = np.matrix([[cos(angle), 0, sin(angle), 0], [0, 1, 0, 0], [-sin(angle), 0, cos(angle), 0], [0, 0, 0, 1]])
    else:
        matr = np.matrix([[cos(angle), -sin(angle), 0, 0], [sin(angle), cos(angle), 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]])
    return matr

def inverse(T):
    pass

def direct(q):
    r1 = rot_matr('z', q[0])
    r2 = rot_matr('x', q[1])
    r3 = rot_matr('x', q[2])
    r4 = rot_matr('y', q[3])
    r5 = rot_matr('x', q[4])
    r6 = rot_matr('y', q[5])

    T = r1 * t1 * t2 * r2 * t3 * r3 * t4 * r4 * t5 * r5 * r6 * t6
    return T

def talker():
    pub = rospy.Publisher('joint_states', JointState, queue_size=10)
    rospy.init_node('joint_state_publisher')
    rate = rospy.Rate(10) # 10hz
    control_str = JointState()
    control_str.header = Header()
    control_str.name = ['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6']
    control_str.velocity = []
    control_str.effort = []
    control_str.position = [0, pi / 2, 0, 0, 0, 0]
    control_str.header.stamp = rospy.Time.now()
    time.sleep(1.5)
    pub.publish(control_str)

    rate.sleep()
    x0, y0, z0 = [float(i) for i in input("Base position: ").split()]
    while not rospy.is_shutdown():
      control_str.position = [float(i) for i in input("Set angle for each joint: ").split()]
      control_str.header.stamp = rospy.Time.now()
      pub.publish(control_str)
      T = direct([0.5, 1.2, 0.5, 1, 0.1, 1.5])
      r = (T * transpose(np.matrix([x0, y0, z0, 1])))[0:3]
      print("end effector position: " + str(r))
      rate.sleep()

if __name__ == '__main__':
    L = [670, 312, 1075, 225, 1280, 215]

    idle = np.matrix([[1, 0, 0], [0, 1, 0], [0, 0, 1], [0, 0, 0]])
    t1 = np.concatenate([idle, transpose(np.matrix([0, 0, L[0], 1]))], axis=1)
    t2 = np.concatenate([idle, transpose(np.matrix([0, L[1], 0, 1]))], axis=1)
    t3 = np.concatenate([idle, transpose(np.matrix([0, 0, L[2], 1]))], axis=1)
    t4 = np.concatenate([idle, transpose(np.matrix([0, 0, L[3], 1]))], axis=1)
    t5 = np.concatenate([idle, transpose(np.matrix([0, L[4], 0, 1]))], axis=1)
    t6 = np.concatenate([idle, transpose(np.matrix([0, L[5], 0, 1]))], axis=1)

    try:
        talker()
    except rospy.ROSInterruptException:
        pass

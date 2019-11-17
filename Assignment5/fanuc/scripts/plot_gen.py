#!/usr/bin/env python3
import numpy as np
from numpy import abs
import rospy
from std_msgs.msg import Float32
from sensor_msgs.msg import JointState
import matplotlib.pyplot as plt
from time import time
from funcs import *

global pos_j1, pos_j2, pos_j3, pos_j4, pos_j5, pos_j6
global vel_j1, vel_j2, vel_j3, vel_j4, vel_j5, vel_j6
global pos_x, pos_y, pos_z
global vel_x, vel_y, vel_z
global writing, plot_num, t

pos_j1 = []
vel_j1 = []

pos_j2 = []
vel_j2 = []

pos_j3 = []
vel_j3 = []

pos_j4 = []
vel_j4 = []

pos_j5 = []
vel_j5 = []

pos_j6 = []
vel_j6 = []

pos_x = []
pos_y = []
pos_z = []

vel_x = []
vel_y = []
vel_z = []

t = []

plot_num = 1
writing = False

global x0, y0, z0, Tbase

x0, y0, z0 = [1, 1, 1]
Tbase = hom_trans(np.eye(3), np.array([x0, y0, z0]))

def callback(data):
    global pos_j1, pos_j2, pos_j3, pos_j4, pos_j5, pos_j6
    global vel_j1, vel_j2, vel_j3, vel_j4, vel_j5, vel_j6
    global pos_x, pos_y, pos_z
    global vel_x, vel_y, vel_z
    global writing, plot_num, t
    global x0, y0, z0, Tbase

    if abs(data.velocity).any() > 0.001:
        writing = True
        t.append(time())

        pos_j1.append(data.position[0])
        pos_j2.append(data.position[1])
        pos_j3.append(data.position[2])
        pos_j4.append(data.position[3])
        pos_j5.append(data.position[4])
        pos_j6.append(data.position[5])

        r = (Tbase * direct_fanuc(data.position))[0:3, 3]
        r = np.asarray(np.transpose(r))[0]

        pos_x.append(r[0])
        pos_y.append(r[1])
        pos_z.append(r[2])

        vel_j1.append(data.velocity[0])
        vel_j2.append(data.velocity[1])
        vel_j3.append(data.velocity[2])
        vel_j4.append(data.velocity[3])
        vel_j5.append(data.velocity[4])
        vel_j6.append(data.velocity[5])

    elif writing and plot_num == 4:
        plt.xlabel('time')
        plt.ylabel('postion')
        plt.plot(t, pos_j1, label="Joint_1")
        plt.plot(t, pos_j2, label="Joint_2")
        plt.plot(t, pos_j3, label="Joint_3")
        plt.plot(t, pos_j4, label="Joint_4")
        plt.plot(t, pos_j5, label="Joint_5")
        plt.plot(t, pos_j6, label="Joint_6")
        plt.legend()

        name = "/home/evgenii/jp.png"
        plt.savefig(name)
        plt.clf()

        plt.xlabel('time')
        plt.ylabel('velocity')
        plt.plot(t, vel_j1, label="Joint_1")
        plt.plot(t, vel_j2, label="Joint_2")
        plt.plot(t, vel_j3, label="Joint_3")
        plt.plot(t, vel_j4, label="Joint_4")
        plt.plot(t, vel_j5, label="Joint_5")
        plt.plot(t, vel_j6, label="Joint_6")
        plt.legend()

        name = "/home/evgenii/jv.png"
        plt.savefig(name)
        plt.clf()


        for i in range(0, len(vel_j1)):
            J = jac_fanuc(data.position, [x0, y0, z0])
            vel_j = np.array([vel_j1[i], vel_j2[i], vel_j3[i], vel_j4[i], vel_j5[i], vel_j6[i]])
            twist = np.asarray(np.transpose(J * np.transpose((np.matrix(vel_j)))))[0]
            vel_x.append(twist[0])
            vel_y.append(twist[1])
            vel_z.append(twist[2])

        plt.xlabel('time')
        plt.ylabel('velocity')
        plt.plot(t, vel_x, label="X")
        plt.plot(t, vel_y, label="Y")
        plt.plot(t, vel_z, label="Z")
        plt.legend()

        name = "/home/evgenii/xyz_v.png"
        plt.savefig(name)
        plt.clf()

        plt.xlabel('time')
        plt.ylabel('postion')
        plt.plot(t, pos_x, label="X")
        plt.plot(t, pos_y, label="Y")
        plt.plot(t, pos_z, label="Z")
        plt.legend()

        name = "/home/evgenii/xyz_p.png"
        plt.savefig(name)
        plt.clf()

        print("Plot built")

        pos_j1 = []
        vel_j1 = []

        pos_j2 = []
        vel_j2 = []

        pos_j3 = []
        vel_j3 = []

        pos_j4 = []
        vel_j4 = []

        pos_j5 = []
        vel_j5 = []

        pos_j6 = []
        vel_j6 = []

        pos_x = []
        pos_y = []
        pos_z = []

        vel_x = []
        vel_y = []
        vel_z = []

        t = []
        writing = False

        plot_num = 1
    elif writing:
        plot_num += 1
        writing = False



def listener():
    rospy.init_node('plot_gen', anonymous=True)
    rospy.Subscriber("joint_states", JointState, callback)
    rospy.spin()

if __name__ == '__main__':
    listener()

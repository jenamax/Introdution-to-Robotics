#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import JointState
from std_msgs.msg import Header, Float32
import time
import numpy as np
from numpy import sin, cos, sqrt, pi, transpose, array, arctan2, arctan, arccos
from numpy.linalg import inv, det
from funcs import *
from time import time, sleep
from math import ceil
import threading

class sates_pub_thread (threading.Thread): # for publishing joint states constantly
    def __init__(self):
       threading.Thread.__init__(self)
       self.breaking = False

    def run(self):
        global joints_control, pub, pos_pub
        while not rospy.is_shutdown() and not self.breaking:
            for i in range(0, 6):
                pos_pub[i].publish(joints_control.position[i])
            for i in range(0, 6):
                vel_pub[i].publish(joints_control.velocity[i])
            pub.publish(joints_control)

    def break_pub(self):
        self.breaking = True

def reset(): # return all joints to initial positions
    global joints_control
    joints_control.velocity = np.zeros(6, dtype=np.float)
    joints_control.effort = np.zeros(6, dtype=np.float)
    joints_control.position = np.zeros(6, dtype=np.float)

global a_joint_max, V_joint_max, a_cart_max, V_cart_max, t_discr, junc

global x0, y0, z0
global joints_control, thread1, pub, pos_pub

a_joint_max = np.ones(6) * 0.1
V_joint_max = np.ones(6)

a_cart_max = 1000 * np.ones(3)
V_cart_max = 1000 * np.ones(3)

t_discr = 0.01

junc = 5 * t_discr

def cur_vel(time, traj_params, command): # compute velocity for current moment of time with given profile parameters
    # get position by time
    if command == "PTP":
        QRet = np.zeros(6, dtype=np.float)
    elif command == "LIN":
        QRet = np.zeros(3, dtype=np.float)
    [t, T, Tf, Vmax, amax, trapezia] = traj_params
    for i in range(len(QRet)):
        if trapezia[i]:
            if time <= t:
                QRet[i] = amax[i] * time
            elif time > t and time <= T:
                QRet[i] = Vmax[i]
            elif time > T and time <= Tf:
                QRet[i] = Vmax[i] - amax[i] * (time - T)
            else:
                QRet[i] = 0
        else:
            if time <= t:
                QRet[i] = amax[i] * time
            elif time > t and time <= Tf:
                QRet[i] = Vmax[i] - amax[i] * (time - T)
            else:
                QRet[i] = 0
    return np.asarray(QRet)

def traj_params(Q, command): # compute trajectory parameters
    global a_joint_max, V_joint_max, a_cart_max, V_cart_max, t_discr, junc
    if command == 'LIN':
        t = np.zeros(3, dtype=np.float)
        T = np.zeros(3, dtype=np.float)
        Tf = np.zeros(3, dtype=np.float)
        V_max = np.copy(V_cart_max)
        a_max = np.copy(a_cart_max)
    elif command == 'PTP':
        t = np.zeros(6, dtype=np.float)
        T = np.zeros(6, dtype=np.float)
        Tf = np.zeros(6, dtype=np.float)
        V_max = np.copy(V_joint_max)
        a_max = np.copy(a_joint_max)

    for i in range(len(t)): # compute for each axis separately
        if np.sqrt(a_max[i] * abs(Q[i])) > V_max[i]:
            t[i] = V_max[i] / a_max[i]
            T[i] = abs(Q[i]) / V_max[i]
            Tf[i] = t[i] * 2 + T[i]
        else:
            V_max[i] = np.sqrt(a_max[i] * abs(Q[i]))
            t[i] = np.sqrt(abs(Q[i]) / a_max[i])
            T[i] = t[i]
            Tf[i] = 2 * t[i]

    tmax = ceil(max(t) / t_discr) * t_discr
    Tmax = ceil(max(T) / t_discr) * t_discr
    trapezia = []
    for i in range(len(t)): # recompute in order to stop movement simultaneosly on all axes
        V_max[i] = abs(Q[i]) / Tmax
        a_max[i] = V_max[i] / tmax
        if np.sqrt(a_max[i] * abs(Q[i])) > V_max[i]:
            t[i] = V_max[i] / a_max[i]
            T[i] = abs(Q[i]) / V_max[i]
            Tf[i] = T[i] + t[i]
            trapezia.append(True)
        else:
            V_max[i] = np.sqrt(a_max[i] * abs(Q[i]))
            t[i] = np.sqrt(abs(Q[i]) / a_max[i])
            T[i] = t[i]
            Tf[i] = 2 * t[i]
            trapezia.append(False)

    print('Speed up time:', t)
    print('Time on max speed:', T)
    print('Finish time:', Tf)
    print('Max V:', V_max)
    print('Max acc:', a_max)
    print('Trapezioid profile:', trapezia)

    return [t[0], T[0], Tf[0], V_max, a_max, trapezia]

def talker():
    global x0, y0, z0, joints_control, pub
    # initialize controller for joints
    rospy.init_node('trajectory_planning')
    rate = rospy.Rate(100) # 10hz
    joints_control = JointState()
    joints_control.header = Header()
    joints_control.name = ['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6']
    joints_control.velocity = np.zeros(6, dtype=np.float)
    joints_control.effort = np.zeros(6, dtype=np.float)
    joints_control.position = np.zeros(6, dtype=np.float)
    joints_control.header.stamp = rospy.Time.now()

    thread1 = sates_pub_thread()
    thread1.start()

    rate.sleep()
    x0, y0, z0 = [1, 1, 1]
    Tbase = hom_trans(np.eye(3), np.array([x0, y0, z0]))
    r0 = (Tbase * direct_fanuc(joints_control.position))[0:3, 3]
    r0 = np.asarray(transpose(r0))[0]

    print("Input something to start")
    input()

    print("Start position (Joint): ", joints_control.position)
    print("Start position (Cartesian): ", r0)
    while not rospy.is_shutdown():
        # set maximal velocities and accelerations
        a_joint_max = np.ones(6)
        V_joint_max = np.ones(6)
        a_cart_max = 1000 * np.ones(3)
        V_cart_max = 1000 * np.ones(3)

        joints_control.header.stamp = rospy.Time.now()

        # set desired postion for each of four moves
        q_des1 = np.array([2, 0.5, 1, 0.8, 1.5, 2.5])
        q_des2 = np.array([410, 400, 2950])
        q_des3 = np.array([2.5, 0.8, 1.2, 0.5, 1, 0.7])
        q_des4 = np.array([900, 800, 2410])
        q_des = [q_des1, q_des2, q_des3, q_des4]
        commands = ["PTP", "LIN", "PTP", "LIN"]

        for i in range(0, 4):
            q = joints_control.position
            r0 = ((Tbase * direct_fanuc(joints_control.position)))[0:3, 3]
            r0 = transpose(r0)
            r = r0

            if commands[i] == "PTP":
                delta_q = (np.asarray(q_des[i] - q))
            elif commands[i] == "LIN":
                delta_q = (np.asarray(q_des[i] - r0))[0]
            else:
                print("Command " + commands[i] + " invalid")

            print("Delta q:", delta_q)
            start_time = time()
            cur_time = time()
            prev_time = time()
            vel_profile = traj_params(delta_q, commands[i])
            dir = abs(delta_q) / delta_q
            print("start move")
            while prev_time - start_time < vel_profile[2]:
                freq = 0.
                steps = 0

                cur_time = time()
                dt = cur_time - prev_time
                freq += dt
                steps += 1
                #print("{0:.4f}".format(dt)) # check control frequency
                vel = cur_vel(cur_time - start_time, vel_profile, commands[i])
                if len(q_des[i]) == 6:
                    q = joints_control.position + vel * dt * dir
                elif len(q_des[i]) == 3:
                    vel_cart = vel * dir
                    vel_joint = np.asarray(transpose(inv(jac_fanuc(joints_control.position, [x0, y0, z0])) \
                                                * transpose(np.matrix(np.concatenate([vel_cart, np.zeros(3)])))))[0]
                    q = joints_control.position + vel_joint * dt
                    r += vel_cart * dt
                joints_control.position = q
                if len(q_des[i]) == 6:
                    joints_control.velocity = vel * dir
                elif len(q_des[i]) == 3:
                    joints_control.velocity = vel_joint
                while time() - cur_time < t_discr: # for making control actions with exactly (almost) 100Hz frequence
                    sleep(0.00001)
                prev_time = cur_time
            joints_control.velocity = np.zeros(6)
            print("finish move in time:", time() - start_time)
            print("Desired postion for current move: ", q_des[i])
            print("Final postion for current move (Joints)", joints_control.position)
            print("Final postion for current move (Cartesian)", np.asarray(transpose((Tbase * direct_fanuc(joints_control.position))[0:3, 3])))
            print("Average control frequency: " + str(steps / freq) + "Hz")
            print()
        break
    exit(0)

if __name__ == '__main__':
    global L, pub, pos_pub
    pub = rospy.Publisher('joint_states', JointState, queue_size=10)
    pos_pub = []
    for i in range(0, 6):
        name = 'joint_pos_' + str(i)
        pos_pub.append(rospy.Publisher(name, Float32, queue_size=10))
    vel_pub = []
    for i in range(0, 6):
        name = 'joint_vel_' + str(i)
        vel_pub.append(rospy.Publisher(name, Float32, queue_size=10))
    L = [670, 312, 1075, 225, 1280, 215]
    try:
        talker()
    except rospy.ROSInterruptException:
        thread1.join()
        exit(0)
        pass

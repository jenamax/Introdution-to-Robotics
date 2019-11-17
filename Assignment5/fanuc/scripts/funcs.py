import numpy as np
from numpy import sin, cos

np.random.seed(0)

def rot_matr(axis, angle):
    if axis == 'x':
        matr = np.matrix([[1, 0, 0], [0, cos(angle), -sin(angle)], [0, sin(angle), cos(angle)]])
    elif axis == 'y':
        matr = np.matrix([[cos(angle), 0, sin(angle)], [0, 1, 0], [-sin(angle), 0, cos(angle)]])
    else:
        matr = np.matrix([[cos(angle), -sin(angle), 0], [sin(angle), cos(angle), 0], [0, 0, 1]])
    return matr

def hom_trans(rot, trans):
    T = np.matrix(np.eye(4), dtype=float)
    T[0:3, 0:3] = rot
    T[0:3, 3] = trans.reshape(3, 1)
    return T

def Rx(angle):
    return hom_trans(rot_matr('x', angle), np.zeros(3))

def Ry(angle):
    return hom_trans(rot_matr('y', angle), np.zeros(3))

def Rz(angle):
    return hom_trans(rot_matr('z', angle), np.zeros(3))

def Tx(p):
    return hom_trans(np.eye(3), np.array([p, 0, 0]))

def Ty(p):
    return hom_trans(np.eye(3), np.array([0, p, 0]))

def Tz(p):
    return hom_trans(np.eye(3), np.array([0, 0, p]))

def dTx():
    T = np.matrix(np.zeros((4, 4)), dtype=float)
    T[0, 3] = 1
    return T

def dTy():
    T = np.matrix(np.zeros((4, 4)), dtype=float)
    T[1, 3] = 1
    return T

def dTz():
    T = np.matrix(np.zeros((4, 4)), dtype=float)
    T[2, 3] = 1
    return T

def dRx():
    T = np.matrix(np.zeros((4, 4)))
    T[2, 1] = 1
    T[1, 2] = -1
    return T

def dRy():
    T = np.matrix(np.zeros((4, 4)))
    T[0, 2] = 1
    T[2, 0] = -1
    return T

def dRz():
    T = np.matrix(np.zeros((4, 4)))
    T[1, 0] = 1
    T[0, 1] = -1
    return T

def jac_fanuc(q, base): # compute jacobians numerically
    x0, y0, z0 = base[0], base[1], base[2]
    L = [670, 312, 1075, 225, 1280, 215]
    Tb = hom_trans(np.matrix(np.eye(3)), np.transpose(np.matrix([x0, y0, z0])))

    H1 = np.matrix(np.zeros((4, 4)))
    H1[1, 0] = 1
    H1[0, 1] = -1

    H2 = np.matrix(np.zeros((4, 4)))
    H2[2, 1] = 1
    H2[1, 2] = -1

    H2 = np.matrix(np.zeros((4, 4)))
    H2[2, 1] = 1
    H2[1, 2] = -1

    H3 = np.matrix(np.zeros((4, 4)))
    H3[2, 1] = 1
    H3[1, 2] = -1

    H4 = np.matrix(np.zeros((4, 4)))
    H4[0, 2] = 1
    H4[2, 0] = -1

    H5 = np.matrix(np.zeros((4, 4)))
    H5[2, 1] = 1
    H5[1, 2] = -1

    H6 = np.matrix(np.zeros((4, 4)))
    H6[0, 2] = 1
    H6[2, 0] = -1

    T1 = Tb * Rz(q[0]) * H1 * Tz(L[0]) * Ty(L[1]) * Rx(q[1]) * Tz(L[2]) * Rx(q[2]) * Tz(L[3]) * Ry(q[3]) * Ty(L[4]) * Rx(q[4]) * Ry(q[5]) * Ty(L[5])
    T2 = Tb * Rz(q[0]) * Tz(L[0]) * Ty(L[1]) * Rx(q[1]) * H2 * Tz(L[2]) * Rx(q[2]) * Tz(L[3]) * Ry(q[3]) * Ty(L[4]) * Rx(q[4]) * Ry(q[5]) * Ty(L[5])
    T3 = Tb * Rz(q[0]) * Tz(L[0]) * Ty(L[1]) * Rx(q[1]) * Tz(L[2]) * Rx(q[2]) * H3 * Tz(L[3]) * Ry(q[3]) * Ty(L[4]) * Rx(q[4]) * Ry(q[5]) * Ty(L[5])
    T4 = Tb * Rz(q[0]) * Tz(L[0]) * Ty(L[1]) * Rx(q[1]) * Tz(L[2]) * Rx(q[2]) * Tz(L[3]) * Ty(L[4]) * Ry(q[3]) * H4 * Rx(q[4]) * Ry(q[5]) * Ty(L[5])
    T5 = Tb * Rz(q[0]) * Tz(L[0]) * Ty(L[1]) * Rx(q[1]) * Tz(L[2]) * Rx(q[2]) * Tz(L[3]) * Ry(q[3]) * Ty(L[4]) * Rx(q[4]) * H5 * Ry(q[5]) * Ty(L[5])
    T6 = Tb * Rz(q[0]) * Tz(L[0]) * Ty(L[1]) * Rx(q[1]) * Tz(L[2]) * Rx(q[2]) * Tz(L[3]) * Ry(q[3]) * Ty(L[4]) * Rx(q[4]) * Ry(q[5]) * H6 * Ty(L[5])

    T = [T1, T2, T3, T4, T5, T6]
    J = np.matrix(np.zeros((6, 6)))
    DK = direct_fanuc(q)
    if np.linalg.det(DK) == 0:
        print("Singularity error")
    D_inv = np.linalg.inv(np.matrix(DK))
    for i in range(0, 6):
        J[0, i] = T[i][0, 3]
        J[1, i] = T[i][1, 3]
        J[2, i] = T[i][2, 3]
        J[3, i] = (T[i] * D_inv)[2, 1]
        J[4, i] = (T[i] * D_inv)[0, 2]
        J[5, i]= (T[i] * D_inv)[1, 0]
    return J

def direct_fanuc(q): # compute direct kinematics for robot
    L = [670, 312, 1075, 225, 1280, 215]
    T = Rz(q[0]) * Tz(L[0]) * Ty(L[1]) * Rx(q[1]) * Tz(L[2]) * Rx(q[2]) * Tz(L[3]) \
     * Ry(q[3]) * Ty(L[4]) * Rx(q[4]) * Ry(q[5]) * Ty(L[5])
    return T

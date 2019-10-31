import numpy as np
from numpy import sin, cos

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

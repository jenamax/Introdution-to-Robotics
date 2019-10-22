import numpy as np
from numpy import sqrt, pi, transpose
from numpy.linalg import inv
from numpy import arctan2 as atan2
from sympy import Symbol, sin, cos, lambdify, Matrix

def rot_matr(axis, angle):
    if axis == 'x':
        matr = np.matrix([[1, 0, 0, 0], [0, cos(angle), -sin(angle), 0], [0, sin(angle), cos(angle), 0], [0, 0, 0, 1]])
    elif axis == 'y':
        matr = np.matrix([[cos(angle), 0, sin(angle), 0], [0, 1, 0, 0], [-sin(angle), 0, cos(angle), 0], [0, 0, 0, 1]])
    else:
        matr = np.matrix([[cos(angle), -sin(angle), 0, 0], [sin(angle), cos(angle), 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]])
    return matr

def direct():
    T = r1 * t1 * t2 * r2 * t3 * r3 * t4 * r4 * t5 * r5 * r6 * t6
    return T

def jac_num():
    Tb = np.concatenate([idle, transpose(np.matrix([x0, y0, z0, 1]))], axis=1)

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

    T1 = Tb * H1 * t1 * t2 * r2 * t3 * r3 * t4 * r4 * t5 * r5 * r6 * t6
    T2 = Tb * r1 * t1 * t2 * H2 * t3 * r3 * t4 * r4 * t5 * r5 * r6 * t6
    T3 = Tb * r1 * t1 * t2 * r2 * t3 * H3 * t4 * r4 * t5 * r5 * r6 * t6
    T4 = Tb * r1 * t1 * t2 * r2 * t3 * r3 * t4 * H4 * t5 * r5 * r6 * t6
    T5 = Tb * r1 * t1 * t2 * r2 * t3 * r3 * t4 * r4 * t5 * H5 * r6 * t6
    T6 = Tb * r1 * t1 * t2 * r2 * t3 * r3 * t4 * r4 * t5 * r5 * H6 * t6

    T = [T1, T2, T3, T4, T5, T6]
    J = Matrix(np.zeros((6, 6)))
    D_inv = Matrix(direct())**(-1)
    for i in range(0, 6):
        J[0, i] = T[i][0, 3]
        J[1, i] = T[i][1, 3]
        J[2, i] = T[i][2, 3]
        J[3, i] = (T[i] * D_inv)[2, 1]
        J[4, i] = (T[i] * D_inv)[0, 2]
        J[5, i]= (T[i] * D_inv)[1, 0]
    return J

def jac_skew():
    T0 = np.matrix(np.eye(4))
    T1 = t1 * r1
    T2 = t1 * r1 * t2 * r2
    T3 = t1 * r1 * t2 * r2 * t3 * r3
    T4 = t1 * r1 * t2 * r2 * t3 * r3 * t4 * r4
    T5 = t1 * r1 * t2 * r2 * t3 * r3 * t4 * r4 * t5 * r5
    T6 = t1 * r1 * t2 * r2 * t3 * r3 * t4 * r4 * t5 * r5 * t6 * r6

    Z0 = T0[0:3, 2]
    Z1 = T1[0:3, 0]
    Z2 = T2[0:3, 0]
    Z3 = T3[0:3, 1]
    Z4 = T4[0:3, 0]
    Z5 = T5[0:3, 1]

    O0 = T0[0:3, 3]
    O1 = T1[0:3, 3]
    O2 = T2[0:3, 3]
    O3 = T3[0:3, 3]
    O4 = T4[0:3, 3]
    O5 = T5[0:3, 3]
    O6 = T6[0:3, 3]

    J1 = np.concatenate([np.cross(Z0, (O6 - O0), axis=0), Z0])
    J2 = np.concatenate([np.cross(Z1, (O6 - O1), axis=0), Z1])
    J3 = np.concatenate([np.cross(Z2, (O6 - O2), axis=0), Z2])
    J4 = np.concatenate([np.cross(Z3, (O6 - O3), axis=0), Z3])
    J5 = np.concatenate([np.cross(Z4, (O6 - O4), axis=0), Z4])
    J6 = np.concatenate([np.cross(Z5, (O6 - O5), axis=0), Z5])

    return np.concatenate([J1, J2, J3, J4, J5, J6], axis=1)

if __name__ == '__main__':
    q1 = Symbol('q1')
    q2 = Symbol('q2')
    q3 = Symbol('q3')
    q4 = Symbol('q4')
    q5 = Symbol('q5')
    q6 = Symbol('q6')

    L = [670, 312, 1075, 225, 1280, 215]

    idle = np.matrix([[1, 0, 0], [0, 1, 0], [0, 0, 1], [0, 0, 0]])
    t1 = np.concatenate([idle, transpose(np.matrix([0, 0, L[0], 1]))], axis=1)
    t2 = np.concatenate([idle, transpose(np.matrix([0, L[1], 0, 1]))], axis=1)
    t3 = np.concatenate([idle, transpose(np.matrix([0, 0, L[2], 1]))], axis=1)
    t4 = np.concatenate([idle, transpose(np.matrix([0, 0, L[3], 1]))], axis=1)
    t5 = np.concatenate([idle, transpose(np.matrix([0, L[4], 0, 1]))], axis=1)
    t6 = np.concatenate([idle, transpose(np.matrix([0, L[5], 0, 1]))], axis=1)

    r1 = rot_matr('z', q1)
    r2 = rot_matr('x', q2)
    r3 = rot_matr('x', q3)
    r4 = rot_matr('y', q4)
    r5 = rot_matr('x', q5)
    r6 = rot_matr('y', q6)

    T = direct()
    x0, y0, z0 = 0, 0, 1
    r = (T * transpose(np.matrix([x0, y0, z0, 1])))[0:3]

    j_n = jac_num()

    q = [0.5, 0.8, 1.3, 1.5, 0.2, 0.9]
    result_n = np.zeros((6, 6))
    for i in range(0, j_n.shape[0]):
        for j in range(0, j_n.shape[1]):
            if str(type(j_n[i, j])) != "<class 'int'>":
                f = lambdify((q1, q2, q3, q4, q5, q6), j_n[i, j])
                result_n[i][j] = f(q[0], q[1], q[2], q[3], q[4], q[5])
    print("Numeric:")
    print(result_n)

    j_s = jac_skew()
    result_s = np.zeros((6, 6))
    for i in range(0, j_s.shape[0]):
        for j in range(0, j_s.shape[1]):
            if str(type(j_s[i, j])) != "<class 'int'>":
                f = lambdify((q1, q2, q3, q4, q5, q6), j_s[i, j])
                result_s[i][j] = f(q[0], q[1], q[2], q[3], q[4], q[5])
    print("Skew")
    print(result_s)

    print()
    print(result_n - result_s)

import numpy as np
from numpy import sqrt, pi, transpose
from numpy.linalg import inv, matrix_rank, det
from numpy import arctan2 as atan2
from sympy import Symbol, sin, cos, lambdify, Matrix, simplify
from numpy import random
from funcs import *

def length_err():
    return random.normal(0, 2.5)

def ang_err():
    return random.normal(0, 3. / 180 * pi)

def base_err():
    return random.normal(0, 2)

def T_base_true():
    return Tx(x0 + base_x_err) * Ty(y0 + base_y_err) * Tz(L[0] + z0 + base_z_err)

def T_tool_true():
    return Tx(Lt + t1x_err) * Ty(t1y_err) * Tz(t1z_err), Tx(t1x_err) * Ty(t1y_err) * Tz(Lt + t1z_err), Tx(Lt / (2)**(1 / 2) + t1x_err) * Ty(t1y_err) * Tz(Lt / (2)**(1 / 2) + t1z_err)

def T_robot_true():
    return Rz(q1 + q1_err) * Tx(txL1_err) * Ty(L[1] + tyL1_err) * Ry(ryL1_err) * Rx(q2 + q2_err) * Tz(L[2] + tzL2_err) * Ry(ryL2_err) * Rz(rzL2_err) * Rx(q3 + q3_err) * Ty(L[3] + tyL3_err + L[5]) * Tz(L[4] + tzL3_err) * Rz(rzL3_err) * Ry(q4 + q4_err) * Tx(txL4_err) * Tz(tzL4_err) * Rz(rzL4_err) * Rx(q5 + q5_err) * Tz(tzL5_err) * Rz(rzL5_err) * Ry(q6 + q6_err) * Ty(L[6])

def measure_true():
    Tbr = T_base_true() * T_robot_true()
    Ttool = T_tool_true()
    return np.array([Tbr * Ttool[0], Tbr * Ttool[1], Tbr * Ttool[2]])

def T_base_est():
    return Tx(x0 + est_params[0]) * Ty(y0 + est_params[1]) * Tz(L[0] + z0 + est_params[2])

def T_tool_est():
    return Tx(Lt + est_params[23]) * Ty(est_params[24]) * Tz(est_params[25]), Tx(est_params[26]) * Ty(est_params[27]) * Tz(Lt + est_params[28]), Tx(Lt / (2)**(1 / 2) + est_params[29]) * Ty(est_params[30]) * Tz(Lt / (2)**(1 / 2) + est_params[31])

def T_robot_est():
    est_params[3] = 0
    return Rz(q1 + est_params[3]) * Tx(est_params[4]) * Ty(L[1] + est_params[5]) * Ry(est_params[6]) * Rx(q2 + est_params[7]) * Tz(L[2] + est_params[8]) * Ry(est_params[9]) * Rz(est_params[10]) * Rx(q3 + est_params[11]) * Ty(L[3] + est_params[12] + L[5]) * Tz(L[4] + est_params[13]) * Rz(est_params[14]) * Ry(q4 + est_params[15]) * Tx(est_params[16]) * Tz(est_params[17]) * Rz(est_params[18]) * Rx(q5 + est_params[19]) * Tz(est_params[20]) * Rz(est_params[21]) * Ry(q6 + est_params[22]) * Ty(L[6])

def measure_est():
    est_params[3] = 0
    Tbr = T_base_est() * T_robot_est()
    Ttool = T_tool_est()
    return np.array([Tbr * Ttool[0], Tbr * Ttool[1], Tbr * Ttool[2]])

def J(tool_num):
    est_params[3] = 0
    est_params[4] = 0
    # J1 = T_base_est() * Tz(L[0] + est_params[3]) * dTz() * Rz(q1 + est_params[4]) * Tx(est_params[5]) * Ty(L[1] + est_params[6]) * Ry(est_params[7]) * Rx(q2 + est_params[8]) * Tz(L[2] + est_params[9]) * Ry(est_params[10]) * Rz(est_params[11]) * Rx(q3 + est_params[12]) * Ty(L[3] + est_params[13] + L[5]) * Tz(L[4] + est_params[4]) * Rz(est_params[15]) * Ry(q4 + est_params[16]) * Tx(est_params[17]) * Tz(est_params[18]) * Rz(est_params[19]) * Rx(q5 + est_params[20]) * Tz(est_params[21]) * Rz(est_params[22]) * Ry(q6 + est_params[23]) * Ty(L[6]) * T_tool_est()[tool_num]
    J2 = T_base_est() * Rz(q1 + est_params[3]) * dRz() * Tx(est_params[4]) * \
    Ty(L[1] + est_params[5]) * Ry(est_params[6]) * Rx(q2 + est_params[7]) * \
    Tz(L[2] + est_params[8]) * Ry(est_params[9]) * Rz(est_params[10]) * \
    Rx(q3 + est_params[11]) * Ty(L[3] + est_params[12] + L[5]) * \
    Tz(L[4] + est_params[13]) * Rz(est_params[14]) * Ry(q4 + est_params[15]) * \
    Tx(est_params[16]) * Tz(est_params[17]) * Rz(est_params[18]) * \
    Rx(q5 + est_params[19]) * Tz(est_params[20]) * Rz(est_params[21]) * \
    Ry(q6 + est_params[22]) * Ty(L[6]) * T_tool_est()[tool_num]

    J3 = T_base_est() * Rz(q1 + est_params[3]) * Tx(est_params[4]) * dTx() * \
    Ty(L[1] + est_params[5]) * Ry(est_params[6]) * Rx(q2 + est_params[7]) * \
    Tz(L[2] + est_params[8]) * Ry(est_params[9]) * Rz(est_params[10]) * \
    Rx(q3 + est_params[11]) * Ty(L[3] + est_params[12] + L[5]) * \
    Tz(L[4] + est_params[13]) * Rz(est_params[14]) * Ry(q4 + est_params[15]) * \
    Tx(est_params[16]) * Tz(est_params[17]) * Rz(est_params[18]) * \
    Rx(q5 + est_params[19]) * Tz(est_params[20]) * Rz(est_params[21]) * \
    Ry(q6 + est_params[22]) * Ty(L[6]) * T_tool_est()[tool_num]

    J4 = T_base_est() * Rz(q1 + est_params[3]) * Tx(est_params[4]) * \
    Ty(L[1] + est_params[5]) * dTy() * Ry(est_params[6]) * Rx(q2 + est_params[7]) * \
    Tz(L[2] + est_params[8]) * Ry(est_params[9]) * Rz(est_params[10]) * \
    Rx(q3 + est_params[11]) * Ty(L[3] + est_params[12] + L[5]) * \
    Tz(L[4] + est_params[13]) * Rz(est_params[14]) * Ry(q4 + est_params[15]) * \
    Tx(est_params[16]) * Tz(est_params[17]) * Rz(est_params[18]) * \
    Rx(q5 + est_params[19]) * Tz(est_params[20]) * Rz(est_params[21]) * \
    Ry(q6 + est_params[22]) * Ty(L[6]) * T_tool_est()[tool_num]

    J5 = T_base_est() * Rz(q1 + est_params[3]) * Tx(est_params[4]) * \
    Ty(L[1] + est_params[5]) * Ry(est_params[6]) * dRy() * Rx(q2 + est_params[7]) * \
    Tz(L[2] + est_params[8]) * Ry(est_params[9]) * Rz(est_params[10]) * \
    Rx(q3 + est_params[11]) * Ty(L[3] + est_params[12] + L[5]) * \
    Tz(L[4] + est_params[13]) * Rz(est_params[14]) * Ry(q4 + est_params[15]) * \
    Tx(est_params[16]) * Tz(est_params[17]) * Rz(est_params[18]) * \
    Rx(q5 + est_params[19]) * Tz(est_params[20]) * Rz(est_params[21]) * \
    Ry(q6 + est_params[22]) * Ty(L[6]) * T_tool_est()[tool_num]

    J6 = T_base_est() * Rz(q1 + est_params[3]) * Tx(est_params[4]) * \
    Ty(L[1] + est_params[5]) * Ry(est_params[6]) * Rx(q2 + est_params[7]) * \
    dRx() * Tz(L[2] + est_params[8]) * Ry(est_params[9]) * Rz(est_params[10]) * \
    Rx(q3 + est_params[11]) * Ty(L[3] + est_params[12] + L[5]) * \
    Tz(L[4] + est_params[13]) * Rz(est_params[14]) * Ry(q4 + est_params[15]) * \
    Tx(est_params[16]) * Tz(est_params[17]) * Rz(est_params[18]) * \
    Rx(q5 + est_params[19]) * Tz(est_params[20]) * Rz(est_params[21]) * \
    Ry(q6 + est_params[22]) * Ty(L[6]) * T_tool_est()[tool_num]

    J7 = T_base_est() * Rz(q1 + est_params[3]) * Tx(est_params[4]) * \
    Ty(L[1] + est_params[5]) * Ry(est_params[6]) * Rx(q2 + est_params[7]) * \
    Tz(L[2] + est_params[8]) * dTz() * Ry(est_params[9]) * Rz(est_params[10]) * \
    Rx(q3 + est_params[11]) * Ty(L[3] + est_params[12] + L[5]) * \
    Tz(L[4] + est_params[13]) * Rz(est_params[14]) * Ry(q4 + est_params[15]) * \
    Tx(est_params[16]) * Tz(est_params[17]) * Rz(est_params[18]) * \
    Rx(q5 + est_params[19]) * Tz(est_params[20]) * Rz(est_params[21]) * \
    Ry(q6 + est_params[2]) * Ty(L[6]) * T_tool_est()[tool_num]

    J8 = T_base_est() * Rz(q1 + est_params[3]) * Tx(est_params[4]) * \
    Ty(L[1] + est_params[5]) * Ry(est_params[6]) * Rx(q2 + est_params[7]) * \
    Tz(L[2] + est_params[8]) * Ry(est_params[9]) * dRy() * Rz(est_params[10]) * \
    Rx(q3 + est_params[11]) * Ty(L[3] + est_params[12] + L[5]) * \
    Tz(L[4] + est_params[13]) * Rz(est_params[14]) * Ry(q4 + est_params[15]) * \
    Tx(est_params[16]) * Tz(est_params[17]) * Rz(est_params[18]) * \
    Rx(q5 + est_params[19]) * Tz(est_params[20]) * Rz(est_params[21]) * \
    Ry(q6 + est_params[22]) * Ty(L[6]) * T_tool_est()[tool_num]

    J9 = T_base_est() * Rz(q1 + est_params[3]) * Tx(est_params[4]) * \
    Ty(L[1] + est_params[5]) * Ry(est_params[6]) * Rx(q2 + est_params[7]) * \
    Tz(L[2] + est_params[8]) * Ry(est_params[9]) * Rz(est_params[10]) * dRz() * \
    Rx(q3 + est_params[11]) * Ty(L[3] + est_params[12] + L[5]) * \
    Tz(L[4] + est_params[13]) * Rz(est_params[14]) * Ry(q4 + est_params[15]) * \
    Tx(est_params[16]) * Tz(est_params[17]) * Rz(est_params[18]) * \
    Rx(q5 + est_params[19]) * Tz(est_params[20]) * Rz(est_params[21]) * \
    Ry(q6 + est_params[22]) * Ty(L[6]) * T_tool_est()[tool_num]

    J10 = T_base_est() * Rz(q1 + est_params[3]) * Tx(est_params[4]) * \
    Ty(L[1] + est_params[5]) * Ry(est_params[6]) * Rx(q2 + est_params[7]) * \
    Tz(L[2] + est_params[8]) * Ry(est_params[9]) * Rz(est_params[10]) * \
    Rx(q3 + est_params[11]) * dRx() * Ty(L[3] + est_params[12] + L[5]) * \
    Tz(L[4] + est_params[13]) * Rz(est_params[14]) * Ry(q4 + est_params[15]) * \
    Tx(est_params[16]) * Tz(est_params[17]) * Rz(est_params[18]) * \
    Rx(q5 + est_params[19]) * Tz(est_params[20]) * Rz(est_params[21]) * \
    Ry(q6 + est_params[22]) * Ty(L[6]) * T_tool_est()[tool_num]

    J11 = T_base_est() * Rz(q1 + est_params[3]) * Tx(est_params[4]) * \
    Ty(L[1] + est_params[5]) * Ry(est_params[6]) * Rx(q2 + est_params[7]) * \
    Tz(L[2] + est_params[8]) * Ry(est_params[9]) * Rz(est_params[10]) * \
    Rx(q3 + est_params[11]) * Ty(L[3] + est_params[12] + L[5]) * dTy() * \
    Tz(L[4] + est_params[13]) * Rz(est_params[14]) * Ry(q4 + est_params[15]) * \
    Tx(est_params[16]) * Tz(est_params[17]) * Rz(est_params[18]) * \
    Rx(q5 + est_params[19]) * Tz(est_params[20]) * Rz(est_params[21]) * \
    Ry(q6 + est_params[22]) * Ty(L[6]) * T_tool_est()[tool_num]

    J12 = T_base_est() * Rz(q1 + est_params[3]) * Tx(est_params[4]) * \
    Ty(L[1] + est_params[5]) * Ry(est_params[6]) * Rx(q2 + est_params[7]) * \
    Tz(L[2] + est_params[8]) * Ry(est_params[9]) * Rz(est_params[10]) * \
    Rx(q3 + est_params[11]) * Ty(L[3] + est_params[12] + L[5]) * \
    Tz(L[4] + est_params[13]) * dTz() * Rz(est_params[14]) * \
    Ry(q4 + est_params[15]) * Tx(est_params[16]) * Tz(est_params[17]) * \
    Rz(est_params[18]) * Rx(q5 + est_params[19]) * Tz(est_params[20]) * \
    Rz(est_params[21]) * Ry(q6 + est_params[22]) * Ty(L[6]) * T_tool_est()[tool_num]

    J13 = T_base_est() * Rz(q1 + est_params[3]) * Tx(est_params[4]) * \
    Ty(L[1] + est_params[5]) * Ry(est_params[6]) * Rx(q2 + est_params[7]) * \
    Tz(L[2] + est_params[8]) * Ry(est_params[9]) * Rz(est_params[10]) * \
    Rx(q3 + est_params[11]) * Ty(L[3] + est_params[12] + L[5]) * \
    Tz(L[4] + est_params[13]) * Rz(est_params[14]) * dRz() * \
    Ry(q4 + est_params[15]) * Tx(est_params[16]) * Tz(est_params[17]) * \
    Rz(est_params[18]) * Rx(q5 + est_params[19]) * Tz(est_params[20]) * \
    Rz(est_params[21]) * Ry(q6 + est_params[22]) * Ty(L[6]) * T_tool_est()[tool_num]

    J14 = T_base_est() * Rz(q1 + est_params[3]) * Tx(est_params[4]) * \
    Ty(L[1] + est_params[5]) * Ry(est_params[6]) * Rx(q2 + est_params[7]) * \
    Tz(L[2] + est_params[8]) * Ry(est_params[9]) * Rz(est_params[10]) * \
    Rx(q3 + est_params[11]) * Ty(L[3] + est_params[12] + L[5]) * \
    Tz(L[4] + est_params[13]) * Rz(est_params[14]) * Ry(q4 + est_params[15]) * \
    dRy() * Tx(est_params[16]) * Tz(est_params[17]) * Rz(est_params[18]) * \
    Rx(q5 + est_params[19]) * Tz(est_params[20]) * Rz(est_params[21]) * \
    Ry(q6 + est_params[22]) * Ty(L[6]) * T_tool_est()[tool_num]

    J15 = T_base_est() * Rz(q1 + est_params[3]) * Tx(est_params[4]) * \
    Ty(L[1] + est_params[5]) * Ry(est_params[6]) * Rx(q2 + est_params[7]) * \
    Tz(L[2] + est_params[8]) * Ry(est_params[9]) * Rz(est_params[10]) * \
    Rx(q3 + est_params[11]) * Ty(L[3] + est_params[12] + L[5]) * \
    Tz(L[4] + est_params[13]) * Rz(est_params[14]) * Ry(q4 + est_params[15]) * \
    Tx(est_params[16]) * dTx() * Tz(est_params[17]) * Rz(est_params[18]) * \
    Rx(q5 + est_params[19]) * Tz(est_params[20]) * Rz(est_params[21]) * \
    Ry(q6 + est_params[22]) * Ty(L[6]) * T_tool_est()[tool_num]

    J16 = T_base_est() * Rz(q1 + est_params[3]) * Tx(est_params[4]) * \
    Ty(L[1] + est_params[5]) * Ry(est_params[6]) * Rx(q2 + est_params[7]) * \
    Tz(L[2] + est_params[8]) * Ry(est_params[9]) * Rz(est_params[10]) * \
    Rx(q3 + est_params[11]) * Ty(L[3] + est_params[12] + L[5]) * \
    Tz(L[4] + est_params[13]) * Rz(est_params[14]) * Ry(q4 + est_params[15]) * \
    Tx(est_params[16]) * Tz(est_params[17]) * dTz() * Rz(est_params[18]) * \
    Rx(q5 + est_params[19]) * Tz(est_params[20]) * Rz(est_params[21]) * \
    Ry(q6 + est_params[22]) * Ty(L[6]) * T_tool_est()[tool_num]

    J17 = T_base_est() * Rz(q1 + est_params[3]) * Tx(est_params[4]) * \
    Ty(L[1] + est_params[5]) * Ry(est_params[6]) * Rx(q2 + est_params[7]) * \
    Tz(L[2] + est_params[8]) * Ry(est_params[9]) * Rz(est_params[10]) * \
    Rx(q3 + est_params[11]) * Ty(L[3] + est_params[12] + L[5]) * \
    Tz(L[4] + est_params[13]) * Rz(est_params[14]) * Ry(q4 + est_params[15]) * \
    Tx(est_params[16]) * Tz(est_params[17]) * Rz(est_params[18]) * dRz() * \
    Rx(q5 + est_params[19]) * Tz(est_params[20]) * Rz(est_params[21]) * \
    Ry(q6 + est_params[22]) * Ty(L[6]) * T_tool_est()[tool_num]

    J18 = T_base_est() * Rz(q1 + est_params[3]) * Tx(est_params[4]) * \
    Ty(L[1] + est_params[5]) * Ry(est_params[6]) * Rx(q2 + est_params[7]) * \
    Tz(L[2] + est_params[8]) * Ry(est_params[9]) * Rz(est_params[10]) * \
    Rx(q3 + est_params[11]) * Ty(L[3] + est_params[12] + L[5]) * \
    Tz(L[4] + est_params[13]) * Rz(est_params[14]) * Ry(q4 + est_params[15]) * \
    Tx(est_params[16]) * Tz(est_params[17]) * Rz(est_params[18]) * \
    Rx(q5 + est_params[19]) * dRz() * Tz(est_params[20]) * Rz(est_params[21]) * \
    Ry(q6 + est_params[22]) * Ty(L[6]) * T_tool_est()[tool_num]

    J19 = T_base_est() * Rz(q1 + est_params[3]) * Tx(est_params[4]) * \
    Ty(L[1] + est_params[5]) * Ry(est_params[6]) * Rx(q2 + est_params[7]) * \
    Tz(L[2] + est_params[8]) * Ry(est_params[9]) * Rz(est_params[10]) * \
    Rx(q3 + est_params[11]) * Ty(L[3] + est_params[12] + L[5]) * \
    Tz(L[4] + est_params[13]) * Rz(est_params[14]) * Ry(q4 + est_params[15]) * \
    Tx(est_params[16]) * Tz(est_params[17]) * Rz(est_params[18]) * \
    Rx(q5 + est_params[19]) * Tz(est_params[20]) * dTz() * Rz(est_params[21]) * \
    Ry(q6 + est_params[22]) * Ty(L[6]) * T_tool_est()[tool_num]

    J20 = T_base_est() * Rz(q1 + est_params[3]) * Tx(est_params[4]) * \
    Ty(L[1] + est_params[5]) * Ry(est_params[6]) * Rx(q2 + est_params[7]) * \
    Tz(L[2] + est_params[8]) * Ry(est_params[9]) * Rz(est_params[10]) * \
    Rx(q3 + est_params[11]) * Ty(L[3] + est_params[12] + L[5]) * \
    Tz(L[4] + est_params[13]) * Rz(est_params[14]) * Ry(q4 + est_params[15]) * \
    Tx(est_params[16]) * Tz(est_params[17]) * Rz(est_params[18]) * \
    Rx(q5 + est_params[19]) * Tz(est_params[20]) * Rz(est_params[21]) * dRz() * \
    Ry(q6 + est_params[22]) * Ty(L[6]) * T_tool_est()[tool_num]

    J21 = T_base_est() * Rz(q1 + est_params[3]) * Tx(est_params[4]) * \
    Ty(L[1] + est_params[5]) * Ry(est_params[6]) * Rx(q2 + est_params[7]) * \
    Tz(L[2] + est_params[8]) * Ry(est_params[9]) * Rz(est_params[10]) * \
    Rx(q3 + est_params[11]) * Ty(L[3] + est_params[12] + L[5]) * \
    Tz(L[4] + est_params[13]) * Rz(est_params[14]) * Ry(q4 + est_params[15]) * \
    Tx(est_params[16]) * Tz(est_params[17]) * Rz(est_params[18]) * \
    Rx(q5 + est_params[19]) * Tz(est_params[20]) * Rz(est_params[21]) * \
    Ry(q6 + est_params[22]) * dRy() * Ty(L[6]) * T_tool_est()[tool_num]

    T = [J2, J3, J4, J5, J6, J7, J8, J9, J10, J11, J12, J13, J14, J15, J16, J17, J18, J19, J20, J21]
    J = np.matrix(np.zeros((6, 20)))
    D_inv = inv(np.matrix(measure_est()[tool_num], dtype = 'float'))
    for i in range(0, len(T)):
        J[0, i] = T[i][0, 3]
        J[1, i] = T[i][1, 3]
        J[2, i] = T[i][2, 3]
        J[3, i] = (T[i] * D_inv)[2, 1]
        J[4, i] = (T[i] * D_inv)[0, 2]
        J[5, i]= (T[i] * D_inv)[1, 0]
    return J

if __name__ == '__main__':
    L = [670, 312, 1075, 155.5, 225, 1124.5, 215]
    Lt = 100
    x0, y0, z0 = 0, 0, 1

    angle_lims = [(-pi / 2, pi / 2), (-pi / 2, pi / 2), (-pi / 2, pi / 2), (-pi / 2, pi / 2), (-pi, pi), (-pi, pi)]

    q1_err = ang_err()
    q2_err = ang_err()
    q3_err = ang_err()
    q4_err = ang_err()
    q5_err = ang_err()
    q6_err = ang_err()

    base_x_err = base_err()
    base_y_err = base_err()
    base_z_err = base_err()

    txL1_err = length_err()
    tyL1_err = length_err()
    ryL1_err = ang_err()

    tzL2_err = length_err()
    ryL2_err = ang_err()
    rzL2_err = ang_err()

    tyL3_err = length_err()
    tzL3_err = length_err()
    rzL3_err = ang_err()

    txL4_err = length_err()
    tzL4_err = length_err()
    rzL4_err = ang_err()

    tzL5_err = length_err()
    rzL5_err = ang_err()

    t1x_err = length_err()
    t1y_err = length_err()
    t1z_err = length_err()

    t2x_err = length_err()
    t2y_err = length_err()
    t2z_err = length_err()

    t3x_err = length_err()
    t3y_err = length_err()
    t3z_err = length_err()

    true_params = np.array([base_x_err, base_y_err, base_z_err, q1_err, txL1_err, tyL1_err, ryL1_err, q2_err, tzL2_err, ryL2_err, rzL2_err, q3_err, tyL3_err, tzL3_err, rzL3_err, q4_err, txL4_err, tzL4_err, rzL4_err, q5_err, tzL5_err, rzL5_err, q6_err, t1x_err, t1y_err, t1z_err, t2x_err, t2y_err, t2z_err, t3x_err, t3y_err, t3z_err])
    print("True parameters values")
    print(true_params)

    est_params = np.zeros(32)

    tests = []
    k = 0
    while k < 10:
        for i in range(0, 30):
            cur_test = []
            for j in range(0, 6):
                cur_test.append(random.uniform(angle_lims[j][0], angle_lims[j][1]))
            tests.append(cur_test)

        s1 = np.matrix(np.zeros((15, 15)))
        s2 = np.matrix(np.zeros((15, 1)))
        params_copy = np.copy(est_params)
        est_params = np.zeros(33)
        for cur_test in tests:
            q1, q2, q3, q4, q5, q6 = cur_test[0], cur_test[1], cur_test[2], cur_test[3], cur_test[4], cur_test[5]

            T_rob = T_robot_est()

            dp1, dp2, dp3 = measure_true() - measure_est()
            dp1 = dp1[0:3, 3]
            dp2 = dp2[0:3, 3]
            dp3 = dp3[0:3, 3]
            dp = transpose(np.matrix(np.concatenate([dp1, dp2, dp3])))

            p_r = T_rob[0:3, 3]
            R_r = T_rob[0:3, 0:3]

            p_r_ss = np.matrix(np.zeros((3, 3)))
            p_r_ss[0, 1] = -p_r[2]
            p_r_ss[1, 0] = p_r[2]
            p_r_ss[0, 2] = p_r[1]
            p_r_ss[2, 0] = -p_r[1]
            p_r_ss[2, 1] = p_r[0]
            p_r_ss[1, 2] = -p_r[0]
            p_r_ss = transpose(p_r_ss)

            A1 = np.matrix(np.concatenate([np.eye(3), p_r_ss, R_r, np.zeros((3, 6))], axis=1))
            A2 = np.matrix(np.concatenate([np.eye(3), p_r_ss, np.zeros((3, 3)), R_r, np.zeros((3, 3))], axis=1))
            A3 = np.matrix(np.concatenate([np.eye(3), p_r_ss, np.zeros((3, 6)), R_r], axis=1))
            A = np.matrix(np.concatenate([A1, A2, A3]))
            s1 += transpose(A) * A
            s2 += transpose(A) * dp
        est_params = params_copy
        if det(s1) == 0:
            continue
        b = inv(s1) * s2
        pbx = b[0]
        pby = b[1]
        pbz = b[2]

        est_params[0] = (x0 + base_x_err) - pbx
        est_params[1] = (y0 + base_y_err) - pby
        est_params[2] = (z0 + base_z_err) - pbz

        pt1x = b[6]
        pt1y = b[7]
        pt1z = b[8]

        est_params[23] = T_tool_true()[0][0:3, 3][0] - pt1x
        est_params[24] = T_tool_true()[0][0:3, 3][1] - pt1y
        est_params[25] = T_tool_true()[0][0:3, 3][2] - pt1z

        pt2x = b[9]
        pt2y = b[10]
        pt2z = b[11]

        est_params[26] = T_tool_true()[1][0:3, 3][0] - pt2x
        est_params[27] = T_tool_true()[1][0:3, 3][1] - pt2y
        est_params[28] = T_tool_true()[1][0:3, 3][2] - pt2z

        pt3x = b[12]
        pt3y = b[13]
        pt3z = b[14]

        est_params[29] = T_tool_true()[2][0:3, 3][0] - pt3x
        est_params[30] = T_tool_true()[2][0:3, 3][1] - pt3y
        est_params[31] = T_tool_true()[2][0:3, 3][2] - pt3z

        sum1 = np.matrix(np.zeros((20, 20)))
        sum2 = np.matrix(np.zeros((20, 1)))

        for cur_test in tests:
            q1, q2, q3, q4, q5, q6 = cur_test[0], cur_test[1], cur_test[2], cur_test[3], cur_test[4], cur_test[5]
            T_rob = T_robot_est()
            for j in range(0, 3):
                dp = measure_true()[j] - measure_est()[j]
                dp = transpose(np.matrix(np.concatenate([transpose(dp[0:3, 3])])))
                cur_j = J(j)[0:3]
                sum1 += transpose(cur_j) * cur_j
                sum2 += transpose(cur_j) * dp
        if det(sum1) != 0:
            pi = inv(sum1) * sum2
            est_params[3:23] = pi.reshape(20)
            print("Tool position difference:")
            print(dp)
            print("Params difference:")
            print(np.abs(true_params - est_params))
            print()
            k += 1

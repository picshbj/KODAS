from math import *
import numpy as np

def GRS80toTM(latitude, longitude):
    # GRS80 to TM parameters
    m_dA0 = 6378137
    m_dE1 = 0.00673949674226
    m_dPI = 3.141592653589793

    m_dE2 = 0.00669437999013
    m_dE4 = m_dE2 * m_dE2
    m_dE6 = m_dE2 * m_dE4

    m_dM0 = 1 - m_dE2/4 - 3*m_dE4/64 - 5*m_dE6/256
    m_dM1 = 3*m_dE2/8 + 3*m_dE4/32 + 45*m_dE6/1024
    m_dM2 = 15*m_dE4/256 + 45*m_dE6/1024
    m_dM3 = 35*m_dE6/3072

    m_dX0 = 600000
    m_dY0 = 200000

    m_dM38 = 4207498.0193
    
    B3 = latitude * m_dPI / 180
    B3_0 = 38 * m_dPI / 180

    dTanT = tan(B3)
    dCosT = cos(B3)
    dSinT = sin(B3)

    T = dTanT * dTanT
    C = m_dE2 / (1-m_dE2) * dCosT * dCosT
    A = (longitude - 127.0) * m_dPI / 180  * dCosT
    N = m_dA0 / sqrt(1-m_dE2 * dSinT * dSinT)

    M = m_dA0 * ( m_dM0 * B3 - m_dM1 * sin(2*B3) + m_dM2 * sin(4*B3) - m_dM3 * sin(6*B3) )
    M0 = m_dA0 * ( m_dM0 * B3_0 - m_dM1 * sin(2*B3_0) + m_dM2 * sin(4*B3_0) - m_dM3 * sin(6*B3_0) )
    
    dY1 = pow(A, 3)/6 * (1-T+C)
    dY2 = pow(A, 5)/120 * (5-18*T+T*T+72*C-58*m_dE2)

    dX1 = A*A/2
    dX2 = pow(A,4)/24 * (5-T+9*C+4*C*C)
    dX3 = pow(A,6)/720 * (61-58*T+T*T+600*C-330*m_dE2)

    Y = m_dY0 + N * ( A + dY1 + dY2)
    X = m_dX0 + (M - M0 + N * dTanT *(dX1 + dX2 + dX3))

    return X, Y


def TR(R, T, P):
    rx = R[0]
    ry = R[1]
    rz = R[2]
    tx = T[0]
    ty = T[1]
    tz = T[2]
    px = P[0]
    py = P[1]
    pz = P[2]

    rxMat = np.array([[1, 0, 0], 
    [0, cos(rx), -sin(rx)], 
    [0, sin(rx), cos(rx)]])

    ryMat = np.array([[cos(ry), 0, sin(ry)],
    [0, 1, 0],
    [-sin(ry), 0, cos(ry)]])

    rzMat = np.array([[cos(rz), -sin(rz), 0],
    [sin(rz), cos(rz), 0],
    [0, 0, 1]])

    p = np.array([[px],[py],[pz]])

    t = np.array([[tx],[ty],[tz]])

    return rxMat.dot(ryMat.dot(rzMat.dot(p))) + t

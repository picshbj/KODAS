# Written by Jeffrey Hong <hbj723@kitech.re.kr>
# last revision: 2019.07.30

# Note: 
# WGS84ToTM and GRS80toTM methods are converted from GpsPositionTM.cpp written by Seungho Baeg
# Central Point: N38, E127


from math import *
import numpy as np

# common variables
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


def WGS84toTM(latitude, longitude):    
    A0 = 6378137
    E0 = 0.00669437999013
    E1 = 0.00673949674226

    C0 = 6399593.6257584402 		# (A0*sqrt(1+E1)) 
    P= 57.2957795130823			# (180/3.141592653589793)       

    dPow2P = 3282.8063500117441	# pow(P,2)
    dPow3P = 188090.94881441945	# pow(P,3)
    dPow4P = 10776817.531677430	# pow(P,4)
    dPow5P = 617466161.14771008	# pow(P,5)
    dPow6P = 35378205025.908562	# pow(P,6)
    dPow7P = 2027021834733.0776	# pow(P,7)
    dPow8P = 116139796111070.03	# pow(P,8)

    A = 1.0050525017881968			# (1+3*E0/4+45*pow(E0,2)/64+175*pow(E0,3)/256+11025*pow(E0,4)/16384+43659*pow(E0,5)/65536)     
    B = 0.0050631085972301180		# (3*E0/4+15*pow(E0,2)/16+525*pow(E0,3)/512+2205*pow(E0,4)/2048+72765*pow(E0,5)/65536)
    CC = 1.0627590158675697E-5		# (15*pow(E0,2)/64+105*pow(E0,3)/256+2205*pow(E0,4)/4096+10395*pow(E0,5)/16384)
    D = 2.0820378264342709E-8		# (35*pow(E0,3)/512+315*pow(E0,4)/2048+31185*pow(E0,5)/131072)
    E = 3.9323712937761384E-11		# (315*pow(E0,4)/16384+3465*pow(E0,5)/65536)

    F = 7.1084532288702243E-14		# (693*pow(E0,5)/131072)
    DL2 = 2.2165681500327983		# (127/P)

    S = 4207498.0191544499 			# (A0*(1-E0)*(S1+S2+S3))	

    if(latitude > 10):	# Degree Unit Data Input
        B3 = latitude /P 		#  * pi / 180
        DL3 = longitude/P		#  * pi / 180
    else: # Radian Unit Data Input
        B3 = latitude 		
        DL3 = longitude

    T=tan(B3)

    dPow2T = T * T
    dPow4T = dPow2T * dPow2T
    dPow6T = dPow4T * dPow2T

    dCosB3 = cos(B3)
    dSinB3 = sin(B3)

    dPow2CosB3 = dCosB3 * dCosB3
    dPow3CosB3 = dCosB3 * dPow2CosB3
    dPow5CosB3 = dPow3CosB3 * dPow2CosB3
    dPow7CosB3 = dPow5CosB3 * dPow2CosB3

    DL=(DL3-DL2)*P     
    V=sqrt(1+E1*dPow2CosB3)
    DN=C0/V

    dPow2DL = DL * DL
    dPow3DL = dPow2DL* DL
    dPow4DL = dPow2DL * dPow2DL
    dPow5DL = dPow4DL * DL
    dPow6DL = dPow4DL * dPow2DL
    dPow7DL = dPow6DL * DL
    dPow8DL = dPow6DL * dPow2DL

    S4=A*B3-B*sin(2*B3)/2
    S5=CC*sin(4*B3)/4-D*sin(6*B3)/6
    S6=E*sin(8*B3)/8-F*sin(10*B3)/10
    S0=A0*(1-E0)*(S4+S5+S6)
    X1=(dPow2DL*DN*sin(B3)*cos(B3))/(2*dPow2P)

    DN1=E1*pow(cos(B3),2)
    X2=dPow4DL*DN*dSinB3*dPow3CosB3*(5-dPow2T+9*DN1+4*pow(DN1,2))/(24*dPow4P)
    X3=dPow6DL*DN*dSinB3*dPow5CosB3*(61-58*dPow2T+dPow4T+270*DN1-330*dPow2T*DN1)/(720*dPow6P)
    X4=dPow8DL*DN*dSinB3*dPow7CosB3/(40320*dPow8P)
    X5=-1385+3111*dPow2T-543*dPow4T+dPow6T
    X0=S0+X1+X2+X3-X4*X5
    X6=(X0-S)
    X=X6+600000

    Y1=dPow3DL*DN*dPow3CosB3*(1-dPow2T+DN1)/(6*dPow3P)
    Y2=dPow5DL*DN*dPow5CosB3*(5-18*dPow2T+dPow4T+14*DN1-58*dPow2T*DN1)/(120*dPow5P)
    Y3=dPow7DL*DN*dPow7CosB3*(-61+479*dPow2T-179*dPow4T+dPow6T)/(5040*dPow7P)
    Y6=(DL*DN*dCosB3/P+Y1+Y2-Y3)
    Y=Y6+200000

    return X, Y


def GRS80toTM(latitude, longitude):
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

    return rzMat.dot(ryMat.dot(rxMat.dot(p))) + t

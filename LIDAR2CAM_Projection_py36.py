import cv2
import numpy as np
import sys
import os

# cam1
RTMat = np.zeros((3,4),dtype='float32')
RTMat[0] = [0.9995, 0.0177,    -0.0260,    -15.5]
RTMat[1] = [-0.0172, 0.9997,     0.0177,   -85.0]
RTMat[2] = [0.0263, -0.0172,    0.9995,    -53.0]
cam_mtx = np.zeros((3,3), dtype='float32')
cam_mtx[0] = [949.0131, 0, 631.8859]
cam_mtx[1] = [0, 957.1422, 472.0568]
cam_mtx[2] = [0, 0, 1]
P_lidar2cam = cam_mtx.dot(RTMat)

def getRotMat(rx, ry, rz):
	rxMat = np.array([	(1, 0, 0),
    					(0, np.cos(rx), -np.sin(rx)),
 						(0, np.sin(rx), np.cos(rx))	]).astype('float32')

	ryMat = np.array([	(np.cos(ry), 0, np.sin(ry)),
    					(0, 1, 0),
    					(-np.sin(ry), 0, np.cos(ry))]).astype('float32')

	rzMat = np.array([	(np.cos(rz), -np.sin(rz), 0),
    					(np.sin(rz), np.cos(rz), 0),
    					(0, 0, 1)]).astype('float32')

	return np.dot(np.dot(rxMat, ryMat), rzMat)

def getMousePointt(event, x, y, flags, param):
    global mouseX, mouseY
    if event == cv2.EVENT_LBUTTONDBLCLK:
        mouseX, mouseY = x, y
        print(' %d %d' % (mouseX, mouseY), end='')

def changeCoordinationLidar2Cam(position):
	tmp = np.copy(position)
	if position.shape[0] == 4:
		position[0] = tmp[1]
		position[1] = -tmp[2]
		position[2] = -tmp[0]
	else:
		position[:,0] = tmp[:,1]
		position[:,1] = -tmp[:,2]
		position[:,2] = -tmp[:,0]
	return position


def getBoxPoints(point, l, w, h):
	l = l/2.0
	w = w/2.0
	h = h/2.0

	box = np.zeros((8,3), dtype='float32')
	box[0,:] = [point[0] + l, point[1] + w, point[2] + h]
	box[1,:] = [point[0] + l, point[1] + w, point[2] - h]
	box[2,:] = [point[0] + l, point[1] - w, point[2] + h]
	box[3,:] = [point[0] + l, point[1] - w, point[2] - h]
	box[4,:] = [point[0] - l, point[1] + w, point[2] + h]
	box[5,:] = [point[0] - l, point[1] + w, point[2] - h]
	box[6,:] = [point[0] - l, point[1] - w, point[2] + h]
	box[7,:] = [point[0] - l, point[1] - w, point[2] - h]
	
	return box

def project(p_in, T):
    # dimension of data and projection matrix
	dim_norm = T.shape[0]
	dim_proj = T.shape[1]

	# do transformation in homogeneous coordinates
	if p_in.shape[1] < dim_proj:
		p_in = np.c_[p_in, np.ones((p_in.shape[0],1))]
	p_out = p_in.dot(T.T)

	# normalize homogeneous coordinates
	for i in range(p_out.shape[0]):
		p_out[i,:] = p_out[i,:]/p_out[i,2]
	
	return p_out

def getProjectedBox(point, length, width, height):
	# get box points 
	boxPoints = getBoxPoints(point, length, width, height)

	# match lidar coordination with camera coordination
	boxPoints2Img = changeCoordinationLidar2Cam(boxPoints)

	# projection
	box8points = project(boxPoints2Img, P_lidar2cam)

	# get 2d box points
	box4points = np.zeros((4,2), dtype='int32')
	x_max = int(box8points[:,0].max())
	x_min = int(box8points[:,0].min())
	y_max = int(box8points[:,1].max())
	y_min = int(box8points[:,1].min())

	box4points[0,:] = [x_max, y_max]
	box4points[1,:] = [x_max, y_min]
	box4points[2,:] = [x_min, y_max]
	box4points[3,:] = [x_min, y_min]
	
	return box4points


# ####################################################################
# ## main
# ####################################################################

# # object initialize
# obj1 = np.array([-3172, 372, -185]).astype('float32')
# obj2 = np.array([-2455.5, -646.6, -173]).astype('float32')
# obj3 = np.array([-2299.0, -282.5, -183.0]).astype('float32')
# obj4 = np.array([-1034.0, 344.5, -165.0]).astype('float32')

# # get 2d box
# p_obj1 = getProjectedBox(obj1, 576.0, 195.0, 153.0)
# p_obj2 = getProjectedBox(obj2, 420.0, 151.0, 132.0)
# p_obj3 = getProjectedBox(obj3, 566.0, 187.0, 157.0)
# p_obj4 = getProjectedBox(obj4, 421.0, 177.0, 160.0)


# imgfname = 'K2K_000101.jpg'
# image = cv2.imread(imgfname, cv2.IMREAD_COLOR)
# cv2.rectangle(image, (p_obj1[0][0], p_obj1[0][1]), (p_obj1[3][0], p_obj1[3][1]), (0,0,255), 2)
# cv2.rectangle(image, (p_obj2[0][0], p_obj2[0][1]), (p_obj2[3][0], p_obj2[3][1]), (0,0,255), 2)
# cv2.rectangle(image, (p_obj3[0][0], p_obj3[0][1]), (p_obj3[3][0], p_obj3[3][1]), (0,0,255), 2)
# cv2.rectangle(image, (p_obj4[0][0], p_obj4[0][1]), (p_obj4[3][0], p_obj4[3][1]), (0,0,255), 2)

# cv2.imshow('image', image)
# cv2.waitKey(0)
# cv2.destroyAllWindows()
from DataFormat_CAMERA_py36 import *

# Camera setting
settings = dict()
settings['SETTING_RAWDATA'] = False
settings['SETTING_UNDIST'] = True

def setParams():
    # read camera parameters from file
    try:
        with open('camera_params.txt', 'r') as fin:
            data1 = fin.readline().split(',')
            data2 = fin.readline().split(',')
            data3 = fin.readline().split(',')
            data4 = fin.readline().split(',')
    except (IOError, ValueError) as e:
        print('no calibration parameter exist.')

    CamMat = [[float(data1[0]), float(data1[1]), float(data1[2])], [float(data1[3]), float(data1[4]), float(data1[5])], [float(data1[6]), float(data1[7]), float(data1[8])]]
    CamDistCoeffs = [float(data1[9]), float(data1[10]), float(data1[11]), float(data1[12]), float(data1[13])]
    Cam1Params = [CamMat, CamDistCoeffs]

    CamMat = [[float(data2[0]), float(data2[1]), float(data2[2])], [float(data2[3]), float(data2[4]), float(data2[5])], [float(data2[6]), float(data2[7]), float(data2[8])]]
    CamDistCoeffs = [float(data2[9]), float(data2[10]), float(data2[11]), float(data2[12]), float(data2[13])]
    Cam2Params = [CamMat, CamDistCoeffs]

    CamMat = [[float(data3[0]), float(data3[1]), float(data3[2])], [float(data3[3]), float(data3[4]), float(data3[5])], [float(data3[6]), float(data3[7]), float(data3[8])]]
    CamDistCoeffs = [float(data3[9]), float(data3[10]), float(data3[11]), float(data3[12]), float(data3[13])]
    Cam3Params = [CamMat, CamDistCoeffs]
    
    CamMat = [[float(data4[0]), float(data4[1]), float(data4[2])], [float(data4[3]), float(data4[4]), float(data4[5])], [float(data4[6]), float(data4[7]), float(data4[8])]]
    CamDistCoeffs = [float(data4[9]), float(data4[10]), float(data4[11]), float(data4[12]), float(data4[13])]
    Cam4Params = [CamMat, CamDistCoeffs]

    return Cam1Params, Cam2Params, Cam3Params, Cam4Params

if __name__ == '__main__':
    # init data
    params = setParams()
    datfile = 'BSDCAM_L_00_2019Y11M25D15H41m15s_FC.dat'
    cam = CCameraFrame(settings, datfile, params[0], 'output', 'FrontLeft')
    
    # get header
    header = cam.header

    # print header
    cam.printHeader()

    # convert raw data to readable data
    # when you call this function, you'll lose current frame pointer.
    # Do not call this function with getNextFrame function at the same time.
    # cam.convertAll()

    # free memory
    del cam
    
    #######################################################################

    # init data 
    # cam = CCameraFrame(settings, datfile, params[0], 'output', 'FrontLeft')
    
    # # read frame by frame
    # while not cam.EOF:
    #     frame = cam.getNextFrame()
    #     # write your code here..

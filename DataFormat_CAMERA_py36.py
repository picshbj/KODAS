import struct
import cv2
import numpy as np
import os
from DefinedNaviSensConst_py36 import *
from LIDAR2CAM_Projection_py36 import *

DEBUG = True
DEBUG = False

HEADER_CAMERA = 'I6BHIiI32s32sI'
HEADER_BITMAPINFOHEADER = 'I2l2H2I2l2I'

SIZE_HEADER_CAMERA = struct.calcsize(HEADER_CAMERA)
SIZE_HEADER_BITMAPINFOHEADER = struct.calcsize(HEADER_BITMAPINFOHEADER)
        
class sBITMAPINFOHEADER():
    def __init__(self, BITHEAD):
        # parsing BITMAPINFOHEADER
        if len(BITHEAD) is not SIZE_HEADER_BITMAPINFOHEADER:
            if(DEBUG): print ('Input bitmap info header data is not valid. Input data size is %d' % (len(BITHEAD)))
            self.frameStatus = False
        else:
            headerData = struct.unpack(HEADER_BITMAPINFOHEADER, BITHEAD)
            self.biSize = headerData[0]
            self.biWidth = headerData[1]
            self.biHeight = headerData[2]
            self.biPlanes = headerData[3]
            self.biBitCount = headerData[4]
            self.biCompression = headerData[5]
            self.biSizeImage = headerData[6]
            self.biXPelsPerMeter = headerData[7]
            self.biYPelsPerMeter = headerData[8]
            self.biClrUsed = headerData[9]
            self.biClrImportant = headerData[10]
            self.frameStatus = True
    def printHeader(self):
        try:
            print ('Size : %i' % (self.biSize))
            print ('Width : %i' % (self.biWidth))
            print ('Height : %i' % (self.biHeight))
            print ('Planes : %i' % (self.biPlanes))
            print ('BitCount : %i' % (self.biBitCount))
            print ('Compression : %i' % (self.biCompression))
            print ('Image Size : %i' % (self.biSizeImage))
            print ('X PelsPerMeter : %i' % (self.biXPelsPerMeter))
            print ('Y PelsPerMeter : %i' % (self.biYPelsPerMeter))
            print ('Clr Used : %i' % (self.biClrUsed))
            print ('Clr Important : %i' % (self.biClrImportant))

        except AttributeError:
            print ('Header is not exist.')

class sCameraHeader():
    def __init__(self, CAMHEAD):
        # parsing Camera Data
        if len(CAMHEAD) is not SIZE_HEADER_CAMERA+SIZE_HEADER_BITMAPINFOHEADER:
            if(DEBUG): print ('Input camera header data is not valid. Input data size is %d' % (len(CAMHEAD)))
            self.frameStatus = False
        else:
            headerData = struct.unpack(HEADER_CAMERA, CAMHEAD[:-40])
            self.dataFileVersion = headerData[0]
            self.dataSaveTime = headerData[1:8]
            self.INSInfoVersion = headerData[8]
            self.INSInfoSize = headerData[9]
            self.cameraID = headerData[10]
            self.cameraName = headerData[11].decode('utf-8')
            self.cameraSerialNo = headerData[12].decode('utf-8')
            self.imageDataVersion = headerData[13]
            self.ImageDataHeader = sBITMAPINFOHEADER(CAMHEAD[-40:])
            self.frameStatus = True
        
        with open('imageHeader.txt', 'w') as fout:
            fout.write('Data File Version : 0x%08X\n' % (self.dataFileVersion))
            fout.write('Data Save Time : 20%02d-%02d-%02d - %02dh%02dm%02d.%ds\n' % (self.dataSaveTime[0], self.dataSaveTime[1], self.dataSaveTime[2], self.dataSaveTime[3]+9, self.dataSaveTime[4], self.dataSaveTime[5], self.dataSaveTime[6]))
            fout.write('INS Info Version : 0x%08X\n' % (self.INSInfoVersion))
            fout.write('INS Info Size : %i\n' % (self.INSInfoSize))
            fout.write('Camera ID : %i\n' % (self.cameraID))
            fout.write('Camera Name : %s\n' % (self.cameraName))
            fout.write('Camera Serial Number : %s\n' % (self.cameraSerialNo))
            fout.write('Image Data Version : 0x%08X\n' % (self.imageDataVersion))

    def printHeader(self):
        try:
            print ('Data File Version : 0x%08X' % (self.dataFileVersion))
            print ('Data Save Time : 20%02d-%02d-%02d - %02dh%02dm%02d.%ds' % (self.dataSaveTime[0], self.dataSaveTime[1], self.dataSaveTime[2], self.dataSaveTime[3]+9, self.dataSaveTime[4], self.dataSaveTime[5], self.dataSaveTime[6]))
            print ('INS Info Version : 0x%08X' % (self.INSInfoVersion))
            print ('INS Info Size : %i' % (self.INSInfoSize))
            print ('Camera ID : %i' % (self.cameraID))
            print ('Camera Name : %s' % (self.cameraName))
            print ('Camera Serial Number : %s' % (self.cameraSerialNo))
            print ('Image Data Version : 0x%08X' % (self.imageDataVersion))
            self.ImageDataHeader.printHeader()

        except AttributeError:
            print ('Header is not exist.')

class sCameraData():
    def __init__(self, settings, CAM, header, params):
        self.settings = settings
        if len(CAM) != (72+header.ImageDataHeader.biSizeImage):
            if(DEBUG): print ('Input camera data is not valid. Input data size is %dbytes, expected %dbytes' % (len(CAM), 72+header.ImageDataHeader.biSizeImage))
            self.frameStatus = False
        else:
            h = header.ImageDataHeader.biHeight
            w = header.ImageDataHeader.biWidth

            frameData = struct.unpack('2I', CAM[0:8])
            self.sequencCount = frameData[0]
            self.saveInterval = frameData[1]
            self.INSData = sINSData(CAM[8:72])
            # header.ImageDataHeader.printHeader()
            b = int(len(CAM[72:])/(h*w))
            # print('%d = %dx%d, %d' % (len(CAM[72:]), h, w, b))            
            image = np.fromstring(CAM[72:], np.uint8).reshape((h, w, b))
            if b == 1:
                self.image = cv2.cvtColor(image, cv2.COLOR_BayerRG2RGB)
            elif b == 2:
                self.image = cv2.cvtColor(image, cv2.COLOR_YUV2BGR_YUYV)


            # undistort 
            if self.settings['SETTING_UNDIST']:
                camMat = np.matrix(params[0])
                distCoeffs = np.array(params[1])
                self.image = cv2.undistort(self.image, camMat, distCoeffs)

            self.frameStatus = True
    


class cCameraFrame():
    def __init__(self, settings, fileName, params, path, cameraName):
        self.settings = settings 
        
        # open file
        self.dataFile = open(fileName, 'rb')
        # outFile = open(fileName+'_box', 'wb')

        self.path = path
        self.cameraName = cameraName
        self.params = params

        self.EOF = False

        if not os.path.isdir(path):
            os.mkdir(path)

        # read header and register
        self.header = sCameraHeader(self.dataFile.read(SIZE_HEADER_CAMERA+SIZE_HEADER_BITMAPINFOHEADER))    # read 132 bytes for camera data header
        # self.printHeader()
        # read data frame and register
                  
                
    def findFrameIndex(self, time):
        obj = open(self.path+'\\Object\\ObjectDB.txt', 'r')
        index = 0
        timegap = 10000
        time = int(time)
        for line in obj:
            objData = line.split(' ')
            if timegap > abs(int(objData[1]) - time):
                timegap = abs(int(objData[1]) - time)
                index = objData[0]
                print(index)
            if time < int(objData[1])-200:
                return int(index)
        return int(index)
    
    def getNextFrame(self):
        frameData = self.dataFile.read(72+self.header.ImageDataHeader.biSizeImage)
        if frameData == b'':
            self.EOF = True
            try:
                dataFile.close()
            except Exception as e:
                pass
            return False
        else:
            frame = sCameraData(self.settings, frameData, self.header, self.params)
            return frame

    def drawBox(self, frame, frameIndex):
        iObjIndex = self.findFrameIndex(frame.INSData.getTime())

        obj = open(self.path+'\\Object\\ObjectDB.txt', 'r')
        for line in obj:
            objData = line.split(' ')
            for i in range(len(objData)):
                objData[i] = float(objData[i])
                
            if iObjIndex == int(objData[0]):
                if objData[4] > -350: continue

                p_obj = np.array([objData[4], objData[5], objData[6]]).astype('float32')  
                p_box = getProjectedBox(p_obj, objData[7], objData[8], objData[9])
                cv2.rectangle(frame.image, (p_box[0][0], p_box[0][1]), (p_box[3][0], p_box[3][1]), (0,0,255), 2)
        
        # cv2.imshow('image', frame)
        # cv2.waitKey(0)
        # cv2.destroyAllWindows()
    def convertAll(self):
        cnt = 1
        while not self.EOF:
            frame = self.getNextFrame()    
            if frame.frameStatus is True:
                self.convert(self.path+'\\'+self.cameraName, cnt, frame)
                cnt = cnt+1

    def convert(self, path, cnt, image):
        if not os.path.isdir(path):
            os.mkdir(path)
        
        logFile = open(path+'\\'+self.cameraName+'.txt', 'a')
        
        # Save log
        data = '%06d %d %d %d %s %.4f %.4f %.4f %.4f %.4f %.4f %.4f %.4f %.4f\n' % (cnt, image.saveInterval, image.INSData.dwDevStatus, image.INSData.nUpdateCount, image.INSData.getTime(), image.INSData.afVelocity[0], image.INSData.afVelocity[1], image.INSData.afVelocity[2], image.INSData.afOrient[0], image.INSData.afOrient[1], image.INSData.afOrient[2], image.INSData.adPosition[0], image.INSData.adPosition[1], image.INSData.adPosition[2])
        logFile.write(data)

        # draw box
        # self.drawBox(image, cnt)

        # Save as Image
        filename = '%s\\%06d.jpg' % (path, cnt)
        cv2.imwrite(filename, image.image)


        logFile.close()

    def printHeader(self):
        self.header.printHeader()

    def writeConvertedImageToDatFile(self):
        pass

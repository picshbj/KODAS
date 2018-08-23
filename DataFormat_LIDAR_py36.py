import struct
import cv2
import numpy as np
import os
from DefinedNaviSensConst_py36 import *

DEBUG = True
DEBUG = False

HEADER_LIDAR = 'I6BHIiI32s32s'
HEADER_LIDAR_SENSOR_INFO = '56s'
HEADER_LIDARMEASUREDATA = 'Q2I2H3I'
PCDDATA = '5h3H'

SIZE_HEADER_LIDAR = struct.calcsize(HEADER_LIDAR)
SIZE_HEADER_LIDAR_SENSOR_INFO = struct.calcsize(HEADER_LIDAR_SENSOR_INFO)
SIZE_HEADER_LIDARMEASUREDATA = struct.calcsize(HEADER_LIDARMEASUREDATA)
SIZE_PCDDATA = struct.calcsize(PCDDATA)


class sLidarSensorInfoHeader():
    def __init__(self, HEADER):
        self.LidarSensorInfoHeader = struct.unpack(HEADER_LIDAR_SENSOR_INFO, HEADER)
        # parsing LIDAR sensor info header
    def printHeader(self):
        print ('Lidar Sensor Info Header : %s' % (self.LidarSensorInfoHeader))
        
class sLidarHeader():
    def __init__(self, LIDARHEAD):
        # parsing LIDAR Header
        if len(LIDARHEAD) != SIZE_HEADER_LIDAR+SIZE_HEADER_LIDAR_SENSOR_INFO:
            if(DEBUG): print ('Input Lidar header data is not valid. Input data size is %d' % (len(LIDARHEAD)))
            self.frameStatus = False
        else:           
            headerData = struct.unpack(HEADER_LIDAR, LIDARHEAD[:SIZE_HEADER_LIDAR])
            self.dataFileVersion = headerData[0]
            self.dataSaveTime = headerData[1:8]
            self.INSInfoVersion = headerData[8]
            self.INSInfoSize = headerData[9]
            self.LidarID = headerData[10]
            self.LidarModelName = headerData[11].decode('utf-8')
            self.LidarSerialNo = headerData[12].decode('utf-8')
            self.LidarSensorInfoHeader = sLidarSensorInfoHeader(LIDARHEAD[SIZE_HEADER_LIDAR:])
            self.frameStatus = True

    def printHeader(self):
        try:
            print ('Data File Version : 0x%08X' % (self.dataFileVersion))
            print ('Data Save Time : 20%02d-%02d-%02d - %02dh%02dm%02d.%ds' % (self.dataSaveTime[0], self.dataSaveTime[1], self.dataSaveTime[2], self.dataSaveTime[3]+9, self.dataSaveTime[4], self.dataSaveTime[5], self.dataSaveTime[6]))
            print ('INS Info Version : 0x%08X' % (self.INSInfoVersion))
            print ('INS Info Size : %i' % (self.INSInfoSize))
            print ('Lidar ID : %i' % (self.LidarID))
            print ('Lidar Model Name : %s' % (self.LidarModelName))
            print ('Lidar Serial Number : %s' % (self.LidarSerialNo))
            self.LidarSensorInfoHeader.printHeader()

        except AttributeError:
            print ('Header is not exist.')

class sLidarMeasureData():
    def __init__(self, HEAD):
        # parsing BITMAPINFOHEADER
        if len(HEAD) is not SIZE_HEADER_LIDARMEASUREDATA:
            if(DEBUG): print ('Input measuredata header data is not valid. Input data size is %d' % (len(HEAD)))
            self.frameStatus = False
        else:
            headerData = struct.unpack(HEADER_LIDARMEASUREDATA, HEAD)
            self.qwTimeStamp = headerData[0]
            self.dwSensorStatus = headerData[1]
            self.dwMeasureCounter = headerData[2]
            self.wPcdDataType = headerData[3]
            self.wPcdDataSizeInByte = headerData[4]
            self.dwMaxPcdBuffCounts = headerData[5]
            self.dwMeasurePcdCounts = headerData[6]
            self.dwReserved = headerData[7]
            self.frameStatus = True
    def printHeader(self):
        try:
            print ('TimeStamp : %i' % (self.qwTimeStamp))
            print ('SensorStatus : %d' % (self.dwSensorStatus))
            print ('MeasureCounter : %d' % (self.dwMeasureCounter))
            print ('PcdDataType : %i' % (self.wPcdDataType))
            print ('PcdDataSizeInByte : %i' % (self.wPcdDataSizeInByte))
            print ('MaxPcdBuffCounts : %d' % (self.dwMaxPcdBuffCounts))
            print ('Reserved : %d' % (self.dwReserved))

        except AttributeError:
            print ('Header is not exist.')

class sPcdData():
    def __init__(self, DATA):
        if len(DATA) != SIZE_PCDDATA:
            if(DEBUG): print ('Input pcd data is not valid. Input data size is %dbytes, expected %d bytes' % (len(Data), SIZE_PCDDATA))
            self.frameStatus = False
        else:
            frameData = struct.unpack(PCDDATA, DATA)
            self.nx = frameData[0]
            self.ny = frameData[1]
            self.nz = frameData[2]
            self.ni = frameData[3]
            self.ne = frameData[4]
            self.wh = frameData[5]
            self.wv = frameData[6]
            self.wr = frameData[7]
    def printPcdData(self):
        print('(%d, %d, %d), (%d, %d), (%d, %d, %d)' % (self.nx, self.ny, self.nz, self.ni, self.ne, self.wh, self.wv, self.wr))

class sLidarData():
    def __init__(self, LIDAR):
        if len(LIDAR) != 104:
            if(DEBUG): print ('Input Lidar data is not valid. Input data size is %dbytes, expected 104 bytes' % (len(LIDAR)))
            self.frameStatus = False
        else:
            frameData = struct.unpack('2I', LIDAR[0:8])
            self.sequencCount = frameData[0]
            self.saveInterval = frameData[1]
            self.INSData = sINSData(LIDAR[8:72])
            self.dataHeader = sLidarMeasureData(LIDAR[72:])
            self.pcdData = []
            self.frameStatus = True

    def getPCDfrombinary(self, DATA):
        unpackData = struct.iter_unpack(PCDDATA, DATA)
        for data in unpackData:
            self.pcdData.append(data)

class cLidarFrame():
    def __init__(self, settings, fileName, path):
        # open file
        dataFile = open(fileName, 'rb')

        # read header and register
        try:
            self.header = sLidarHeader(dataFile.read(SIZE_HEADER_LIDAR+SIZE_HEADER_LIDAR_SENSOR_INFO))    # read 132 bytes for Lidar data header
        except UnicodeDecodeError:
            print('Header parsing failed.')
            
        # read data frame and register
        self.dataFrames = []
        self.fileCount = 0
        cnt = 0
        while True:            
            frameData = dataFile.read(104)
            if frameData == b'':
                break

            frame = sLidarData(frameData)
            if frame.frameStatus is True:
                pcdData = dataFile.read(SIZE_PCDDATA*frame.dataHeader.dwMeasurePcdCounts)
                frame.getPCDfrombinary(pcdData)
                # self.dataFrames.append(frame)
                self.convert(path, frame)

        # file close
        dataFile.close()

    def convertAll(self, path):
        if not os.path.isdir(path):
            os.mkdir(path)
        
        logFile = open(path+'\\log.txt', 'w')
        for lidar in self.dataFrames:
            # Save log
            data =  '%06d.txt %s %d %d %d\n' % (lidar.dataHeader.dwMeasureCounter, lidar.INSData.getTime(), lidar.INSData.nUpdateCount, lidar.sequencCount, lidar.saveInterval)
            logFile.write(data)

            # Save as Lidar data
            filename = '%s\\%06d.txt' % (path, lidar.dataHeader.dwMeasureCounter)
            lidarData = open(filename, 'w')
            for pcd in lidar.pcdData:
                saveData = '%d %d %d %d %d %d %d %d\n' % (pcd[0], pcd[1], pcd[2], pcd[3], pcd[4], pcd[5], pcd[6], pcd[7])
                lidarData.write(saveData)
            lidarData.close()
        logFile.close()

    def convert(self, path, lidar):
        if not os.path.isdir(path):
            os.mkdir(path)
        
        logFile = open(path+'\\log.txt', 'a')
        

        # Save log
        # data =  '%06d.txt %s %d %d %d\n' % (lidar.dataHeader.dwMeasureCounter, lidar.INSData.getTime(), lidar.INSData.nUpdateCount, lidar.sequencCount, lidar.saveInterval)
        data =  '%06d.txt %s %d %d %d\n' % (self.fileCount, lidar.INSData.getTime(), lidar.INSData.nUpdateCount, lidar.sequencCount, lidar.saveInterval)
        logFile.write(data)

        # Save as Lidar data
        filename = '%s\\%06d.txt' % (path, self.fileCount)
        lidarData = open(filename, 'w')
        for pcd in lidar.pcdData:
            saveData = '%d %d %d %d %d %d %d %d\n' % (pcd[0], pcd[1], pcd[2], pcd[3], pcd[4], pcd[5], pcd[6], pcd[7])
            lidarData.write(saveData)
        lidarData.close()
        logFile.close()
        print(self.fileCount)
        self.fileCount = self.fileCount+1

    def printHeader(self):
        self.header.printHeader()
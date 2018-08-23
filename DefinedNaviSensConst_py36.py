import struct
from GPSTranslation import *
import os

DEBUG = True
# DEBUG = False

HEADERTYPE = 'I6BH2f2IiIiIiIiIi'
FRAMEHEADERTYPE = '2I'
INSTYPE = '2i6BH6f3d'
GPSTYPE = '2I6BH3d13fI'
# OBDTYPE = '3I5f6BH'
OBDTYPE = 'I12H7f6BH'
DMITYPE = '2I4l6BH'

HEADERSIZE = struct.calcsize(HEADERTYPE)
INSSIZE = struct.calcsize(INSTYPE)
GPSSIZE = struct.calcsize(GPSTYPE)
OBDSIZE = struct.calcsize(OBDTYPE)
DMISIZE = struct.calcsize(DMITYPE)
FRAMEHEADERSIZE = struct.calcsize(FRAMEHEADERTYPE)
FRAMESIZE = struct.calcsize(FRAMEHEADERTYPE)+INSSIZE+GPSSIZE+OBDSIZE+DMISIZE

class sHeader():
    def __init__(self, header):
        # parsing header
        if len(header) is not HEADERSIZE:
            if(DEBUG): print ('Input header data is not valid. Input data size is %d' % (len(header)))
            self.frameStatus = False
        else:
            headerData = struct.unpack(HEADERTYPE, header)
            self.dataFileVersion = headerData[0]
            self.dataSaveTime = headerData[1:8]
            self.temperature = headerData[8]
            self.humidity = headerData[9]
            self.illuminance = headerData[10]
            self.weather = headerData[11]
            self.frameDataSize = headerData[12]
            self.INSDataVersion = headerData[13]
            self.INSDataSize = headerData[14]
            self.GPSDataVersion = headerData[15]
            self.GPSDataSize = headerData[16]
            self.OBDDataVersion = headerData[17]
            self.OBDDataSize = headerData[18]
            self.DMIDataVersion = headerData[19]
            self.DMIDataSize = headerData[20]
            self.frameStatus = True

        with open('MasterLog.txt', 'w') as headerOut:
            headerOut.write('Data File Version : 0x%08X\n' % (self.dataFileVersion))
            headerOut.write('INS Data Version : 0x%08X\n' % (self.INSDataVersion))
            headerOut.write('INS Data Size : %i\n' % (self.INSDataSize))
            headerOut.write('GPS Data Version : 0x%08X\n' % (self.GPSDataVersion))
            headerOut.write('GPS Data Size : %i\n' % (self.GPSDataSize))
            headerOut.write('OBD Data Version : 0x%08X\n' % (self.OBDDataVersion))
            headerOut.write('OBD Data Size : %i\n' % (self.OBDDataSize))
            headerOut.write('DMI Data Version : 0x%08X\n' % (self.DMIDataVersion))
            headerOut.write('DMI Data Size : %i\n' % (self.DMIDataSize))

    def printHeader(self):
        try:
            print ('Data File Version : 0x%08X' % (self.dataFileVersion))
            print ('INS Data Version : 0x%08X' % (self.INSDataVersion))
            print ('INS Data Size : %i' % (self.INSDataSize))
            print ('GPS Data Version : 0x%08X' % (self.GPSDataVersion))
            print ('GPS Data Size : %i' % (self.GPSDataSize))
            print ('OBD Data Version : 0x%08X' % (self.OBDDataVersion))
            print ('OBD Data Size : %i' % (self.OBDDataSize))
            print ('DMI Data Version : 0x%08X' % (self.DMIDataVersion))
            print ('DMI Data Size : %i' % (self.DMIDataSize))
        except AttributeError:
            print ('Header is not exist.')
        

class sINSData():
    def __init__(self, INS):
        # parsing INS data
        if len(INS) is not INSSIZE:
            if(DEBUG): print ('Input INS data is not valid. Input data size is %d bytes, expected %d bytes.' % (len(INS), INSSIZE))
            self.frameStatus = False
        else:
            INSData = struct.unpack(INSTYPE, INS)
            self.dwDevStatus = INSData[0]
            self.nUpdateCount = INSData[1]
            self.qwGpsTime = INSData[2:9]
            self.afVelocity = INSData[9:12]
            self.afOrient = INSData[12:15]
            self.adPosition = INSData[15:18]
            self.frameStatus = True
            
    def printTime(self):
        try:
            print ('Data Save Time : 20%02d-%02d-%02d - %02dh%02dm%02d.%03ds' % (self.qwGpsTime[0], self.qwGpsTime[1], self.qwGpsTime[2], self.qwGpsTime[3]+9, self.qwGpsTime[4], self.qwGpsTime[5], self.qwGpsTime[6]))
        except AttributeError:
            print ('INS Time is not valid.')

    def getTimeAsString(self, t=0):
        try:
            return '%02d%02d%02d-%02dh%02dm%02d.%03ds' % (self.qwGpsTime[0], self.qwGpsTime[1], self.qwGpsTime[2], self.qwGpsTime[3]+t, self.qwGpsTime[4], self.qwGpsTime[5], self.qwGpsTime[6])
        except AttributeError:
            return 'INS Time is not valid.'
            
    def getTime(self, t=0):
        try:
            return '%02d%02d%02d%03d' % (self.qwGpsTime[3]+t, self.qwGpsTime[4], self.qwGpsTime[5], self.qwGpsTime[6])
        except AttributeError:
            return 'GPS Time is not valid.'

    def getTimeAsList(self):
        return self.qwGpsTime

class sGPSData():
    def __init__(self, GPS):
        # parsing GPS data
        if len(GPS) is not GPSSIZE:
            if(DEBUG): print ('Input GPS data is not valid. Input data size is %d' % (len(GPS)))
            self.frameStatus = False
        else:
            GPSData = struct.unpack(GPSTYPE, GPS)
            self.dwDevStatus = GPSData[0]
            self.nUpdateCount = GPSData[1]
            self.qwGpsTime = GPSData[2:9]
            self.adRawGpsPos = GPSData[9:12]
            self.afRawGpsPosAcc = GPSData[12:15]
            self.fHDOP = GPSData[15]
            self.fUndulation = GPSData[16]
            self.afGpsVel = GPSData[17:20]
            self.afGpsVelAcc = GPSData[20:23]
            self.fCourse = GPSData[23]
            self.fCourseAcc = GPSData[24]
            self.dwReserved = GPSData[25]
            self.frameStatus = True

            # reserved data
            self.sequenceCount = 0
            self.saveInterval = 0
    def getTime(self, t=0):
        try:
            # return '%02d%02d%02d-%02dh%02dm%02d.%03ds' % (self.qwGpsTime[0], self.qwGpsTime[1], self.qwGpsTime[2], self.qwGpsTime[3]+t, self.qwGpsTime[4], self.qwGpsTime[5], self.qwGpsTime[6])
            return '%02d%02d%02d%02d%02d%02d.%03d' % (self.qwGpsTime[0], self.qwGpsTime[1], self.qwGpsTime[2], self.qwGpsTime[3]+t, self.qwGpsTime[4], self.qwGpsTime[5], self.qwGpsTime[6])
        except AttributeError:
            return 'INS Time is not valid.'

class sOBDData():
    def __init__(self, OBD):
        # parsing OBD data
        if len(OBD) is not OBDSIZE:
            if(DEBUG): print ('Input OBD data is not valid. Input data size is %d' % (len(OBD)))
            self.frameStatus = False
        else:
            # version 1
            # OBDData = struct.unpack(OBDTYPE, OBD)
            # self.nUpdateCount = OBDData[0]
            # self.GearSelection = OBDData[1]
            # self.AliveCounter = OBDData[2]
            # self.YawRate = OBDData[3]
            # self.WheelSpeedFL = OBDData[4]
            # self.WheelSpeedFR = OBDData[5]
            # self.WheelSpeedRL = OBDData[6]
            # self.WheelSpeedRR = OBDData[7]
            # self.qwGpsTime = OBDData[8:15]
            # self.frameStatus = True

            # version 2
            OBDData = struct.unpack(OBDTYPE, OBD)
            self.nUpdateCount = OBDData[0]
            self.ClusterMessage = OBDData[1]
            self.GearSelection = OBDData[2]
            self.AliveCounter = OBDData[3]
            self.Reserved = OBDData[4]
            self.EngineRPM = OBDData[5]
            self.AccelPedalPosition = OBDData[6]
            self.ThrottleRate = OBDData[7]
            self.BrakeIndicator = OBDData[8]
            self.BrakePressure = OBDData[9]
            self.SteeringRate = OBDData[10]
            self.SteeringAngle = OBDData[11]
            self.VehicleSpeed = OBDData[12]
            self.AccelRate = OBDData[13:16]
            self.WheelSpeed = OBDData[16:20]
            self.qwGpsTime = OBDData[20:27]
            self.frameStatus = True

class sDMIData():
    def __init__(self, DMI):
        # parsing DMI data
        if len(DMI) is not DMISIZE:
            if(DEBUG): print ('Input DMI data is not valid. Input data size is %d' % (len(DMI)))
            self.frameStatus = False
        else:
            DMIData = struct.unpack(DMITYPE, DMI)
            self.nUpdateCount = DMIData[0]
            self.Reserved = DMIData[1]
            self.FrontWheelCountLeft = DMIData[2]
            self.FrontWheelCountRight = DMIData[3]
            self.RearWheelCountLeft = DMIData[4]
            self.RearWheelCountRight = DMIData[5]
            self.qwGpsTime = DMIData[6:13]
            self.frameStatus = True

class sDataFrame():
    def __init__(self, data):
        # read data frame
        if len(data) != FRAMESIZE:
            if(DEBUG): print ('Input Frame data is not valid. Input data size is %d bytes, expected %d bytes' % (len(data), FRAMESIZE))
            self.frameStatus = False
        else:
            dataFrameHeader = struct.unpack(FRAMEHEADERTYPE, data[0:FRAMEHEADERSIZE])
            self.sequenceCount = dataFrameHeader[0]
            self.saveInterval = dataFrameHeader[1]
            self.INSData = sINSData(data[FRAMEHEADERSIZE:FRAMEHEADERSIZE+INSSIZE])                                                      # read 64 bytes for INS data
            self.GPSData = sGPSData(data[FRAMEHEADERSIZE+INSSIZE:FRAMEHEADERSIZE+INSSIZE+GPSSIZE])                                      # read 96 bytes for GPS data
            # self.OBDData = sOBDData(data[FRAMEHEADERSIZE+INSSIZE+GPSSIZE:FRAMEHEADERSIZE+INSSIZE+GPSSIZE+OBDSIZE])                      # read 40 bytes for OBD data
            self.OBDData = sOBDData(data[FRAMEHEADERSIZE+INSSIZE+GPSSIZE:FRAMEHEADERSIZE+INSSIZE+GPSSIZE+OBDSIZE])                      # read 64 bytes for OBD data
            self.DMIData = sDMIData(data[FRAMEHEADERSIZE+INSSIZE+GPSSIZE+OBDSIZE:FRAMEHEADERSIZE+INSSIZE+GPSSIZE+OBDSIZE+DMISIZE])      # read 32 bytes for DMI data
            self.frameStatus = True

class sObjDataFrame():
    def __init__(self, frameIndex, dataType, dataSize, objCount, timestamp, objects):
        self.frameIndex = frameIndex
        self.dataType = dataType
        self.dataSize = dataSize
        self.objCount = objCount
        self.timestamp = timestamp
        self.objects = objects

        def getTimeAsString(self, t=0):
            try:
                return '%02d%02d%02d-%02dh%02dm%02d.%03ds' % (self.timestamp[0], self.timestamp[1], self.timestamp[2], self.timestamp[3]+t, self.timestamp[4], self.timestamp[5], self.timestamp[6])
            except AttributeError:
                return 'GPS Time is not valid.'
            
    def getTime(self, t=0):
        try:
            return '%02d%02d%02d%03d' % (self.timestamp[3]+t, self.timestamp[4], self.timestamp[5], self.timestamp[6])
        except AttributeError:
            return 'GPS Time is not valid.'

class cMasterDB():
    def __init__(self, settings, fileName):
        self.settings = settings
        # open file
        dataFile = open(fileName, 'rb')

        # read header and register
        self.header = sHeader(dataFile.read(struct.calcsize(HEADERTYPE)))    # read 64 bytes for header

        # read data frame and register
        self.dataFrames = []
        while True:
            frameData = dataFile.read(FRAMESIZE)
            if frameData == b'':
                break

            frame = sDataFrame(frameData)
            if frame.frameStatus is True:
                self.dataFrames.append(frame)        

        # file close
        dataFile.close()

        self.count = 0
        self.EOF = False

    def printHeader(self):
        self.header.printHeader()

    def getAllFrames(self):
        return self.dataFrames
    
    def getNextFrame(self):
        if self.EOF:
            return False
        else:
            self.count = self.count+1
            if self.count == len(self.dataFrames):
                self.EOF = True
            return self.dataFrames[self.count-1]

    def convert2Text(self, path, filename):
        count = 1
        if not os.path.isdir(path):
            os.mkdir(path)
        

        logFile = open('%s\\%s' % (path, filename), 'w')
        for frame in self.dataFrames:
            # tmx, tmy = frame.GPSData.adRawGpsPos[0], frame.GPSData.adRawGpsPos[1]
            # writeData = '%04d %03d %03d %s %.6f %.6f %.6f %.6f %.6f %.6f\n' % (count, frame.sequenceCount, frame.saveInterval, frame.INSData.getTime(9), frame.INSData.afOrient[0], frame.INSData.afOrient[1], frame.INSData.afOrient[2], tmx, tmy, frame.GPSData.fHDOP)

            # tmx, tmy = GRS80toTM(frame.GPSData.adRawGpsPos[0], frame.GPSData.adRawGpsPos[1],)
            # # tmx, tmy = frame.GPSData.adRawGpsPos[0], frame.GPSData.adRawGpsPos[1]
            # writeData = '%04d %s %.6f %.6f %.6f %.6f' % (count, frame.INSData.getTime(9), tmx, tmy, frame.INSData.afOrient[2], frame.GPSData.fHDOP)
            
            # # total count / sequence count / save interval
            # writeData = '%05d %03d %03d ' % (count, frame.sequenceCount, frame.saveInterval)
            writeData = ''
            if self.settings['SETTING_INS']:
                # INS: dwDevstatus / nUpdateCount / qwGpsTime / afVelocity[3] / afOrient[3] / adPosition[3]
                writeData = writeData + '%d %05d %s %.6f %.6f %.6f %.6f %.6f %.6f %.6f %.6f %.6f ' % (frame.INSData.dwDevStatus, frame.INSData.nUpdateCount, frame.INSData.getTime(), frame.INSData.afVelocity[0], frame.INSData.afVelocity[1], frame.INSData.afVelocity[2], frame.INSData.afOrient[0], frame.INSData.afOrient[1], frame.INSData.afOrient[2], frame.INSData.adPosition[0], frame.INSData.adPosition[1], frame.INSData.adPosition[2])
            
            if self.settings['SETTING_GPS']:
                # GPS: dwDevStatus / nUpdateCount / qwGpsTime / adRawGpsPos[3] / afRawGpsPosAcc[3] / fHDOP / fUndulation / afGpsVel[3] / afGpsVelAcc[3] / fCourse / fCourseAcc
                writeData = writeData + '%d %d %s %.6f %.6f %.6f %.6f %.6f %.6f %.6f %.6f %.6f %.6f %.6f %.6f %.6f %.6f %.6f %.6f ' % (frame.GPSData.dwDevStatus, frame.GPSData.nUpdateCount, frame.GPSData.getTime(), frame.GPSData.adRawGpsPos[0], frame.GPSData.adRawGpsPos[1], frame.GPSData.adRawGpsPos[2], frame.GPSData.afRawGpsPosAcc[0], frame.GPSData.afRawGpsPosAcc[1], frame.GPSData.afRawGpsPosAcc[2], frame.GPSData.fHDOP, frame.GPSData.fUndulation, frame.GPSData.afGpsVel[0], frame.GPSData.afGpsVel[1], frame.GPSData.afGpsVel[2], frame.GPSData.afGpsVelAcc[0], frame.GPSData.afGpsVelAcc[1], frame.GPSData.afGpsVelAcc[2], frame.GPSData.fCourse, frame.GPSData.fCourseAcc)
                # writeData = writeData + '%.6f ' % (frame.GPSData.fHDOP)

            if self.settings['SETTING_OBD']:
                # version 1
                # OBD: nUpdateCount / Gear Selection / Alive Counter / Steering Angle / Wheel Speed FL / Wheel Speed FR / Wheel Speed RL / Wheel Speed RR
                # writeData = writeData + '%d %d %d %.6f %.6f %.6f %.6f %.6f ' % (frame.OBDData.nUpdateCount, frame.OBDData.GearSelection, frame.OBDData.AliveCounter, frame.OBDData.YawRate, frame.OBDData.WheelSpeedFL, frame.OBDData.WheelSpeedFR, frame.OBDData.WheelSpeedRL, frame.OBDData.WheelSpeedRR)

                # version 2
                writeData = writeData + '%d %u %u %u %u %u %u %u %u %u %u %u %.6f %.6f %.6f %.6f %.6f %.6f %.6f ' % (frame.OBDData.nUpdateCount, frame.OBDData.ClusterMessage, frame.OBDData.GearSelection, frame.OBDData.AliveCounter, frame.OBDData.EngineRPM, frame.OBDData.AccelPedalPosition, frame.OBDData.ThrottleRate, frame.OBDData.BrakeIndicator, frame.OBDData.BrakePressure, frame.OBDData.SteeringRate, frame.OBDData.SteeringAngle, frame.OBDData.VehicleSpeed, frame.OBDData.AccelRate[0], frame.OBDData.AccelRate[1], frame.OBDData.AccelRate[2], frame.OBDData.WheelSpeed[0], frame.OBDData.WheelSpeed[1], frame.OBDData.WheelSpeed[2], frame.OBDData.WheelSpeed[3])
               
            if self.settings['SETTING_DMI']:
                # DMI: nUpdateCount / Front Wheel Count Left / Front Wheel Count Right / Rear Wheel Count Left / Rear Wheel Count Right
                writeData = writeData + '%d %.6f %.6f %.6f %.6f ' % (frame.DMIData.nUpdateCount, frame.DMIData.FrontWheelCountLeft, frame.DMIData.FrontWheelCountRight, frame.DMIData.RearWheelCountLeft, frame.DMIData.RearWheelCountRight)

            writeData = writeData + '\n'
            # tmx, tmy = frame.GPSData.adRawGpsPos[0], frame.GPSData.adRawGpsPos[1]
            # writeData = '%s %.6f %.6f\n' % (frame.INSData.getTime(9), tmx, tmy)
            
            logFile.write(writeData)
            count = count + 1
        logFile.close()

    def findByTime(self, t):
        numberOfFrame = len(self.dataFrames)
        timeGap = 100000000
        matched_index = 0
        for i in range(numberOfFrame):
            masterTime = self.dataFrames[i].INSData.qwGpsTime
            masterTimestamp = masterTime[3]*60*60*1000 + masterTime[4]*60*1000 + masterTime[5]*1000 + masterTime[6]

            dataTimestamp = t[3]*60*60*1000 + t[4]*60*1000 + t[5]*1000 + t[6]

            if abs(masterTimestamp - dataTimestamp) < timeGap:
                matched_index = i
                timeGap = abs(masterTimestamp - dataTimestamp)
        return self.dataFrames[matched_index]
        

class cReferenceDB():
    def __init__(self, fileName):
        # open file
        dataFile = open(fileName, 'rb')

        # read header and register
        self.header = sHeader(dataFile.read(struct.calcsize(HEADERTYPE)))    # read 64 bytes for header

        # read data frame and register
        self.dataFrames = []
        # for frameData in iter(lambda: dataFile.read(FRAMEHEADERSIZE+GPSSIZE), ''):
        
        while True:
            frameData = dataFile.read(FRAMEHEADERSIZE+GPSSIZE)
            if frameData == b'':
                break
            dataFrameHeader = struct.unpack(FRAMEHEADERTYPE, frameData[0:FRAMEHEADERSIZE])
            
            frame = sGPSData(frameData[FRAMEHEADERSIZE:])
            if frame.frameStatus is True:
                frame.sequenceCount = dataFrameHeader[0]
                frame.saveInterval = dataFrameHeader[1]
                self.dataFrames.append(frame)        

        # file close
        dataFile.close()

    def printHeader(self):
        self.header.printHeader()
    
    def convert(self, path):
        count = 1
        if not os.path.isdir(path):
            os.mkdir(path)
        
        logFile = open(path+'\\ReferenceDB.txt', 'w')
        for frame in self.dataFrames:
            tmx, tmy = GRS80toTM(frame.adRawGpsPos[0], frame.adRawGpsPos[1],)
            writeData = '%04d %03d %03d %s %.6f %.6f %.6f\n' % (count, frame.sequenceCount, frame.saveInterval, frame.getTime(0), tmx, tmy, frame.fHDOP)
            count = count + 1
            logFile.write(writeData)
        logFile.close()

    def findByTime(self, t):
        numberOfFrame = len(self.dataFrames)
        timeGap = 100000000
        matched_index = 0
        for i in range(numberOfFrame):
            masterTime = self.dataFrames[i].qwGpsTime
            masterTimestamp = (masterTime[3]-9)*60*60*1000 + masterTime[4]*60*1000 + masterTime[5]*1000 + masterTime[6]

            dataTimestamp = t[3]*60*60*1000 + t[4]*60*1000 + t[5]*1000 + t[6]
            
            if abs(masterTimestamp - dataTimestamp) < timeGap:
                matched_index = i
                timeGap = abs(masterTimestamp - dataTimestamp)
        return self.dataFrames[matched_index]

class cObjectDB():
    def __init__(self, settings, fileName):
        self.settings = settings

        # open file
        dataFile = open(fileName, 'rb')
        print ('open file: %s' % (fileName))

        # read header and register
        header = dataFile.read(struct.calcsize('I6BHI'))
        header = struct.unpack('I6BHI', header)
        
        self.dataSaveVersion = header[0]   # read 16 bytes for header
        self.dataSaveTime = header[1:8]
        self.totalSavedDataCount = header[8]
        
        # read data frame and register
        self.dataFrames = []
        # for frameData in iter(lambda: dataFile.read(FRAMEHEADERSIZE+GPSSIZE), ''):
        
        for i in range(self.totalSavedDataCount):
            frameHeader = dataFile.read(struct.calcsize('4H6BH'))
            if frameHeader == b'':
                break
            frameHeader = struct.unpack('4H6BH', frameHeader)
            
            frameIndex = frameHeader[0]
            dataType = frameHeader[1]
            dataSize = frameHeader[2]
            objCount = frameHeader[3]
            timestamp = frameHeader[4:11]
            objects = []

            for j in range(objCount):                
                # objData = dataFile.read(struct.calcsize('30f'))
                # objData = struct.unpack('30f', objData)
                objData = dataFile.read(struct.calcsize('16f'))
                objData = struct.unpack('16f', objData)
                objects.append(objData)
            
            self.dataFrames.append(sObjDataFrame(frameIndex, dataType, dataSize, objCount, timestamp, objects))

        # file close
        dataFile.close()

    def printHeader(self):
        print('Data save version : 0x%08X' % (self.dataSaveVersion))
        print('Data Save Time : %s' % (self.getTime(9)))
        print('Total Saved Data Count : %d' %(self.totalSavedDataCount))
    
    def convert(self, path):
        count = 1
        if not os.path.isdir(path):
            os.mkdir(path)
        
        logFile = open(path+'\\ObjectDB.txt', 'w')
        for frame in self.dataFrames:
            # frame index | sequence number | object id | class id | cx | cy | cz | w | l | h | orientation |
            seqNum = 1
            for i in range(30):
                try:
                    o = frame.objects[i]                
                    writeData = '%04d %d %d %d %.6f %.6f %.6f %.6f %.6f %.6f %.6f\n' % (frame.frameIndex, seqNum, o[1], o[9], o[2], o[3], o[4], o[5], o[6], o[7], o[8])

                except IndexError as e1:
                    writeData = '%04d %d 0 0 0 0 0 0 0 0\n' % (frame.frameIndex, seqNum)
                
                seqNum +=1
                logFile.write(writeData)                

                # print (writeData)
        
                # print(writeData)
                # count = count + 1

                # frame index | object id | cx | cy | cz | orientation |          
                # distance = sqrt(o[2]*o[2] + o[3]*o[3])
                # writeData = '%04d %.6f %.6f %.6f %.6f\n' % (frame.frameIndex, o[2], o[3], distance, o[8])
        logFile.close()
                    
        
    def getTimeAsString(self, t=0):
        try:
            return '%02d%02d%02d-%02dh%02dm%02d.%03ds' % (self.dataSaveTime[0], self.dataSaveTime[1], self.dataSaveTime[2], self.dataSaveTime[3]+t, self.dataSaveTime[4], self.dataSaveTime[5], self.dataSaveTime[6])
        except AttributeError:
            return 'GPS Time is not valid.'
            
    def getTime(self, t=0):
        try:
            return '%02d%02d%02d%03d' % (self.dataSaveTime[3]+t, self.dataSaveTime[4], self.dataSaveTime[5], self.dataSaveTime[6])
        except AttributeError:
            return 'GPS Time is not valid.'

# obj = cObjectDB('201703M08D15H32m23s\\Object_00_201703M08D15H32m23s.dat')
# obj.convert('C:\\ConvertedData')

# masterDB = cMasterDB('NsuMDB_07_201703M08D15H32m23s.dat')
# masterDB.convert('C:\\ConvertedData')
# obj = cObjectDB('Object_03M23D10H48m58s.dat')
# obj.convert('C:\\ConvertedData')

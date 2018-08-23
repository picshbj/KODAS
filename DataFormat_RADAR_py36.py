import struct
import cv2
import numpy as np
import os
from DefinedNaviSensConst_py36 import *

DEBUG = True
DEBUG = False

HEADER_RADAR = 'I6BIiI32s32s'
HEADER_RADAR_SENSOR_INFO = '28s'
HEADER_RADARMEASUREDATA = 'IQ2H2BHh2H2f'
RADARTRACKMSG = '8B8f'

SIZE_HEADER_RADAR = struct.calcsize(HEADER_RADAR)
SIZE_HEADER_RADAR_SENSOR_INFO = struct.calcsize(HEADER_RADAR_SENSOR_INFO)
SIZE_HEADER_RADARMEASUREDATA = struct.calcsize(HEADER_RADARMEASUREDATA)
SIZE_RADARTRACKMSG = struct.calcsize(RADARTRACKMSG)


class sRadarSensorInfoHeader():
	def __init__(self, HEADER):
		self.RadarSensorInfoHeader = struct.unpack(HEADER_RADAR_SENSOR_INFO, HEADER)
		# parsing RADAR sensor info header
	def printHeader(self):
		print ('Radar Sensor Info Header : %s' % (self.RadarSensorInfoHeader))

class sRadarHeader():
	def __init__(self, RADARHEAD):
		# parsing RADAR Header
		if len(RADARHEAD) != SIZE_HEADER_RADAR+SIZE_HEADER_RADAR_SENSOR_INFO:
			if(DEBUG): print ('Input Radar header data is not valid. Input data size is %d' % (len(RADARHEAD)))
			self.frameStatus = False
		else:
			headerData = struct.unpack(HEADER_RADAR, RADARHEAD[:SIZE_HEADER_RADAR])
			self.dataSaveVersion = headerData[0]
			self.dataSaveTime = headerData[0:7]
			self.INSInfoVersion = headerData[7]
			self.INSInfoSize = headerData[8]
			self.RadarID = headerData[9]
			self.RadarModelName = headerData[10].decode('utf-8')
			self.RadarSerialNo = headerData[11].decode('utf-8')
			self.RadarSensorInfoHeader = sRadarSensorInfoHeader(RADARHEAD[SIZE_HEADER_RADAR:])
			self.frameStatus = True

	def printHeader(self):
		try:
			print ('Data Save Version : 0x%08X' % (self.dataSaveVersion))
			print ('Data Save Time : 20%02d-%02d-%02d - %02dh%02dm%02d.%ds' % (self.dataSaveTime[0], self.dataSaveTime[1], self.dataSaveTime[2], self.dataSaveTime[3]+9, self.dataSaveTime[4], self.dataSaveTime[5], self.dataSaveTime[6]))
			print ('INS Info Version : 0x%08X' % (self.INSInfoVersion))
			print ('INS Info Size : %i' % (self.INSInfoSize))
			print ('Radar ID : %i' % (self.RadarID))
			print ('Radar Model Name : %s' % (self.RadarModelName))
			print ('Radar Serial Number : %s' % (self.RadarSerialNo))
			self.RadarSensorInfoHeader.printHeader()

		except AttributeError:
			print ('Header is not exist.')

class sRadarMeasureData():
	def __init__(self, HEAD):
		# parsing BITMAPINFOHEADER
		if len(HEAD) is not SIZE_HEADER_RADARMEASUREDATA:
			if(DEBUG): print ('Input measuredata header data is not valid. Input data size is %d' % (len(HEAD)))
			self.frameStatus = False
		else:
			headerData = struct.unpack(HEADER_RADARMEASUREDATA, HEAD)
			self.dwUpdateCount = headerData[0]
			self.qwTimeStamp = headerData[1]
			self.wScanIndex = headerData[2]
			self.wTrackNo = headerData[3]
			#sRADAR_STATUS_MSG
			self.bRollingCount1 = headerData[4]
			self.bCommError = headerData[5]
			self.wDspTimeStamp = headerData[6]
			self.wRadiusCurvature = headerData[7]
			self.wScanIndex = headerData[8]
			self.nSwVersionDsp = headerData[9]
			self.fYawRateCalc = headerData[10]
			self.fVehicleSpeedCalc = headerData[11]
			self.frameStatus = True
	def printHeader(self):
		try:
			print ('TimeStamp : %i' % (self.qwTimeStamp))
			print ('wScanIndex : %d' % (self.wScanIndex))
			print ('wTrackNo : %d' % (self.wTrackNo))
			print ('wScanIndex : %i' % (self.wScanIndex))

		except AttributeError:
			print ('Header is not exist.')

class sRADARTRACKMSG():
	def __init__(self, DATA):
		if len(DATA) != SIZE_RADARTRACKMSG:
			if(DEBUG): print ('Input Radar Track data is not valid. Input data size is %dbytes, expected %d bytes' % (len(Data), SIZE_RADARTRACKMSG))
			self.frameStatus = False
		else:
			frameData = struct.unpack(RADARTRACKMSG, DATA)
			self.bID = frameData[0]
			self.bStatus = frameData[1]
			self.bGroupingChanged = frameData[2]
			self.bOnComming = frameData[3]
			self.bMedRangeMode = frameData[4]
			self.bBridgeObject = frameData[5]
			self.bRollingCount = frameData[6]
			self.bReserved = frameData[7]
			self.fLateRate = frameData[8]
			self.fValidLevel = frameData[9]
			self.fAmplitude = frameData[10]
			self.fAngle = frameData[11]
			self.fRange = frameData[12]
			self.fWidth = frameData[13]
			self.fRangeAccel = frameData[14]
			self.fRangeRate = frameData[15]
			self.frameStatus = True

	def printRADARTRACKMSG(self):
		print('ID' % (self.bID))
		print('bStatus' % (self.bStatus))
		print('fRange' % (self.fRange))
		print('fWidth' % (self.fWidth))
		print('fRangeAccel' % (self.fRangeAccel))
		print('fRangeRate' % (self.fRangeRate))

class sRadarData():
	def __init__(self, RADAR):
		if len(RADAR) != 112:
			if(DEBUG): print ('Input Radar data is not valid. Input data size is %dbytes, expected 110 bytes' % (len(RADAR)))
			self.frameStatus = False
		else:
			frameData = struct.unpack('2I', RADAR[0:8])
			self.SOF = frameData[0]
			self.saveInterval = frameData[1]
			self.INSData = sINSData(RADAR[8:72])
			self.dataHeader = sRadarMeasureData(RADAR[72:])
			self.RADARTRACKMSG = []
			self.frameStatus = True

	def getMeasurementDatafrombinary(self, DATA):
		unpackData = struct.iter_unpack(RADARTRACKMSG, DATA)
		for data in unpackData:
			self.RADARTRACKMSG.append(data)

class cRadarFrame():
	def __init__(self, settings, fileName, path):
		# open file
		dataFile = open(fileName, 'rb')
		# read header and register
		try:
			self.header = sRadarHeader(dataFile.read(SIZE_HEADER_RADAR+SIZE_HEADER_RADAR_SENSOR_INFO))    # read 116 bytes for Radar data header
		except UnicodeDecodeError:
			print('Header parsing failed.')

		# read data frame and register
		self.fileCount = 0
		cnt = 0
		while True:
			frameData = dataFile.read(112)
			frame = sRadarData(frameData)
			if frame.frameStatus is True:
				RADARTRACKMSG = dataFile.read(SIZE_RADARTRACKMSG*64)
				if RADARTRACKMSG == b'':
					break
				frame.getMeasurementDatafrombinary(RADARTRACKMSG)
				# self.convert(path, frame)

		# file close
		dataFile.close()

	def convert(self, path, Radar):
		if not os.path.isdir(path):
			os.mkdir(path)

		logFile = open(path+'\\log.txt', 'a')


		# Save log
		# data =  '%06d.txt %s %d %d %d\n' % (Radar.dataHeader.dwMeasureCounter, Radar.INSData.getTime(), Radar.INSData.nUpdateCount, Radar.sequencCount, Radar.saveInterval)
		data =  '%06d.txt %s %d %d %d\n' % (self.fileCount, Radar.INSData.getTime(), Radar.INSData.nUpdateCount, Radar.SOF, Radar.saveInterval)
		logFile.write(data)

		# Save as Radar data
		filename = '%s\\%06d.txt' % (path, self.fileCount)
		RadarData = open(filename, 'w')
		for measurement in Radar.RADARTRACKMSG:
			saveData = '%d %d %f %f %f %f %f %f\n' % (measurement.bID, measurement.bStatus, measurement.fAmplitude, measurement.fAngle, measurement.fRange, measurement.fWidth, measurement.fRangeAccel, measurement.fRangeRate)
			RadarData.write(saveData)
		RadarData.close()
		logFile.close()
		print(self.fileCount)
		self.fileCount = self.fileCount+1

	def printHeader(self):
		self.header.printHeader()
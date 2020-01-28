# Written by Jeffrey Hong <hbj723@kitech.re.kr>
# last revision: 2019.07.31
# 
# Modified by Jeffrey Hong on 2019.07.31 [Bug fix] 
# 1. [Variable Definition] HEADER_RADAR: 'I6BIiI32s32s' -> 'I6BHIiI32s32s'
# 2. [Function           ] SRadarHeader.__init__ : Header initialization
# 3. [Function           ] CRadarFrame.getNextFrame : EOF Detection
# 4. [Function           ] CRadarFrame.convertAll : Bug Fix

# Note: KODAS API for RADAR Data
#		This API support currently version 1 only.

import struct
import cv2
import numpy as np
import os
from DefinedNaviSensConst_py36 import *

DEBUG = True
DEBUG = False

HEADER_RADAR = 'I6BHIiI32s32s'
HEADER_RADAR_SENSOR_INFO = '28s'
HEADER_RADARMEASUREDATA = 'IQ2H2BHh2H2f'
RADARTRACKMSG = '8B8f'

SIZE_HEADER_RADAR = struct.calcsize(HEADER_RADAR)
SIZE_HEADER_RADAR_SENSOR_INFO = struct.calcsize(HEADER_RADAR_SENSOR_INFO)
SIZE_HEADER_RADARMEASUREDATA = struct.calcsize(HEADER_RADARMEASUREDATA)
SIZE_RADARTRACKMSG = struct.calcsize(RADARTRACKMSG)


class SRadarSensorInfoHeader():
	def __init__(self, HEADER):
		if len(HEADER) != SIZE_HEADER_RADAR_SENSOR_INFO:
			if(DEBUG): print ('Input HEADER_RADAR_SENSOR_INFO data is not valid. Input data size is %d' % SIZE_HEADER_RADAR_SENSOR_INFO)
		else:
			self.RadarSensorInfoHeader = struct.unpack(HEADER_RADAR_SENSOR_INFO, HEADER)
			# parsing RADAR sensor info header
	def printHeader(self):
		print ('Radar Sensor Info Header : %s' % (self.RadarSensorInfoHeader))

class SRadarHeader():
	def __init__(self, RADARHEAD):
		# parsing RADAR Header
		if len(RADARHEAD) != SIZE_HEADER_RADAR+SIZE_HEADER_RADAR_SENSOR_INFO:
			if(DEBUG): print ('Input Radar header data is not valid. Input data size is %d' % (len(RADARHEAD)))
			self.frameStatus = False
		else:
			headerData = struct.unpack(HEADER_RADAR, RADARHEAD[:SIZE_HEADER_RADAR])
			self.dataSaveVersion = headerData[0]
			self.dataSaveTime = headerData[1:8]
			self.INSInfoVersion = headerData[8]
			self.INSInfoSize = headerData[9]
			self.RadarID = headerData[10]
			self.RadarModelName = headerData[11].decode('utf-8')
			self.RadarSerialNo = headerData[12].decode('utf-8')
			self.RadarSensorInfoHeader = SRadarSensorInfoHeader(RADARHEAD[SIZE_HEADER_RADAR:])
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

class SRadarMeasureData():
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
			#SRADAR_STATUS_MSG
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

class SRADARTRACKMSG():
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

class SRadarData():
	def __init__(self, RADAR):
		if len(RADAR) != 112:
			if(DEBUG): print ('Input Radar data is not valid. Input data size is %dbytes, expected 110 bytes' % (len(RADAR)))
			self.frameStatus = False
		else:
			frameData = struct.unpack('2I', RADAR[0:8])
			self.SOF = frameData[0]
			self.saveInterval = frameData[1]
			self.INSData = SINSData(RADAR[8:72])
			self.dataHeader = SRadarMeasureData(RADAR[72:])
			self.RADARTRACKMSG = []
			self.frameStatus = True

	def getMeasurementDatafrombinary(self, DATA):
		unpackData = struct.iter_unpack(RADARTRACKMSG, DATA)
		for data in unpackData:
			self.RADARTRACKMSG.append(data)

class CRadarFrame():
	def __init__(self, fileName, path):
		# open file
		self.dataFile = open(fileName, 'rb')

		# read header and register
		try:
			self.header = SRadarHeader(self.dataFile.read(SIZE_HEADER_RADAR+SIZE_HEADER_RADAR_SENSOR_INFO))    # read 116 bytes for Radar data header
		except UnicodeDecodeError:
			print('Header parsing failed.')

		# read data frame and register
		self.fileCount = 0
		self.EOF = False
		self.path = path

		if not os.path.isdir(path):
			os.mkdir(path)

	def getNextFrame(self):
		frameData = self.dataFile.read(112)
		frame = SRadarData(frameData)
		
		if frameData == b'\xff\xff\xff\xff\xff\xff\xff\xff':
			self.EOF = True
			self.dataFile.close()
		else:
			RADARTRACKMSG = self.dataFile.read(SIZE_RADARTRACKMSG*64)
			frame.getMeasurementDatafrombinary(RADARTRACKMSG)

		return frame

	def convertAll(self):
		while not self.EOF:
			frame = self.getNextFrame()
			if frame.frameStatus:
				self.convert(self.path, frame)


	def convert(self, path, Radar):
		logFile = open(path+'\\log.txt', 'a')

		# Save log
		# data =  '%06d.txt %s %d %d %d\n' % (Radar.dataHeader.dwMeasureCounter, Radar.INSData.getTime(), Radar.INSData.nUpdateCount, Radar.sequencCount, Radar.saveInterval)
		data =  '%06d.txt %s %d %d %d\n' % (self.fileCount, Radar.INSData.getTime(), Radar.INSData.nUpdateCount, Radar.SOF, Radar.saveInterval)
		logFile.write(data)

		# Save as Radar data
		filename = '%s\\%06d.txt' % (path, self.fileCount)
		RadarData = open(filename, 'w')
		for measurement in Radar.RADARTRACKMSG:
			# saveData = '%d %d %f %f %f %f %f %f\n' % (measurement.bID, measurement.bStatus, measurement.fAmplitude, measurement.fAngle, measurement.fRange, measurement.fWidth, measurement.fRangeAccel, measurement.fRangeRate)
			saveData = '%d %d %d %d %d %d %d %d %f %f %f %f %f %f %f %f\n' % (measurement[0], measurement[1], measurement[2], measurement[3], measurement[4], measurement[5], measurement[6], measurement[7], measurement[8], measurement[9], measurement[10], measurement[11], measurement[12], measurement[13], measurement[14], measurement[15])
			RadarData.write(saveData)
		RadarData.close()
		logFile.close()
		# print(self.fileCount)
		self.fileCount = self.fileCount+1

	def printHeader(self):
		self.header.printHeader()
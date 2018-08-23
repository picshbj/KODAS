from DefinedNaviSensConst_py36 import *

# Master setting
settings = dict()
settings['SETTING_INS'] = True
settings['SETTING_GPS'] = True
settings['SETTING_OBD'] = True
settings['SETTING_DMI'] = True

if __name__ == '__main__':
    # init data
    datfile = 'NsuMDB_01_2018Y08M08D18H18m14s.dat'
    Master = cMasterDB(settings, datfile)

    # get header
    header = Master.header

    # print header
    Master.printHeader()

    # convert raw data to readable data
    Master.convert2Text('output', 'MasterDB_output.txt')
    
    # read frame by frame
    while Master.EOF is False:
        frame = Master.getNextFrame()
        # write your code here..
from DataFormat_RADAR_py36 import *

if __name__ == '__main__':
    filename = 'RadarRR_07_201705M16D13H50m56s.dat'
    output_folder_name = 'Radar'
    
    # init data
    radar = CRadarFrame(filename, output_folder_name)
    
    # get header
    header = radar.header

    # print header
    radar.printHeader()

    # convert raw data to readable data
    # when you call this function, you'll lose current frame pointer.
    # Do not call this function with getNextFrame function at the same time.
    radar.convertAll()

    # free memory
    del radar
    
    #######################################################################

    # init data 
    radar = CRadarFrame(filename, output_folder_name)
    
    # read frame by frame
    while not radar.EOF:
        frame = radar.getNextFrame()
        # write your code here..
    
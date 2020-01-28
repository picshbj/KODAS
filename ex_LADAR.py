from DataFormat_LIDAR_py36 import *

if __name__ == '__main__':
    filename = 'HDL32_01_2018Y08M08D18H18m14s.dat'
    output_folder_name = 'HDL32'
    
    # init data
    lidar = CLidarFrame(filename, output_folder_name)

    # get header
    header = lidar.header

    # print header
    lidar.printHeader()

    # convert raw data to readable data
    # when you call this function, you'll lose current frame pointer.
    # Do not call this function with getNextFrame function at the same time.
    lidar.convertAll()

    # free memory
    del lidar
    
    #######################################################################

    # # init data 
    # lidar = CLidarFrame(filename, output_folder_name)
    
    # # read frame by frame
    # while not lidar.EOF:
    #     frame = lidar.getNextFrame()
    #     # write your code here..
import rospy
from nav_msgs.msg import OccupancyGrid, MapMetaData
import numpy as np
from occupancy_grid_sub import occupancy_grid
from robot_odom_sub import odom_data

if __name__ == '__main__':
    og = occupancy_grid()
    ro = odom_data()
    # print(og.og_np)

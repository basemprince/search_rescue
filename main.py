import rospy
from nav_msgs.msg import OccupancyGrid, MapMetaData
import numpy as np
from occupancy_grid_sub import occupancy_grid

if __name__ == '__main__':
    og = occupancy_grid()
    print(og.og_np)

#!/usr/bin/env python

import rospy
from nav_msgs.msg import OccupancyGrid, MapMetaData
import numpy as np


class occupancy_grid:

    def __init__(self):
        rospy.init_node('map_grid_subscriber', anonymous=False)
        rospy.Subscriber('map', OccupancyGrid , self.callback)
        try:
          rospy.spin()
        except KeyboardInterrupt:
          print("Shutting down")


    # convert from an occupancy grid point to a real world map point
    def og_to_world(map_x,map_y, map):
        pos_x = map.info.origin.position.x + map_x * map.info.resolution
        pos_y = map.info.origin.position.y + map_y * map.info.resolution
        return pos_x, pos_y

    def occupancygrid_to_numpy(self, msg):
        data = np.asarray(msg.data, dtype=np.int8).reshape(msg.info.height, msg.info.width)
        
        # data = np.ma.array(data, mask=data==-1, fill_value=-1)
        return data

    def callback(self, data):
        rospy.loginfo("recieved data")
        # self.og_np is the occupancy gril in a numpy array ->>>
        self.og_np = self.occupancygrid_to_numpy(data)
        # you can use the data from here ###################
        rospy.loginfo(self.og_np)


if __name__ == '__main__':
  og = occupancy_grid()
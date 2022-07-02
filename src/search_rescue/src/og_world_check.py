#!/usr/bin/env python
import rospy
from geometry_msgs.msg import PointStamped
from nav_msgs.msg import OccupancyGrid
import numpy as np

class og_world_check:

    def __init__(self):
        rospy.init_node('og_world_check', anonymous=False)
        
        rospy.Subscriber('map', OccupancyGrid , self.og_callback)
        rospy.Subscriber('clicked_point',PointStamped,self.cp_callback)
        rospy.Subscriber('tb3_0/move_base/global_costmap/costmap', OccupancyGrid, self.cm_callback)
        try:
          rospy.spin()
        except KeyboardInterrupt:
          print("Shutting down")

    def cp_callback(self, data):
        self.cp = data
        column1, row1 = self.world_to_og(self.cost_map)
        column2, row2 = self.world_to_og(self.map)
        print("cost_map row/column:", row1,column1)
        print("occupancy row/column:", row2,column2)
        value_of_cm = self.cm_np[row1,column1]
        value_of_og = self.og_np[row2,column2]
        print("value of occupancy:", value_of_og, "value of cost:", value_of_cm)

    def og_callback(self, data):
        self.map = data
        self.og_np = self.og_to_numpy(data)

    def cm_callback(self, data):
        self.cost_map = data
        self.cm_np = self.og_to_numpy(data)
    
    # convert ocupancy grid to a numpy array
    def og_to_numpy(self,array):
        data = np.asarray(array.data, dtype=np.int8).reshape(array.info.height,
                                                                array.info.width)
        return data

    def world_to_og(self,world):
        column = int(( self.cp.point.x - world.info.origin.position.x )/ world.info.resolution)
        row = int(( self.cp.point.y - world.info.origin.position.y )/ world.info.resolution)
        return column, row



if __name__ == '__main__':
  tg = og_world_check()

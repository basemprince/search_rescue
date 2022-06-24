#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Point
from nav_msgs.msg import OccupancyGrid, MapMetaData, Odometry
import numpy as np

class voronoi_partition:

    def __init__(self):
        self.tb0 = None
        self.tb1 = None
        self.tb2 = None
        self.og_np = None
        self.way_pt0 = Point()
        self.way_pt1 = Point()
        self.way_pt2 = Point()
        rospy.init_node('voronoi_node', anonymous=False)
        rospy.Subscriber('tb3_0/odom', Odometry , self.tb0_callback)
        rospy.Subscriber('tb3_1/odom', Odometry , self.tb1_callback)
        rospy.Subscriber('tb3_2/odom', Odometry , self.tb2_callback)
        rospy.Subscriber('map', OccupancyGrid , self.og_callback)
        self.pub0= rospy.Publisher('/way_point/tb3_0',Point, queue_size=10)
        self.pub1= rospy.Publisher('/way_point/tb3_1',Point, queue_size=10)
        self.pub2= rospy.Publisher('/way_point/tb3_2',Point, queue_size=10)
        try:
          rospy.spin()
        except KeyboardInterrupt:
          print("Shutting down")

    # convert ocupancy grid to real world map coords
    # input: the column and row number in the occupancy grid
    # output: the x and y location on real map
    def og_to_world(og_column,og_row, map):
        pos_x = map.info.origin.position.x + og_column * map.info.resolution
        pos_y = map.info.origin.position.y + og_row * map.info.resolution
        return pos_x, pos_y

    # convert ocupancy grid to a numpy array
    def og_to_numpy(self, msg):
        data = np.asarray(msg.data, dtype=np.int8).reshape(msg.info.height, msg.info.width)
        # data = np.ma.array(data, mask=data==-1, fill_value=-1)
        self.og_np = data
        rospy.loginfo(self.og_np)

    # call back function for occupancy grid
    def og_callback(self, data):
        # rospy.loginfo("recieved data - occupancy grid")
        # the occupancy grid in a numpy array ->>>
        self.og_to_numpy(data)
        self.voroni_compute()
        
    # call back function for robot1 position
    def tb0_callback(self, data):
        # rospy.loginfo("recieved data - tb0")
        self.tb0 = data
        self.voroni_compute()
    
    # call back function for robot2 position
    def tb1_callback(self, data):
        # rospy.loginfo("recieved data - tb1")
        self.tb1 = data
        self.voroni_compute()
        
    # call back function for robot2 position
    def tb2_callback(self, data):
        # rospy.loginfo("recieved data - tb2")
        self.tb2 = data
        self.voroni_compute()
        

    # calculate voronoi partitions using occupancy grid and robot location
    def voroni_compute(self):

        ## position
        # x = self.tb0.pose.pose.position.x
        # y = self.tb0.pose.pose.position.y
        ## orientatoin in quaternion
        # xo = self.tb0.pose.pose.orientation.x
        # yo = self.tb0.pose.pose.orientation.y
        # zo = self.tb0.pose.pose.orientation.z
        # wo = self.tb0.pose.pose.orientation.w


        ##### insert voronoi code here #######


        

        if self.tb0 is not None:
            tb0_position_x = self.tb0.pose.pose.position.x
            print(tb0_position_x)

        # for publishing to robot 0:
        self.way_pt0.x = 0 #### < change to calculation
        self.way_pt0.y = 0 #### < change to calculation
        self.way_pt0.z = 0

        # for publishing to robot 1:
        self.way_pt1.x = 0 #### < change to calculation
        self.way_pt1.y = 0 #### < change to calculation
        self.way_pt1.z = 0

        # for publishing to robot 2:
        self.way_pt2.x = 0 #### < change to calculation
        self.way_pt2.y = 0 #### < change to calculation
        self.way_pt2.z = 0

        self.publish_pts()

    def publish_pts(self):
        self.pub0.publish(self.way_pt0)
        self.pub1.publish(self.way_pt1)
        self.pub2.publish(self.way_pt2)

if __name__ == '__main__':
  voronoi_partition()
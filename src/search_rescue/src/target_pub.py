#!/usr/bin/env python
import rospy
from geometry_msgs.msg import PointStamped
from nav_msgs.msg import OccupancyGrid
import numpy as np

class target_goal:

    def __init__(self):
        rospy.init_node('target_pose', anonymous=False)
        self.radius = 0.5
        self.count = 0

        self.og_np = None
        self.locations_to_check = []
        self.target_pose = PointStamped()
        self.target_pose.header.seq = 1
        self.target_pose.header.stamp = rospy.Time.now()
        self.target_pose.header.frame_id = "map"

        self.target_pose.point.x = 0.99
        self.target_pose.point.y = 2.7
        self.target_pose.point.z = 0.0

        rospy.Subscriber('map', OccupancyGrid , self.og_callback)


        self.target_pose_publisher = rospy.Publisher("/target_pose", PointStamped, queue_size=5)
        try:
          rospy.spin()
        except KeyboardInterrupt:
          print("Shutting down")


    def og_callback(self, data):
        self.map = data
        self.count+=1
        self.og_to_numpy()
        if self.count ==1:
            self.box_len = int(self.radius/data.info.resolution)
            [column,row] = self.world_to_og()
            min_column = column - self.box_len
            min_row = row - self.box_len
            for i in range(self.box_len):
                curr_row = min_row+i
                for k in range(self.box_len):
                    curr_col = min_column + k
                    self.locations_to_check.append([curr_row,curr_col])
           
        self.check_found()

    # convert ocupancy grid to a numpy array
    def og_to_numpy(self):
        data = np.asarray(self.map.data, dtype=np.int8).reshape(self.map.info.height,
                                                                self.map.info.width)
        self.og_np = data

    def world_to_og(self):
        column = int(( self.target_pose.point.x - self.map.info.origin.position.x )/ self.map.info.resolution)
        row = int(( self.target_pose.point.y - self.map.info.origin.position.y )/ self.map.info.resolution)
        return column, row

    def check_found(self):
        for loc in self.locations_to_check:
            row, column = loc
            result = self.og_np[row,column]
            if result != -1:
                print("target found! publishing target pose")
                self.publish_target_pose()

    def publish_target_pose(self):
        self.target_pose_publisher.publish(self.target_pose)
        rospy.sleep(0.3)

if __name__ == '__main__':
  tg = target_goal()

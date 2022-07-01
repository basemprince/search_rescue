import rospy
from geometry_msgs.msg import PointStamped
from nav_msgs.msg import OccupancyGrid, MapMetaData, Odometry
from sensor_msgs.msg import  LaserScan
import numpy as np
import math
import tf

class target_goal:

    def __init__(self):
        rospy.init_node('target_pose', anonymous=False)

        self.tb0_pts = []
        self.tb1_pts = None
        self.tb2_pts = None

        self.target_pose = PointStamped()
        self.target_pose.header.seq = 1
        self.target_pose.header.stamp = rospy.Time.now()
        self.target_pose.header.frame_id = "map"

        self.target_pose.point.x = 0.99
        self.target_pose.point.y = 2.7
        self.target_pose.point.z = 0.0

        rospy.Subscriber("/tb3_0/scan", LaserScan, self.tb0_callback, queue_size=1)
        rospy.Subscriber("/tb3_1/scan", LaserScan, self.tb1_callback, queue_size=1)
        rospy.Subscriber("/tb3_2/scan", LaserScan, self.tb2_callback, queue_size=1)

        self.target_pose_publisher = rospy.Publisher("/target_pose", PointStamped, queue_size=5)
        try:
          rospy.spin()
        except KeyboardInterrupt:
          print("Shutting down")


    # call back function for robot1 position
    def tb0_callback(self, data):
        listener = tf.TransformListener()
        target_frame = "map"
        current_frame = "tb3_0/base_scan"
        listener.waitForTransform(target_frame, current_frame, rospy.Time(), rospy.Duration(4.0))
        self.tb0_pts = []
        for angle, distance in enumerate(data.ranges):
            if distance == math.inf:
                continue
            x = math.cos(angle)/distance
            y = math.sin(angle)/distance
            pt = PointStamped()
            pt.header.frame_id = data.header.frame_id
            pt.point.x = x
            pt.point.y = y
            pt.point.z = 0
            pt_transformed = listener.transformPoint(target_frame,pt)
            self.tb0_pts.append([pt_transformed.point.x,pt_transformed.point.y])
        # print(self.tb0_pts)
    
    # call back function for robot2 position
    def tb1_callback(self, data):
        listener = tf.TransformListener()
        target_frame = "map"
        current_frame = "tb3_1/base_scan"
        listener.waitForTransform(target_frame, current_frame, rospy.Time(), rospy.Duration(4.0))
        self.tb1_pts = []
        for angle, distance in enumerate(data.ranges):
            if distance == math.inf:
                continue
            x = math.cos(angle)/distance
            y = math.sin(angle)/distance
            pt = PointStamped()
            pt.header.frame_id = data.header.frame_id
            pt.point.x = x
            pt.point.y = y
            pt.point.z = 0
            pt_transformed = listener.transformPoint(target_frame,pt)
            self.tb1_pts.append([pt_transformed.point.x,pt_transformed.point.y])
        # print(self.tb1_pts)
        
    # call back function for robot2 position
    def tb2_callback(self, data):
        listener = tf.TransformListener()
        target_frame = "map"
        current_frame = "tb3_2/base_scan"
        listener.waitForTransform(target_frame, current_frame, rospy.Time(), rospy.Duration(4.0))
        self.tb2_pts = []
        for angle, distance in enumerate(data.ranges):
            if distance == math.inf:
                continue
            x = math.cos(angle)/distance
            y = math.sin(angle)/distance
            pt = PointStamped()
            pt.header.frame_id = data.header.frame_id
            pt.point.x = x
            pt.point.y = y
            pt.point.z = 0
            pt_transformed = listener.transformPoint(target_frame,pt)
            self.tb2_pts.append([pt_transformed.point.x,pt_transformed.point.y])
        print(self.tb2_pts)

    def check_found(self):
        pass

    def publish_target_pose(self):
        self.target_pose_publisher.publish(self.target_pose)

if __name__ == '__main__':
  tg = target_goal()

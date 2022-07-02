#!/usr/bin/env python

from re import U
import rospy
from geometry_msgs.msg import PoseStamped, PointStamped
from nav_msgs.msg import OccupancyGrid, MapMetaData, Odometry
from map_msgs.msg import OccupancyGridUpdate
import numpy as np

import matplotlib.pyplot as plt
from scipy.stats import multivariate_normal
from matplotlib.pyplot import cm


class voronoi_partition:

    def __init__(self):
        self.tb0 = None
        self.tb1 = None
        self.tb2 = None
        self.og_np = None
        self.count = 0
        self.freq = 30
        self.cm0 = None
        self.cm1 = None
        self.cm2 = None
        self.target = None
        self.way_pt0 = PoseStamped()
        self.way_pt1 = PoseStamped()
        self.way_pt2 = PoseStamped()

        rospy.init_node('voronoi_node', anonymous=False)

        # robot odom subscriber
        rospy.Subscriber('tb3_0/odom', Odometry, self.tb0_callback)
        rospy.Subscriber('tb3_1/odom', Odometry, self.tb1_callback)
        rospy.Subscriber('tb3_2/odom', Odometry, self.tb2_callback)

        # occupancy grid subscriber
        rospy.Subscriber('map', OccupancyGrid , self.og_callback)

        # cost map subscriber
        rospy.Subscriber('tb3_0/move_base/global_costmap/costmap',
            OccupancyGrid , self.cm0_callback)
        rospy.Subscriber('tb3_1/move_base/global_costmap/costmap',
            OccupancyGrid , self.cm1_callback)
        rospy.Subscriber('tb3_2/move_base/global_costmap/costmap',
            OccupancyGrid , self.cm2_callback)
        
        # target subscriber
        rospy.Subscriber('target_pose', PointStamped, self.target_callback)

        # robot target publisher
        self.pub0= rospy.Publisher("tb3_0/move_base_simple/goal", PoseStamped, queue_size=5)
        self.pub1= rospy.Publisher("tb3_1/move_base_simple/goal", PoseStamped, queue_size=5)
        self.pub2= rospy.Publisher("tb3_2/move_base_simple/goal", PoseStamped, queue_size=5)

        try:
          rospy.spin()
        except KeyboardInterrupt:
          print("Shutting down")

    # call back function for occupancy grid
    def og_callback(self, data):
        # rospy.loginfo("recieved data - occupancy grid")
        # the occupancy grid in a numpy array ->>>
        self.map = data
        self.og_to_numpy()
        if self.target is None:
            if self.count % self.freq == 0:
                self.voronoi_compute()
            self.count += 1
        else:
            self.voronoi_compute()
        
    # call back function for robot1 position
    def tb0_callback(self, data):
        # rospy.loginfo("recieved data - tb0")
        # print(data.pose.pose.position)
        self.tb0 = data
        # self.voronoi_compute()
    
    # call back function for robot2 position
    def tb1_callback(self, data):
        # rospy.loginfo("recieved data - tb1")
        self.tb1 = data
        # self.voronoi_compute()
        
    # call back function for robot2 position
    def tb2_callback(self, data):
        # rospy.loginfo("recieved data - tb2")
        self.tb2 = data
        # self.voronoi_compute()
    
    # call back function for robot1 cost map
    def cm0_callback(self, data):
        # rospy.loginfo("recieved data - cm0")
        self.cm0 = np.asarray(data.data, dtype=np.int8).reshape(data.info.height, data.info.width)
        # print('0-', self.cm0.shape)
    
    # call back function for robot1 cost map
    def cm1_callback(self, data):
        # rospy.loginfo("recieved data - cm1")
        self.cm1 = np.asarray(data.data, dtype=np.int8).reshape(data.info.height, data.info.width)
        # print('1-', self.cm1.shape)
    
    # call back function for robot1 cost map
    def cm2_callback(self, data):
        # rospy.loginfo("recieved data - cm2")
        self.cm2 = np.asarray(data.data, dtype=np.int8).reshape(data.info.height, data.info.width)
        # print('2-', self.cm2.shape)
    
    # call back function for rescue target
    def target_callback(self, data):
        # rospy.loginfo("recieved data - target")
        self.target = data
    
    # convert ocupancy grid to real world map coords
    # input: the column and row number in the occupancy grid
    # output: the x and y location on real map
    def og_to_world(self, value=0):
        rows, columns = np.where(self.og_np==value)
        xs = round(self.map.info.origin.position.x,2) + round(self.map.info.resolution,2) * columns
        ys = round(self.map.info.origin.position.y,2) + round(self.map.info.resolution,2) * rows
        
        return np.column_stack((xs, ys))

    # convert ocupancy grid to a numpy array
    def og_to_numpy(self):
        data = np.asarray(self.map.data, dtype=np.int8).reshape(self.map.info.height,
                                                                self.map.info.width)
        # data = np.ma.array(data, mask=data==-1, fill_value=-1)
        self.og_np = data
        # rospy.loginfo(self.og_np)
    
    # check
    def info_available(self):
        return self.tb0 is not None and self.tb1 is not None and \
            self.tb2 is not None and self.og_np is not None and \
            self.cm0 is not None and self.cm1 is not None and \
            self.cm2 is not None
        
    # density function
    def density_function(self, points, target=None, cov=1):
        if target is not None:
            return multivariate_normal.pdf(points, mean=target, cov=cov)
        else:
            return np.ones(points.shape[0])
    
    # voronoi tessellation
    def voronoi(self, generators, points, target, cov=1, n_iter=1):
        n_generator = generators.shape[0]

        for it in range(n_iter):
            # voronoi tessellation
            vor_indices = [np.argmin([np.inner(generator - point, generator - point) \
                for generator in generators]) for point in points]
            vor_indices = np.array(vor_indices)

            centroids = np.empty(generators.shape)
            if self.target is None:
                weights = self.density_function(points, target, cov)
                for i in range(n_generator):
                    points_i = points[vor_indices==i]
                    weights_i = weights[vor_indices==i].reshape(-1,1)
                    centroids[i] = np.sum(weights_i * points_i, axis=0) / np.sum(weights_i)
            else:
                target = np.array([self.target.point.x, self.target.point.y])
                weights = self.density_function(points, target, cov)
                for i in range(n_generator):
                    points_i = points[vor_indices==i]
                    weights_i = weights[vor_indices==i].reshape(-1,1)
                    centroids[i] = np.sum(weights_i * points_i, axis=0) / np.sum(weights_i)
            
            # update
            generators = np.copy(centroids)
        
        return generators

    # calculate voronoi partitions using occupancy grid and robot location
    def voronoi_compute(self):

        ##### insert voronoi code here #######
        if self.info_available():
            # robot pos
            robots_pos = np.array([
                [self.tb0.pose.pose.position.x, self.tb0.pose.pose.position.y],
                [self.tb1.pose.pose.position.x, self.tb1.pose.pose.position.y],
                [self.tb2.pose.pose.position.x, self.tb2.pose.pose.position.y]
            ])

            # free points
            free_points = self.og_to_world()

            # unknown points
            unknown_points = self.og_to_world(-1)

            # voronoi
            target = unknown_points[np.random.randint(0,len(unknown_points))]
            new_robots_pos = self.voronoi(robots_pos, free_points, target, cov=1)

            # cost
            # new_robots_pos[0] = free_points[np.argmin([np.inner(new_robots_pos[0]-point, new_robots_pos[0]-point) \
            #     for point, grid in zip(free_points, free_points_grids) if self.cm0[grid[1],grid[0]]<99])]
            
            # new_robots_pos[1] = free_points[np.argmin([np.inner(new_robots_pos[1]-point, new_robots_pos[1]-point) \
            #     for point, grid in zip(free_points, free_points_grids) if self.cm1[grid[1],grid[0]]<99])]
            
            # new_robots_pos[2] = free_points[np.argmin([np.inner(new_robots_pos[2]-point, new_robots_pos[2]-point) \
            #     for point, grid in zip(free_points, free_points_grids) if self.cm2[grid[1],grid[0]]<99])]

            print('----------------------- voronoi -------------------------')
            print('target:', target)
            for i in range(3):
                print('robot', i, robots_pos[i], '-->', new_robots_pos[i])

            # for publishing to robot 0:
            self.way_pt0.pose.position.x = new_robots_pos[0,0]
            self.way_pt0.pose.position.y = new_robots_pos[0,1]
            self.way_pt0.pose.position.z = 0
            self.way_pt0.pose.orientation.x = self.tb0.pose.pose.orientation.x
            self.way_pt0.pose.orientation.y = self.tb0.pose.pose.orientation.y
            self.way_pt0.pose.orientation.z = self.tb0.pose.pose.orientation.z
            self.way_pt0.pose.orientation.w = self.tb0.pose.pose.orientation.w

            # for publishing to robot 1:
            self.way_pt1.pose.position.x = new_robots_pos[1,0]
            self.way_pt1.pose.position.y = new_robots_pos[1,1]
            self.way_pt1.pose.position.z = 0
            self.way_pt1.pose.orientation.x = self.tb1.pose.pose.orientation.x
            self.way_pt1.pose.orientation.y = self.tb1.pose.pose.orientation.y
            self.way_pt1.pose.orientation.z = self.tb1.pose.pose.orientation.z
            self.way_pt1.pose.orientation.w = self.tb1.pose.pose.orientation.w

            # for publishing to robot 2:
            self.way_pt2.pose.position.x = new_robots_pos[2,0]
            self.way_pt2.pose.position.y = new_robots_pos[2,1]
            self.way_pt2.pose.position.z = 0
            self.way_pt2.pose.orientation.x = self.tb2.pose.pose.orientation.x
            self.way_pt2.pose.orientation.y = self.tb2.pose.pose.orientation.y
            self.way_pt2.pose.orientation.z = self.tb2.pose.pose.orientation.z
            self.way_pt2.pose.orientation.w = self.tb2.pose.pose.orientation.w

            self.publish_pts()

    def publish_pts(self):

        self.way_pt0.header.seq = 1
        self.way_pt0.header.stamp = rospy.Time.now()
        self.way_pt0.header.frame_id = "map"

        self.way_pt1.header.seq = 1
        self.way_pt1.header.stamp = rospy.Time.now()
        self.way_pt1.header.frame_id = "map"

        self.way_pt2.header.seq = 1
        self.way_pt2.header.stamp = rospy.Time.now()
        self.way_pt2.header.frame_id = "map"


        self.pub0.publish(self.way_pt0)
        rospy.sleep(0.5)
        self.pub1.publish(self.way_pt1)
        rospy.sleep(0.5)
        self.pub2.publish(self.way_pt2)
        rospy.sleep(0.5)

if __name__ == '__main__':
  voronoi_partition()
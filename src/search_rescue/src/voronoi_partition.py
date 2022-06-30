#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import OccupancyGrid, MapMetaData, Odometry
import numpy as np

import matplotlib.pyplot as plt
from scipy.stats import multivariate_normal
from matplotlib.pyplot import cm


class voronoi_partition:

    def __init__(self):
        self.tb0 = None
        self.tb1 = None
        self.tb2 = None
        self.tb0_target = None
        self.tb1_target = None
        self.tb2_target = None
        self.og_np = None
        self.count = 0
        self.memory = None
        self.way_pt0 = PoseStamped()
        self.way_pt1 = PoseStamped()
        self.way_pt2 = PoseStamped()
        rospy.init_node('voronoi_node', anonymous=False)
        rospy.Subscriber('tb3_0/odom', Odometry , self.tb0_callback)
        rospy.Subscriber('tb3_1/odom', Odometry , self.tb1_callback)
        rospy.Subscriber('tb3_2/odom', Odometry , self.tb2_callback)
        rospy.Subscriber('map', OccupancyGrid , self.og_callback)
        self.pub0= rospy.Publisher("tb3_0/move_base_simple/goal",PoseStamped, queue_size=5)
        self.pub1= rospy.Publisher("tb3_1/move_base_simple/goal",PoseStamped, queue_size=5)
        self.pub2= rospy.Publisher("tb3_2/move_base_simple/goal",PoseStamped, queue_size=5)
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
        if self.count % 30 == 0:
            self.voronoi_compute()
        self.count += 1
        
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
    
    # convert ocupancy grid to real world map coords
    # input: the column and row number in the occupancy grid
    # output: the x and y location on real map
    def og_to_world(self, value=0):
        rows, columns = np.where(self.og_np==value)
        xs = round(self.map.info.origin.position.x,2) + round(self.map.info.resolution,2) * columns
        ys = round(self.map.info.origin.position.y,2) + round(self.map.info.resolution,2) * rows
        
        return np.column_stack((xs, ys)), np.column_stack((columns, rows))

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
            self.tb2 is not None and self.og_np is not None
        
    # density function
    def density_function(self, points, target=None, cov=1):
        if target is not None:
            return multivariate_normal.pdf(points, mean=target, cov=cov)
        else:
            return np.ones(points.shape[0])
    
    # voronoi tessellation
    def voronoi(self, generators, points, points_grids, new_points, new_points_grids, target=None, cov=1, n_iter=1):
        n_generator = generators.shape[0]

        for it in range(n_iter):
            # voronoi tessellation
            vor_indices = [np.argmin([np.inner(generator - point, generator - point) \
                for generator in generators]) for point in points]
            vor_indices = np.array(vor_indices)

            if target is None:
                centroids = np.zeros(generators.shape)
                new_points_list = new_points_grids.tolist()
                for i in range(n_generator):
                    points_i = points[vor_indices==i]
                    points_i_list = points_grids[vor_indices==i].tolist()
                    new_points_i = np.array([p for p, id in zip(points_i, points_i_list) if id in new_points_list])
                    if len(new_points_i) > 20:
                        target_i = new_points_i[np.argmax([np.inner(generators[i]-point, generators[i]-point) \
                            for point in new_points_i])]
                    elif len(new_points) > 50:
                        target_i = new_points[np.argmax([np.inner(generators[i]-point, generators[i]-point) \
                            for point in new_points])]
                    else:
                        target_i = points[np.argmax([np.inner(generators[i]-point, generators[i]-point) \
                            for point in points])]
                    weights_i = self.density_function(points_i, target_i, cov/10).reshape(-1,1)
                    centroids[i] = np.sum(weights_i * points_i, axis=0) / np.sum(weights_i)
            else:
                weights = self.density_function(points, target=target, cov=cov)
                # for i in range(n_generator):
                #     points_i = points[vor_indices==i]
                #     weights_i = weights[vor_indices==i].reshape(-1,1)
                #     centroids[i] = np.sum(points_i * weights_i, axis=0) / np.sum(weights_i)

                # mass and centriod
                mass = np.bincount(vor_indices, weights=weights)
                centroids_x = np.bincount(vor_indices, weights=weights*points[:,0])
                centroids_y = np.bincount(vor_indices, weights=weights*points[:,1])
                for i in range(n_generator):
                    if mass[i] > 0:
                        centroids_x[i] /= float(mass[i])
                        centroids_y[i] /= float(mass[i])
                centroids = np.column_stack((centroids_x, centroids_y))
            
            # update
            generators = centroids
        
        return generators

    # calculate voronoi partitions using occupancy grid and robot location
    def voronoi_compute(self):

        ## position
        # x = self.tb0.pose.pose.position.x
        # y = self.tb0.pose.pose.position.y
        ## orientatoin in quaternion
        # xo = self.tb0.pose.pose.orientation.x
        # yo = self.tb0.pose.pose.orientation.y
        # zo = self.tb0.pose.pose.orientation.z
        # wo = self.tb0.pose.pose.orientation.w


        ##### insert voronoi code here #######
        if self.info_available():
            # robot pos
            robots_pos = np.array([
                [self.tb0.pose.pose.position.x, self.tb0.pose.pose.position.y],
                [self.tb1.pose.pose.position.x, self.tb1.pose.pose.position.y],
                [self.tb2.pose.pose.position.x, self.tb2.pose.pose.position.y]
            ])

            # free points
            free_points, free_points_grids = self.og_to_world()
            if self.memory is not None:
                cur = free_points_grids.tolist()
                pre = self.memory.tolist()
                new_points = np.array([p for p, id in zip(free_points, cur) if id not in pre])
                new_points_grids = np.array([id for id in cur if id not in pre])
            else:
                new_points = np.copy(free_points)
                new_points_grids = np.copy(free_points_grids)
            self.memory = np.copy(free_points_grids)

            new_robots_pos = self.voronoi(robots_pos, free_points, free_points_grids,
                new_points, new_points_grids, cov=1)
            
            for i, pos in enumerate(new_robots_pos):
                new_robots_pos[i] = free_points[np.argmin([np.inner(pos-point, pos-point) \
                    for point in free_points])]

            print('----------------------- voronoi -------------------------')
            print('total points:', len(free_points), 'new points:', len(new_points))
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
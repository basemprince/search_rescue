import rospy
from geometry_msgs.msg import PointStamped,PoseStamped
from nav_msgs.msg import OccupancyGrid
import numpy as np
import time
import csv

List=[1,55,533]

with open(r'time_keeper.csv', 'a', newline='') as csvfile:
    fieldnames = ['robot_count','time_taken','waypoint_count']
    writer = csv.DictWriter(csvfile, fieldnames=fieldnames)

    writer.writerow({'robot_count':List[0], 'time_taken':List[1],'waypoint_count':List[2]})
#!/usr/bin/env python
import rospy
from geometry_msgs.msg import PointStamped,PoseStamped
from nav_msgs.msg import OccupancyGrid
import numpy as np
import time
import csv
import subprocess

  

class time_keeper:

    def __init__(self):
        rospy.init_node('time_keeper', anonymous=False)
        self.t0 = None
        self.t1 = None
        self.counter0 = 0  
        self.counter1 = 0  
        self.counter2 = 0
        self.row_range = [45, 350]
        self.col_range = [25, 371]
        rospy.Subscriber("/target_pose", PointStamped, self.tp_callback)
        rospy.Subscriber("tb3_0/move_base_simple/goal", PoseStamped, self.mbg0_callback)
        rospy.Subscriber("tb3_1/move_base_simple/goal", PoseStamped, self.mbg1_callback)
        rospy.Subscriber("tb3_2/move_base_simple/goal", PoseStamped, self.mbg2_callback)       
        try:
          rospy.spin()
        except KeyboardInterrupt:
          print("Shutting down")

    def og_callback(self, data):
        self.map = data
        self.og_to_numpy()

    def og_to_numpy(self):
        data = np.asarray(self.map.data, dtype=np.int8).reshape(self.map.info.height,
                                                                self.map.info.width)
        self.og_np = data
        self.og_np = self.og_np[45:350,25:371]

    def mbg0_callback(self, data):
        if self.counter0 == 0:
            self.t0 = time.time()
            string_time = time.strftime("%H:%M:%S")
            print("started timer at:",string_time)
        self.counter0 +=1

    def mbg1_callback(self, data):
        self.counter1 +=1

    def mbg2_callback(self, data):
        self.counter2 +=1

    def shutdown(self):
        print ("shutting down")

    def tp_callback(self, data):
        if self.t0 is None:
            self.t0 = time.time()-40

        if self.counter2 != 0:
            self.robot_count = 3
        elif self.counter1 != 0:
            self.robot_count = 2
        else:
            self.robot_count = 1

        self.t1 = time.time()
        self.time_elapsed = self.t1 - self.t0
        subprocess.call(['speech-dispatcher'])        #start speech dispatcher
        subprocess.call(['spd-say', '"done"'])
        print("goal reached in: ",self.time_elapsed," seconds")    

        rospy.Subscriber('map', OccupancyGrid , self.og_callback)
        rospy.sleep(1)
        self.not_discovered = self.og_np[self.og_np==-1]
        print(self.not_discovered)
        print("not discovered: ", self.not_discovered.size, "total:", self.og_np.size)
        self.percent_discovered = 1.0 - (float(self.not_discovered.size) / float(self.og_np.size))
        print("percent:" ,self.percent_discovered)
        # List 
        List=[self.robot_count,self.time_elapsed,self.counter0,self.counter1,self.counter2,self.percent_discovered,'y']
        
        with open(r'time_keeper.csv', 'a', newline='') as csvfile:
            fieldnames = ['robot_count','time_taken','waypoint_count_0','waypoint_count_1','waypoint_count_2','perc_disc','new']
            writer = csv.DictWriter(csvfile, fieldnames=fieldnames)

            writer.writerow({'robot_count':List[0], 'time_taken':List[1],'waypoint_count_0':List[2],'waypoint_count_1':List[3],'waypoint_count_2':List[4],'perc_disc':List[5],'new':List[6]})

        rospy.signal_shutdown("shutting down")


if __name__ == '__main__':
  tk = time_keeper()

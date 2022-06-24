import rospy
from nav_msgs.msg import Odometry

class odom_data:

    def __init__(self):
        self.tb0 = None
        self.tb1 = None
        self.tb2 = None
        rospy.init_node('odom_sub', anonymous=False)
        rospy.Subscriber('tb3_0/odom', Odometry , self.tb0_callback)
        rospy.Subscriber('tb3_1/odom', Odometry , self.tb1_callback)
        rospy.Subscriber('tb3_2/odom', Odometry , self.tb2_callback)
        try:
          rospy.spin()
        except KeyboardInterrupt:
          print("Shutting down")

    
    def tb0_callback(self, data):
        rospy.loginfo("recieved data - tb0")
        self.tb0 = data
        self.position_compute()
       
    def tb1_callback(self, data):
        rospy.loginfo("recieved data - tb1")
        self.tb1 = data
        self.position_compute()
        

    def tb2_callback(self, data):
        rospy.loginfo("recieved data - tb2")
        self.tb2 = data
        self.position_compute()
        

    # retrive the position information for all robots from this function [tb0, tb1, tb2]
    def position_compute(self):

      ## position
      # x = self.tb0.pose.pose.position.x
      # y = self.tb0.pose.pose.position.y
      ## orientatoin in quaternion
      # xo = self.tb0.pose.pose.orientation.x
      # yo = self.tb0.pose.pose.orientation.y
      # zo = self.tb0.pose.pose.orientation.z
      # wo = self.tb0.pose.pose.orientation.w
      if self.tb0 is not None:
        tb0_position_x = self.tb0.pose.pose.position.x
        print(tb0_position_x)



if __name__ == '__main__':
  tb_odoms = odom_data()
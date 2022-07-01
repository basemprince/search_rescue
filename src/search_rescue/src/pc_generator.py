import rospy
from sensor_msgs.msg import  LaserScan
from std_msgs.msg import Int32MultiArray
from geometry_msgs.msg import PointStamped
import math
import tf

def scan_cb(msg):

    pt_array = []
    listener = tf.TransformListener()
    target_frame = "map"
    current_frame = "tb3_0/base_scan"
    listener.waitForTransform(target_frame, current_frame, rospy.Time(), rospy.Duration(4.0))
    (trans,rot) = listener.lookupTransform(target_frame, current_frame, rospy.Time(0))

    for angle, distance in enumerate(msg.ranges):
        if distance == math.inf:
            continue
        x = math.cos(angle)/distance
        y = math.sin(angle)/distance
        pt = PointStamped()
        pt.header.frame_id = msg.header.frame_id
        pt.point.x = x
        pt.point.y = y
        pt.point.z = 0
        pt_transformed = listener.transformPoint(target_frame,pt)
        pt_array.append([pt_transformed.point.x,pt_transformed.point.y])
    print(pt_array)


rospy.init_node("laser_to_distance")

pc_pub = rospy.Publisher("/laser_to_distance", Int32MultiArray, queue_size=1)
rospy.Subscriber("/tb3_0/scan", LaserScan, scan_cb, queue_size=1)



rospy.spin()
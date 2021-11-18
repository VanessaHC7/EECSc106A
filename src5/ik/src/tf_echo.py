#!/usr/bin/env python
import tf2_ros
import rospy
from geometry_msgs.msg import TransformStamped
import sys

def listener():
	tfBuffer = tf2_ros.Buffer()
	tfListener = tf2_ros.TransformListener(tfBuffer)
	rate = rospy.Rate(10)
	while True:
		rate.sleep()

		try:
			trans = tfBuffer.lookup_transform(sys.argv[1], sys.argv[2], rospy.Time())
			print(trans)

		except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException), e:
	 		pass 
	 		print(e)
	 		rospy.logerr('FAILED TO GET TRANSformation FROM %s to %s ' % (sys.argv[1], sys.argv[2]))
	 
	

# Python's syntax for a main() method
if __name__ == '__main__':

    # Run this program as a new node in the ROS computation graph called
    # /listener_<id>, where <id> is a randomly generated numeric string. This
    # randomly generated name means we can start multiple copies of this node
    # without having multiple nodes with the same name, which ROS doesn't allow.
    rospy.init_node('listener', anonymous=True)

    listener()
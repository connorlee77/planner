#!/usr/bin/env python
# license removed for brevity



import rospy
import roslib
import tf 
from std_msgs.msg import String
import math
import geometry_msgs.msg


def takeoff():
    cmd = geometry_msgs.msg.Twist()
    o=(0, 0, 1, 0)

    cmd.linear.x = o[0]
    cmd.linear.y = o[1]
    cmd.linear.z = o[2]
    cmd.angular.x = 0
    cmd.angular.y = 0
    cmd.angular.z = o[3]

    return cmd 

def cmd_command():
    
    cmd = geometry_msgs.msg.Twist()



    return cmd
    
def closed_loop():

    rospy.init_node('closed_loop', anonymous=True)
    listener = tf.TransformListener()
    quad_vel = rospy.Publisher('cmd_vel', geometry_msgs.msg.Twist, queue_size=1)
    rate = rospy.Rate(10) # 10hz
    i = 0
    while not rospy.is_shutdown():

        try:
            (trans,rot) = listener.lookupTransform('/ground_truth/state', '/ground_truth_to_tf', rospy.Time(0))
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            continue
        rospy.loginfo(trans)
        rospy.loginfo(rot)

        if i < height:
            quad_vel.publish(takeoff())
        quad_vel.publish(cmd_command())
        rate.sleep()
        i += 1

if __name__ == '__main__':
	try:
		closed_loop()
	except rospy.ROSInterruptException:
		pass

#!/usr/bin/env python
# license removed for brevity



import rospy
from std_msgs.msg import String
import math
import geometry_msgs.msg



def cmd_command(i):
    
    cmd = geometry_msgs.msg.Twist()
    ##
    ## INSERT YOUR CODE HERE
    ##
    o = (1, 0, 0, 1)
    if i < 50:
        o=(0, 0, 1, 0)

    cmd.linear.x = o[0]
    cmd.linear.y = o[1]
    cmd.linear.z = o[2]
    cmd.angular.x = 0
    cmd.angular.y = 0
    cmd.angular.z = o[3]
    ##
    ##
    ##
    return cmd
    
def open_loop():
    rospy.init_node('open_loop', anonymous=True)
    quad_vel = rospy.Publisher('cmd_vel', geometry_msgs.msg.Twist,queue_size=1)
    rate = rospy.Rate(10) # 10hz
    i = 0
    while not rospy.is_shutdown():

        quad_vel.publish(cmd_command(i))
        rate.sleep()
        i += 1

if __name__ == '__main__':
	try:
		open_loop()
	except rospy.ROSInterruptException:
		pass

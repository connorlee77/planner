#!/usr/bin/env python
# license removed for brevity



import rospy
import roslib
import tf 
import numpy as np 
import matplotlib.pyplot as plt 
from std_msgs.msg import String
import math
import geometry_msgs.msg

def error(true_position, final_position):
    return true_position - final_position

def d_error(true_position, final_position, dt):
    return (true_position - final_position)/dt

def bernoulli_spiral(a, b, t):
    time = np.arange(1, t)

    x = a*np.exp(b*time)*np.cos(time)
    y = a*np.exp(b*time)*np.sin(time)
    # plt.plot(x, y)
    # plt.show()
    return x, y

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

def cmd_command(true_position, final_position, kp, kd, dt):
    
    output = kp * error(true_position, final_position) + kd * d_error(true_position, final_position, dt)

    cmd = geometry_msgs.msg.Twist()
    cmd.linear.x = output[0]
    cmd.linear.y = output[1]
    cmd.linear.z = 0
    cmd.angular.x = 0
    cmd.angular.y = 0
    cmd.angular.z = 0

    return cmd
    
def closed_loop():

    rospy.init_node('closed_loop', anonymous=True)
    
    listener = tf.TransformListener()
    quad_vel = rospy.Publisher('cmd_vel', geometry_msgs.msg.Twist, queue_size=1)
    
    hz = 10
    dt = 1.0/hz
    rate = rospy.Rate(hz) # 10hz
    

    steps = 30
    height = 10
    
    x, y = bernoulli_spiral(1, 0.05, steps)
    
    i = 0
    t = 0
    trans = None
    rot = None
    while not rospy.is_shutdown():
        
        try:
            (trans,rot) = listener.lookupTransform('/base_link', '/world', rospy.Time(0))
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            continue
        
        # Take off
        if i < height:
            quad_vel.publish(takeoff())
        
        # Follow path
        elif trans is not None and t < steps - 1:
            rospy.loginfo(t)
            true_position = np.array([trans[0], trans[1]])
            final_position = np.array([x[t], y[t]])
            
            quad_vel.publish(cmd_command(true_position, final_position, kp=0.5, kd=0.01, dt=dt))
            if np.linalg.norm(error(true_position, final_position)) < 1:
                t += 1
        
        # Stop moving
        else:
            cmd = geometry_msgs.msg.Twist()
            cmd.linear.x = 0
            cmd.linear.y = 0
            cmd.linear.z = 0
            cmd.angular.x = 0
            cmd.angular.y = 0
            cmd.angular.z = 0
            quad_vel.publish(cmd)
            
        rate.sleep()
        i += 1

if __name__ == '__main__':
	try:
		closed_loop()
	except rospy.ROSInterruptException:
		pass

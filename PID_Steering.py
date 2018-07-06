#!/usr/bin/env python
import rospy
import numpy as np 
from std_msgs.msg import Float64



def Steerin_control(data):
	kp=1.9789999999999974
	ki=0.9504999999999982
	kd= -0.09025000000000018
	global error
	global prev_error
	global pub

	target_angle=(data.data-15)*np.pi/180
	current_angle = (data.data)*np.pi/180
	acc=error+prev_error

	while(np.abs(current_angle-target_angle)>0.001):
		error= target_angle - current_angle

		angular_speed=kp*error+ki*acc+kd*(error-prev_error)/0.3

		angle= angular_speed*0.3

		current_angle=current_angle-angle


		pub.publish(current_angle)


		prev_error=error




if __name__=='__main__':
	rospy.init_node('steering')
	error=prev_error=0
	pub=rospy.Publisher ('/steering_angle',Float64,queue_size=1)
	rospy.Subscriber('/steer_feedback',Float64,Steerin_control)
	rospy.spin()


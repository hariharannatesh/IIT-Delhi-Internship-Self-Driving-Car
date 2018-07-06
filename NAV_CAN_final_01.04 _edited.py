#!/usr/bin/env python
import rospy
import time
import threading
import math
import pickle
import numpy as np
import matplotlib.pyplot as plt
from PCANBasic import *
from math import ceil
from std_msgs.msg import Float64
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry

#=====================================================================
#INITIALISE VARIABLES
max_steer_angle_radian = 0.69           # Based on E20 turning radius and allowed steering angle, max value will be decided, beyond which steering... 
min_steer_angle_radian = -0.69			# ...angle will get clamped to maximum; theoretically, inner wheel turns by 39 degree and outer by 29 degrees 

car_speed = 0							#Current_car_speed (Kmph)
target_speed = 0						#Target speed (Kmph)
target_steer = 0						#Target Steering angle (With sign in Degrees)
steer_feedback = 0						#Steering angle Feedback (With sign in Degrees)
count_cmd = 0 							#Number of speedometer reads between two cmd_vel_receives
vx_target = 0 							#Velocity_x (with sign) - Target
drive_mode = 0							#Driving Mode (R = 2; N = 0; D = 1)
gear_neutral = True 					#Is(Gear == NEUTRAL)
new_cmd_vel =  False 					#Whether a new command has been received for the velocity
Initial_Brake = 0						#Braking at Initial neutral condition = 30%
gear = 0								#position of gear ; neutral = 0, drive = 1, reverse = -1
acceleration = 0						#percentage press of accelerator pedal
brake = 0								#percentage press of brake pedal
prev_vx_target = 0						#last recieved target velocity
#=====================================================================
#DEFINE CONSTANTS

# Bit [55:51] Driving_Mode_RNDB_Cmd
# Bit [50] Driving mode RNDB Validity [Invalid =0| Valid=1]
# B6-> [55 54 53 52  51 1 0 0]
# 00001 -> Reverse 	
# 00010 -> neutral 	
# 00100 -> FwdDrive 
# 01000 -> Reserved
# 10000 -> Boost 
RNDB_NEUTRAL = 0x14
RNDB_REVERSE = 0x0C
RNDB_DRIVE = 0x24
RNDB_BOOST = 0x84

# Bit [63:60] Driving_Turn_Indicator_Cmd
# Bit [59] Driving_Turn_Indicator_Cmd_Valid
# 63 62 61 60 <- Bits
# 1000 -> Left Blink at 1 Hz 
# 0001 -> Right Blink at 1 Hz 
# 1001 -> Hazard Blink at 1 Hz
# 0000 -> Off
# Others -> Reserved
LAMP_OFF = 0x09
LAMP_RIND = 0x19
LAMP_LIND = 0x89
LAMP_HAZARD = 0x99

# Bit [58:57] Driving_HL_Cmd
# Bit [56] Driving_HL_Cmd_Vld
# 58 57 <- Bits
# 01 -> Low Beam 
# 10 -> High Beam 
# 00 -> Off
# 11 -> Reserved
LAMP_LBEAM = 0x0B
LAMP_HBEAM = 0x0D

# Bit [55:54] Brake_Light_Indicator_Cmd
# Bit [53] Brake_Light_Indicator_Cmd_Vld
# 55 54 <- Bits
# 01 -> Reserved
# 10 -> Reserved
# 00 -> Off
# 11 -> On
LAMP_BKLIGHT_B7 = 0x09
LAMP_BKLIGHT_B6_ON = 0xE0
LAMP_BKLIGHT_B6_OFF = 0x20

# Bit [63:58] Steering_Cntrl_Deg_Cmd 
# VALUE: 0 - 40 -> valid range in degrees

# Bit [57:56] Steering_Cntrl_Dir_Cmd
# 10 -> Turn Left
# 01 -> Turn Right 
# 00, 11 -> reserved

# Bit [55] Steering control command direction Validity 
# Bit [54] Steering control command degree Validity
# 0 -> Invalid 1 -> Valid
STEER_ZERO = 0x01
STEER_LEFT = 0xA2
STEER_RIGHT = 0xA1

BRAKE_ZERO = 0x01
BRAKE_FULL = 0xC9

ACCEL_ZERO = 0x01
ACCEL_FULL = 0xC9

WIPER_OFF = 0x20
WIPER_INT = 0x60
WIPER_HIGH = 0xA0
HORN_ON = 0x18
HORN_OFF = 0x08

#=====================================================================
#initialize e2OCAN messages
e2OCAN_RNDB = TPCANMsg()
e2OCAN_LAMP = TPCANMsg()
e2OCAN_STEER = TPCANMsg()
e2OCAN_BRAKE = TPCANMsg()
e2OCAN_ACCEL = TPCANMsg()
e2OCAN_WIPHORN = TPCANMsg()

for i in range(8):
	e2OCAN_RNDB.DATA[i] = 0x00
	e2OCAN_LAMP.DATA[i] = 0x00
	e2OCAN_STEER.DATA[i] = 0x00
	e2OCAN_BRAKE.DATA[i] = 0x00
	e2OCAN_ACCEL.DATA[i] = 0x00
	e2OCAN_WIPHORN.DATA[i] = 0x00

# Motorola Byte Order (Big Endian)
# Little Endian (64-bits): 	B7 B6 B5 B4 B3 B2 B1
# Big Endian (64-bits): 	B0 B1 B3 B4 B5 B6 B7

# Bits in Big Endian Byte Order
# B6->[55 54 53 52  51 50 49 48] B7->[63 62 61 60  59 58 57 56]


e2OCAN_RNDB.ID = 0x778
e2OCAN_RNDB.MSGTYPE = PCAN_MESSAGE_STANDARD
e2OCAN_RNDB.LEN = 8


e2OCAN_LAMP.ID = 0x776
e2OCAN_LAMP.MSGTYPE = PCAN_MESSAGE_STANDARD
e2OCAN_LAMP.LEN = 8

e2OCAN_STEER.ID = 0x774
e2OCAN_STEER.MSGTYPE = PCAN_MESSAGE_STANDARD
e2OCAN_STEER.LEN = 8

e2OCAN_BRAKE.ID = 0x772
e2OCAN_BRAKE.MSGTYPE = PCAN_MESSAGE_STANDARD
e2OCAN_BRAKE.LEN = 8

e2OCAN_ACCEL.ID = 0x770
e2OCAN_ACCEL.MSGTYPE = PCAN_MESSAGE_STANDARD
e2OCAN_ACCEL.LEN = 8

e2OCAN_WIPHORN.ID = 0x76D
e2OCAN_WIPHORN.MSGTYPE = PCAN_MESSAGE_STANDARD
e2OCAN_WIPHORN.LEN = 8

#=====================================================================
#Construct PCAN bus Handle and Print BUS Parameters
m_PcanHandle = PCAN_USBBUS1
m_objPCANBasic = PCANBasic()
Status = m_objPCANBasic.Initialize(m_PcanHandle, PCAN_BAUD_500K, 0, 0, 0)
print(Status)

print("PCAN_CHANNEL_CONDITION (Before) = " + str(m_objPCANBasic.GetValue(m_PcanHandle, PCAN_CHANNEL_CONDITION)))
print("PCAN_DEVICE_NUMBER = " + str(m_objPCANBasic.GetValue(m_PcanHandle, PCAN_DEVICE_NUMBER)))
print("PCAN_API_VERSION= " + str(m_objPCANBasic.GetValue(m_PcanHandle, PCAN_API_VERSION)))
print("PCAN_HARDWARE_NAME = " + str(m_objPCANBasic.GetValue(m_PcanHandle, PCAN_HARDWARE_NAME)))
print("PCAN_CHANNEL_CONDITION = " + str(m_objPCANBasic.GetValue(m_PcanHandle, PCAN_CHANNEL_CONDITION)))
print("PCAN_CHANNEL_VERSION = " + str(m_objPCANBasic.GetValue(m_PcanHandle, PCAN_CHANNEL_VERSION)))
print("PCAN_BITRATE_INFO = " + str(m_objPCANBasic.GetValue(m_PcanHandle, PCAN_BITRATE_INFO)))
print("PCAN_BUSSPEED_NOMINAL = " + str(m_objPCANBasic.GetValue(m_PcanHandle, PCAN_BUSSPEED_NOMINAL)))
print("PCAN_RECEIVE_STATUS = " + str(m_objPCANBasic.GetValue(m_PcanHandle, PCAN_RECEIVE_STATUS)))

#Set the Initial Parameters of the Car
e2OCAN_LAMP.DATA[6] = 0x20
e2OCAN_STEER.DATA[6] = 0xC0
e2OCAN_RNDB.DATA[6] = RNDB_NEUTRAL
e2OCAN_LAMP.DATA[7] = LAMP_HAZARD
e2OCAN_STEER.DATA[7] = STEER_ZERO
e2OCAN_BRAKE.DATA[7] = ((Initial_Brake/2)<<1) | 1
e2OCAN_ACCEL.DATA[7] = ACCEL_ZERO
e2OCAN_WIPHORN.DATA[7] = WIPER_OFF | HORN_OFF

#=====================================================================
#CALLBACK Function of Threads : tmrRead and tmrWrite
def ReadMessages():
	global car_speed
	global target_speed
	global steer_feedback
	global count_cmd
	global gear
	try:
		Status_pcan = PCAN_ERROR_OK
		while (not (Status_pcan & PCAN_ERROR_QRCVEMPTY)):
			Message = m_objPCANBasic.Read(m_PcanHandle)
			
			Status_pcan = Message[0]
			if Message[0] == PCAN_ERROR_OK:
				MSG_DATA = Message[1]
				msgTimeStamp = Message[2]
				
				Message_time = (msgTimeStamp.micros + 1000 * msgTimeStamp.millis + 0x100000000 * 1000 * msgTimeStamp.millis_overflow)
				print (Message_time)
				for i in range(MSG_DATA.LEN):
					print(format(MSG_DATA.DATA[i],'02x'), " ")
				
				MsgID = format(MSG_DATA.ID,'04x')
				DATA7 = format(MSG_DATA.DATA[7],'02x')
				DATA6 = format(MSG_DATA.DATA[6],'02x')
				
				#=======================Vehicle speed status============================
				if (MsgID == '076c'):
							
					car_speed = int(DATA7,16)
					print "Speed of Car : ", car_speed, " Kmph \n"

					if (car_speed == 0) and (gear != 0):				# if gear is not neutral and car_speed is 0
						count_cmd = count_cmd + 1						# we increase a counter
					else:												# for any other condition
						count_cmd = 0									# counter will go back to zero

					gear_select()										# gear will be selected and target speed will be assigned
					pub_gear.publish(Float64(gear))
					car_speed_ROS_Message = Float64(car_speed)
					target_speed_ROS_Message = Float64(target_speed)

					pub_State.publish(car_speed_ROS_Message)			# car_speed is published . To be used by PID controller
					pub_Target.publish(target_speed_ROS_Message)		# target_speed is published. To be used by PID controller
				
				#=======================Wiper and Horn==================================		
				elif (MsgID == '076e'):
					if(DATA7 == 'a0'):
						print "Wiper : ON ---- Horn : ON"
					elif(DATA7 == '20'):
						print "Wiper : OFF ---- Horn : ON"
					elif(DATA7 == '80'):
						print "Wiper : ON ---- Horn : OFF"
					elif(DATA7 == '00'):
						print "Wiper : OFF ---- Horn : OFF"
					print "\n"
				
				#=======================THROTTLE=====================
				elif (MsgID == '0771'):
					print "Throttle Percentage of Car : ", int(DATA7,16)/2, "\n"
				
				#=======================BRAKES======================
				elif (MsgID == '0773'):
					print "Braking Percentage of Car : ", int(DATA7,16)/2, "\n"
				
				#=======================Steering Angle and Direction=====================
				elif (MsgID == '0775'):
					Steer = int(DATA7,16)
					Steer_angle = (Steer>>2)
									   
					if((Steer&3) == 2):						  #Clockwise
						Steer_angle = Steer_angle*1               
					elif((Steer&3) == 1):					  #AntiClockwise
						Steer_angle = Steer_angle*(-1)
					steer_feedback = Steer_angle
					pub_steer.publish(np.int64(steer_feedback))
					print "Steering Angle : ",Steer_angle, "\n"

				#=======================Turn Indicators and Head Lights=================
				elif (MsgID == '0777'):
					if(DATA7[0] == '8'):
						print "Indicator Status : Left Indicator"
					elif(DATA7[0] == '1'):
						print "Indicator Status : Right Indicator"
					elif(DATA7[0] == '9'):
						print "Indicator Status : Hazard Indicator"
					elif(DATA7[0] == '0'):
						print "Indicator Status : OFF"
					print "\n"

					if(DATA7[1] == '4'):
						print "Head Lights : ON"
					elif(DATA7[1] == '0'):
						print "Head Lights : OFF"
					print "\n"

				#=======================DRIVING MODE and RNDB============================
				elif (MsgID == '0779'):
					if(DATA7 == '40'):
						print "Driving Mode : Manual"
					elif(DATA7 == '80'):
						print "Driving Mode : Autonomous"
					elif(DATA7 == 'c0'):
						print "Driving Mode : Homing in Progress"
					else:
						print "Driving Mode : Invalid"
					print "\n"

					if(DATA6 == '08'):
						drive_mode = 2
						print "Gear Status : Reverse"
					elif(DATA6 == '10'):
						drive_mode = 0
						print "Gear Status : Neutral"
					elif(DATA6 == '20'):
						drive_mode = 1
						print "Gear Status : Drive"
					elif(DATA6 == '80'):
						print "Gear Status : Boost"
					else:
						print "Gear Status : Invalid"
					print "\n"
	except KeyboardInterrupt:
		tmrRead.stop()
		m_objPCANBasic.Uninitialize(m_PcanHandle)
		raise SystemExit, 1

def WriteMessages():
	try:
		m_objPCANBasic.Write(m_PcanHandle, e2OCAN_RNDB)
		m_objPCANBasic.Write(m_PcanHandle, e2OCAN_LAMP)
		m_objPCANBasic.Write(m_PcanHandle, e2OCAN_STEER)
		m_objPCANBasic.Write(m_PcanHandle, e2OCAN_BRAKE)
		m_objPCANBasic.Write(m_PcanHandle, e2OCAN_ACCEL)
		m_objPCANBasic.Write(m_PcanHandle, e2OCAN_WIPHORN)
		
	except KeyboardInterrupt:
		tmrWrite.stop()
		m_objPCANBasic.Uninitialize(m_PcanHandle)
		raise SystemExit, 1

#=====================================================================
class TimerRepeater(object):
	def __init__(self, name, interval, target, isUi, args=[], kwargs={}):
		"""
		Creates a timer.
		Parameters:
			name        name of the thread
			interval    interval in second between execution of target
			target      function that is called every 'interval' seconds
			args        non keyword-argument list for target function
			kwargs      keyword-argument list for target function
		"""
		# define thread and stopping thread event
		self._name = name
		self._thread = None
		self._event = None
		self._isUi = isUi
		# initialize target and its arguments
		self._target = target
		self._args = args
		self._kwargs = kwargs
		# initialize timer
		self._interval = interval
		self._bStarted = False

	def _run(self):
		"""
		Runs the thread that emulates the timer.
		Returns:
			None
		"""
		while not self._event.wait(self._interval):
			if self._isUi:
				# launch target in the context of the main loop
				root.after(1, self._target,*self._args, **self._kwargs)
			else:
				self._target(*self._args, **self._kwargs)

	# Starts the timer
	def start(self):
		# avoid multiple start calls
		if (self._thread == None):
			self._event = threading.Event()
			self._thread = threading.Thread(None, self._run, self._name)
			self._thread.start()

	# Stops the timer
	def stop(self):
		if (self._thread != None):
			self._event.set()
			self._thread = None

#=====================================================================
#DEFINE CALLBACK functions
def callback(pid_output):
	global target_steer
	global vx_target
	global gear_neutral
	global acceleration
	global brake
	global target_speed
	global car_speed
	global prev_vx_target
		#========================== SET Throttle and BRAKE% ====================
	control_effort = int(round(pid_output.data))
	if control_effort > 50:
		control_effort = 50
	if control_effort < (-50):
		control_effort = (-50)
	
	if gear_neutral == True:												# if gear is neutral
		e2OCAN_ACCEL.DATA[7]= ACCEL_ZERO									# acceleration = 0
		acceleration = 0
		if (vx_target == 0):												# if final target velocity is 0 
			e2OCAN_BRAKE.DATA[7]= ((abs(30/2))<<1) | 1						# fixed braking of 30 % pedal
			brake = ((abs(30/2))<<1) | 1
		else:																# if final target is not 0
			e2OCAN_BRAKE.DATA[7] = BRAKE_ZERO								# brake is 0
			brake = 0

	else:																	# if gear is not neutral
		if control_effort >= 0:
			e2OCAN_ACCEL.DATA[7] = ((control_effort/2)<<1) | 1
			e2OCAN_BRAKE.DATA[7] = BRAKE_ZERO
			acceleration = control_effort
			brake = 0
		else:
			e2OCAN_ACCEL.DATA[7]= ACCEL_ZERO
			acceleration = 0
			if abs(car_speed) > abs(target_speed):							# if the current car speed is more than the desired speed
				e2OCAN_BRAKE.DATA[7]= ((abs(control_effort/2))<<1) | 1
				brake = abs(control_effort)
			elif gear == 1 and ((prev_vx_target - vx_target) > 2) :			# if gear is in drive mode and a new target speed is 2 or more units less than the last target speed
				e2OCAN_BRAKE.DATA[7]= ((abs(control_effort/2))<<1) | 1
				brake = abs(control_effort)
			elif gear == -1 and ((vx_target - prev_vx_target) > 2):			# if gear is in reverse mode and a new target speed is 2 or more units greater than the last target speed
				e2OCAN_BRAKE.DATA[7]= ((abs(control_effort/2))<<1) | 1
				brake = abs(control_effort)
			else:															# in all the other conditions
				e2OCAN_BRAKE.DATA[7] = BRAKE_ZERO
				brake = 0

	#============================ SET Steering Angle =======================
	if target_steer >= 0:
		e2OCAN_STEER.DATA[7] = ((abs(target_steer)<<2)| 2)
	else:
		e2OCAN_STEER.DATA[7] = ((abs(target_steer)<<2)| 1)
	pub_acceleration.publish(np.int64(acceleration))
	pub_brake.publish(np.int64(brake))

def cmd_vel_callback(velocity):
	global target_speed
	global target_steer
	global vx_target
	global new_cmd_vel
	global count_cmd
	global min_steer_angle_radian
	global max_steer_angle_radian
	global prev_vx_target

	prev_vx_target = vx_target 														# the last assigned target velocity to vx_target is assigned to prev_vx_target
	
	vx_target = int(round(velocity.linear.x*18/5))									# the new velocity from cmd_vel topic is assigned to vx_target
	
	if (abs(velocity.angular.z) <= 0.001):				# when either angular velocity is 0 or linear velocity is 0
		steer_radian = 0
	else:
		#radius_car = velocity.linear.x/velocity.angular.z
		radius_car = 15/velocity.angular.z
		#steer_radian = math.atan(1.958/radius_car)									# either formula with tan and sin is currect. This will be checked only during actual testing
		steer_radian = 18 * math.asin(1.129/radius_car)
		print steer_radian
	
	target_steer_radian = max(min_steer_angle_radian,min(max_steer_angle_radian,steer_radian))
	target_steer = int(round(target_steer_radian*180/np.pi))									# rounded after converting to degrees
	pub_steer_target.publish(Int64(target_steer))

def cmd_steer_angle(angle_data):
	global target_speed
	global target_steer
	global vx_target
	global new_cmd_vel
	global count_cmd
	global min_steer_angle_radian
	global max_steer_angle_radian
	global prev_vx_target

	prev_vx_target = vx_target 														# the last assigned target velocity to vx_target is assigned to prev_vx_target
	
	vx_target = int(round(velocity.linear.x*18/5))									# the new velocity from cmd_vel topic is assigned to vx_target
	
	#if (abs(velocity.angular.z) <= 0.001):				# when either angular velocity is 0 or linear velocity is 0
	#	steer_radian = 0
	#else:
		#radius_car = velocity.linear.x/velocity.angular.z
	#	radius_car = 15/velocity.angular.z
		#steer_radian = math.atan(1.958/radius_car)									# either formula with tan and sin is currect. This will be checked only during actual testing
	#	steer_radian = 18 * math.asin(1.129/radius_car)
	#	print steer_radian
	steer_radian=angle_data
	
	target_steer_radian = max(min_steer_angle_radian,min(max_steer_angle_radian,steer_radian))
	target_steer = int(round(target_steer_radian*180/np.pi))									# rounded after converting to degrees
	pub_steer_target.publish(Int64(target_steer))

def odom_send_callback(data):
	global car_speed
	global steer_feedback
	global vx_target
	
	if drive_mode == 1:
		car_speed = car_speed * 1
	elif drive_mode == 2:
		car_speed = car_speed * (-1)
	data.twist.twist.linear.x = car_speed
	#data.twist.twist.angular.z = ((vx_target*5/18)/1.958)*math.tan((steer_feedback*np.pi/180))
	data.twist.twist.angular.z = ((vx_target*5/18)/1.129)*math.sin(((steer_feedback/18)*np.pi/180))
	odom_full.publish(data)

#=====================================================================
#DEFINE functions
def gear_select():
	global car_speed
	global target_speed
	global vx_target
	global gear_neutral
	global new_cmd_vel
	global gear
	global prev_vx_target
	
	if (prev_vx_target != vx_target) | (vx_target == 0):		# if either there is a difference in the vx target recieved or vx target speed is zero
		if gear_neutral == True:								# if gear is neutral
			if vx_target > 0:
				e2OCAN_RNDB.DATA[6] = RNDB_DRIVE
				gear_neutral = False
				gear = 1
			elif vx_target < 0:
				e2OCAN_RNDB.DATA[6] = RNDB_REVERSE
				gear_neutral = False
				gear = -1
			target_speed = abs(vx_target)
		else:													# if gear is either drive or reverse mode
			if gear == 1:										# if gear is in drive mode
				if vx_target >= 0:
					target_speed = abs(vx_target)
				elif vx_target < 0:
					target_speed = 0
			elif gear == -1:									# if gear is in reverse mode
				if vx_target <= 0:
					target_speed = abs(vx_target)
				elif vx_target > 0:
					target_speed = 0
		if (abs(car_speed) == 0) and (count_cmd > 500):			# if car speed is 0 for 5 continous seconds
			e2OCAN_RNDB.DATA[6] = RNDB_NEUTRAL
			gear_neutral = True
			gear = 0

#=====================================================================
#ROS NODE : SUBSCRIBERS AND PUBLISHERS ; TimerRepeater Class Objects
if __name__ == '__main__':
	rospy.init_node('e2oMove')

	controller_output = rospy.Subscriber('/control_effort', Float64, callback)
	#target_velocity = rospy.Subscriber('/cmd_vel', Twist, cmd_vel_callback)
	odom_without_speed = rospy.Subscriber('/odom1', Odometry, odom_send_callback)
	steering_angle= rospy.Subscriber('/steering_angle',Float64,cmd_steer_angle)

	odom_full = rospy.Publisher('/odom', Odometry, queue_size=1)
	pub_State = rospy.Publisher('/state', Float64, queue_size=1)
	pub_Target = rospy.Publisher('/setpoint' , Float64, queue_size=1)
	pub_gear = rospy.Publisher('/gear' , Float64, queue_size=1)
	pub_acceleration = rospy.Publisher('/acceleration' , Float64, queue_size=1)
	pub_brake = rospy.Publisher('/brake' , Float64, queue_size=1)
	pub_steer = rospy.Publisher('/steer_feedback', Float64, queue_size=1)
	pub_steer_target = rospy.Publisher('/steer_target', Float64, queue_size=1)
	

	tmrRead = TimerRepeater("tmrRead", 0.010, ReadMessages, False)
	tmrWrite = TimerRepeater("tmrWrite", 0.010, WriteMessages, False)
	tmrWrite.start()
	tmrRead.start()
	rospy.spin()

#=====================================================================


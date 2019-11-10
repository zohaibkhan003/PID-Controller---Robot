#!/usr/bin/env python

from math import pi, sqrt, atan2, cos, sin
import matplotlib.pyplot as plt

import rospy
import tf
from geometry_msgs.msg import Twist, Pose2D
from nav_msgs.msg import Odometry


counter = 0;							
turncount = 0;	
counttoend = 0;								
setValue = 0;	
poutput = 1;
previousE = 0;
currentE = 0;
derivativeE = 0;
	
					

class Turtlebot():
    def __init__(self):
        rospy.init_node("turtlebot_move")
        rospy.loginfo("Press CTRL + C to terminate")

        self.pose = Pose2D()

	self.A = list()		# creates two empty list 
	self.B = list()

        self.odom_sub = rospy.Subscriber("odom", Odometry, self.odom_callback)

        self.vel = Twist()
        self.vel_pub = rospy.Publisher("cmd_vel_mux/input/navi", Twist, queue_size=10)

        self.logging_counter = 0
        self.rate = rospy.Rate(10)	# counts 10 times per second
        self.run()
        self.visualization()


    def run(self):
	global counter							
	global turncount		# vaiables to keep desired variables accound for
	global counttoend
	global setValue
	global Poutput
	global previousE
	global currentE
	global derivativeE
	
	
	KpValue = 0.6			# kp and Kd values that work with the system that allows the better results with tunage
	KdValue = 5
	setPoint = 0
	positionX = self.pose.x
	previousE = setPoint - self.pose.theta	# the intial point is set up here so when it goes in the loop it can be populated
        vel = Twist()
        vel.linear.x = 0.5						
        vel.angular.z = 0
	
        while not rospy.is_shutdown():

		if counter == 0:
			while turncount != 82:	      # counter counts to 80 beacuse Hz (10), which mean it ticks every .1 s. 80(.1) = 8, 4meters = 0.5/8. thus it travels for 4 meters			
				positionX = self.pose.x
				ErrorValue = setPoint - self.pose.theta
				currentE = setPoint - self.pose.theta	# current error is set with setpoint minus the angel of the postion during that time
				rospy.loginfo("Current Position x: %s", positionX)
				rospy.loginfo("Error Value: %s", ErrorValue)
				Poutput = KpValue * (setPoint - self.pose.theta) + KdValue * (currentE - previousE) #the equation to transfer to poutput 
				rospy.loginfo("Poutput: %s", Poutput)
				vel.linear.x = 0.5					
		      		vel.angular.z = Poutput		# z becomes the the poutput value to get the desired angular velocity. 
				self.vel_pub.publish(vel)
			   	self.rate.sleep()
				previousE = currentE		# the prevoius errors becomes the currents and loop continues to be can get the subtraction correctly.
				rospy.loginfo("Time: %s", turncount)
				turncount += 1
		
		counter += 1
		turncount = 0


		#### Same logic is used for the whole system as explained above. The only thing that changes is the setpoint, which follows pi/2 pi pi/2 and 0 to make the approitate turns. This is beacuse 			Gazebo takes the values of [pi,-pi]


		if counter == 1:					
			while turncount < 120:
				setPoint = 1.57	
				currentE = setPoint - self.pose.theta
				Poutput = KpValue * (setPoint - self.pose.theta) + KdValue * (currentE - previousE)				
				vel.linear.x = 0
				rospy.loginfo("P : %s", Poutput)
				vel.angular.z = Poutput	
				previousE = currentE			
				self.vel_pub.publish(vel)
				self.rate.sleep()	
				rospy.loginfo("Time: %s", turncount)
				turncount += 1

		counter +=1
		turncount = 0
				
		
		if counter == 2:
			while turncount != 82:
				positionX = self.pose.x
				setPoint = 1.57
				ErrorValue = setPoint - self.pose.theta
				currentE = setPoint - self.pose.theta
				Poutput = KpValue * (setPoint - self.pose.theta) + KdValue * (currentE - previousE)
				rospy.loginfo("Current Position x: %s", positionX)
				rospy.loginfo("Error Value: %s", ErrorValue)
				rospy.loginfo("Poutput: %s", Poutput)
				vel.linear.x = 0.5					 
		      		vel.angular.z = Poutput
				self.vel_pub.publish(vel)
			   	self.rate.sleep()
				previousE = currentE
				rospy.loginfo("Time: %s", turncount)
				turncount += 1

		counter +=1
		turncount = 0

		if counter == 3:					
			while turncount < 120:	
				setPoint = 3.14	
				currentE = setPoint - self.pose.theta
				Poutput = KpValue * (setPoint - self.pose.theta) + KdValue * (currentE - previousE)				 
				vel.linear.x = 0
				rospy.loginfo("P : %s", Poutput)
				vel.angular.z = Poutput	
				previousE = currentE			
				self.vel_pub.publish(vel)
				self.rate.sleep()	
				rospy.loginfo("Time: %s", turncount)
				turncount += 1
				
		counter +=1
		turncount = 0

		if counter == 4:
			while turncount != 82:
				positionX = self.pose.x
				setPoint = 3.14
				currentE = setPoint - self.pose.theta
				Poutput = KpValue * (setPoint - self.pose.theta) + KdValue * (currentE - previousE)
				ErrorValue = setPoint - self.pose.theta
				rospy.loginfo("Current Position x: %s", positionX)
				rospy.loginfo("Error Value: %s", ErrorValue)
				rospy.loginfo("Poutput: %s", Poutput)
				vel.linear.x = 0.5					 
		      		vel.angular.z = Poutput
				previousE = currentE	
				self.vel_pub.publish(vel)
			   	self.rate.sleep()
				rospy.loginfo("Time: %s", turncount)
				turncount += 1

		counter +=1
		turncount = 0



		if counter == 5:					
			while turncount < 120:	
				setPoint = -1.57
				currentE = setPoint - self.pose.theta
				currentE = abs(currentE)
				Poutput = KpValue * (setPoint - self.pose.theta) + KdValue * (currentE - previousE)			
				vel.linear.x = 0
				rospy.loginfo("P : %s", Poutput)
				vel.angular.z = (Poutput)
				previousE = currentE				
				self.vel_pub.publish(vel)
				self.rate.sleep()
				rospy.loginfo("Time: %s", turncount)
				turncount += 1

		counter +=1
		turncount = 0

		if counter == 6:
			while turncount != 82:
				positionX = self.pose.x
				setPoint = -1.57
				currentE = setPoint - self.pose.theta
				Poutput = KpValue * (setPoint - self.pose.theta) + KdValue * (currentE - previousE)
				ErrorValue = setPoint - self.pose.theta
				rospy.loginfo("Current Position x: %s", positionX)
				rospy.loginfo("Error Value: %s", ErrorValue)
				rospy.loginfo("Poutput: %s", Poutput)
				vel.linear.x = 0.5					 
		      		vel.angular.z = Poutput
				previousE = currentE	
				self.vel_pub.publish(vel)
			   	self.rate.sleep()
				rospy.loginfo("Time: %s", turncount)
				turncount += 1

		counter+=1
		turncount = 0
		checker = 0

		if counter == 7:
			while turncount < 120:	
				setPoint = 0	
				currentE = (setPoint - self.pose.theta)
				checker = currentE - previousE
				rospy.loginfo("Error in Derivative : %s", checker)
				Poutput = KpValue * (setPoint - self.pose.theta) + KdValue * (currentE - previousE)		 
				vel.linear.x = 0
				rospy.loginfo("P : %s", Poutput)
				vel.angular.z = (Poutput)	
				previousE = currentE			
				self.vel_pub.publish(vel)
				self.rate.sleep()	
				rospy.loginfo("Time: %s", turncount)
				turncount += 1
		
		counter +=1

		if counter == 8:
			vel.linear.x = 0
			vel.angular.z = 0			
			self.vel_pub.publish(vel)
			self.rate.sleep()	
			

        pass


    def visualization(self):
        # Add your code here to plot trajectory
	plt.plot(self.A,self.B)				# plot the trajectory
	plt.show()
        pass


    def odom_callback(self, msg):
        # Get (x, y, theta) specification from odometry topic
        quarternion = [msg.pose.pose.orientation.x,msg.pose.pose.orientation.y,\
                    msg.pose.pose.orientation.z, msg.pose.pose.orientation.w]
        (roll, pitch, yaw) = tf.transformations.euler_from_quaternion(quarternion)

        self.pose.theta = yaw
        self.pose.x = msg.pose.pose.position.x
        self.pose.y = msg.pose.pose.position.y

        # Logging once every 100 times
        self.logging_counter += 1
        if self.logging_counter == 100:
            self.logging_counter = 0
            #rospy.loginfo("odom: x=" + str(self.pose.x) +\
             #   ";  y=" + str(self.pose.y) + ";  theta=" + str(yaw))
            # Add your code here to save the trajectory
	    self.A.append(self.pose.x)
            self.B.append(self.pose.y)			# append the list created with the current theta values coming in


if __name__ == '__main__':
    try:
        whatever = Turtlebot()
    except rospy.ROSInterruptException:
        rospy.loginfo("Action terminated.")

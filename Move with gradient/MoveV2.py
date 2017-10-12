# u know the difference
#!/usr/bin/env python
import roslib
import rospy
import numpy as np
import math
import roslib
import rospy
from numpy import *
from nav_msgs.msg import *
from std_msgs.msg import *
from sensor_msgs.msg import Image, LaserScan
from nav_msgs.msg import OccupancyGrid, Odometry
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
import actionlib
from actionlib_msgs.msg import *
from geometry_msgs.msg import Pose, Point, Quaternion
from go_to_specific_point_on_map import *
from geometry_msgs.msg import *
from tf import *
import tf
import PyKDL
class myMoveG:
	def __init__(self,goalx1,goaly1):
		#set the stepsize to be 0.5,which means the size of the grid is 0.5*0.
		self.stepsize=0.5 
		#Declare the starting pose, current pose and the pose before the current pose
		self.startPose=PoseWithCovarianceStamped()
		self.currentPose=PoseWithCovarianceStamped()
		self.previousPose=PoseWithCovarianceStamped()
		self.previousCellx=0#the x index of the cell the previous pose locates
		self.previousCelly=0#the y index of the cell the previous pose locates
		self.currentCellx=0#the y index of the cell the current pose locates
		self.currentCelly=0#the y index of the cell the current pose locates
		self.goalx=goalx1#initialize the x coordinate of the goal point
		self.goaly=goaly1#initialize the y coordinate of the goal point
		self.goalcellx=0#the x index of the cell where the goal pose locates
		self.goalcelly=0#the y index of the cell where the goal pose locates
		self.cmd_vel = rospy.Publisher('cmd_vel_mux/input/navi', Twist, queue_size=10)#publisher to control turtlebot
		rospy.Subscriber("amcl_pose", PoseWithCovarianceStamped, self.callback)#subscribe to "amcl_pose" topic
		rospy.Subscriber("initialpose", PoseWithCovarianceStamped, self.startCall)#susbscribe to the "initialpose topic"

		#twist message that will be published by cmd_vel
		self.vx=0.0
		self.vy=0.0
		self.vz=0.0
		self.ax=0.0	
		self.ay=0.0
		self.az=0.0


		#flags
		self.reached=False# flag to see if the Turtlebot reaches the goal cell or not
		self.alignCell=0# flag to see if the Turtlebot aligns with the goal or not

  		self.base_frame = rospy.get_param('~base_frame', '/base_link')
        	self.odom_frame = rospy.get_param('~odom_frame', '/odom')
        	# Initialize the tf listener
        	self.tf_listener = tf.TransformListener()
        	# Set the odom frame
        	self.odom_frame = '/odom'

	def callback(self,msg):
		#subsribe to the "amcl_pose" topic, if in a new cell,update the currentCell,Update the neighbor value,using the neighbor value to change the x,y
		self.currentPose=msg#get the current pose of the Turtlebot
		self.previousCellx=self.currentCellx#set the previouslyCellx to currentCellx,so it keeps the information of cell position of Turtlebot before updating
		self.previousCelly=self.currentCelly#set the previouslyCelly to currentCelly,so it keeps the information of cell position of Turtlebot before updating
		self.checkCell(self.currentPose)#Check the cell the currentPose locates in and update the currentCellx and currentCelly
		if(self.previousCellx!=self.currentCellx or self.previousCelly!=self.currentCelly ):
		#if the the above statement is true, that means the Turtlebot is in another cell now, so we need to check the neighbor value again an update the twist message accordingly
			#show the previous cell and current cell
			print "self.previousCellx"
			print self.previousCellx
			print "self.previousCelly"
			print self.previousCelly
			print "self.currentCellx ="
			print self.currentCellx
			print "self.currentCelly ="
			print self.currentCelly
			print "into aonther cell now"
			#check the neighbor value again and update the twist message accordingly
			self.checkNeighbourValue()
			self.updateTwist()
	
			
					
	def startCall(self,msg):
		#initial pose
		print "we have initialpose"
		self.currentPose=msg
		self.checkCell(self.currentPose)
		self.checkNeighborValue()
		self.updateTwist()
		
	
	def checkCell(self,msg):
		#update the currentCell information
		#using Pose message and stepsize to get the coordinate of the cell
		xloc=msg.pose.pose.position.x/self.stepsize
		yloc=msg.pose.pose.position.y/self.stepsize
		xcellNo=math.ceil(xloc)
		ycellNo=math.ceil(yloc)
		# assign the x index of cell and y index of the cell to the private variables
		self.currentCellx=xcellNo
		self.currentCelly=ycellNo
	

	def start(self):
		#first check the cell wehre the goal pose locates
		self.checkGoalCell()
		# set the rate to be 10hz
		rate = rospy.Rate(10)
		#try to find the transformtion between odom frame to base_link or base_footprint frame
		try:
            		self.tf_listener.waitForTransform(self.odom_frame, '/base_footprint', rospy.Time(), rospy.Duration(1.0))
            		self.base_frame = '/base_footprint'
        	except (tf.Exception, tf.ConnectivityException, tf.LookupException):
            		try:
                		self.tf_listener.waitForTransform(self.odom_frame, '/base_link', rospy.Time(), rospy.Duration(1.0))
                		self.base_frame = '/base_link'
            		except (tf.Exception, tf.ConnectivityException, tf.LookupException):
                		rospy.loginfo("Cannot find transform between /odom and /base_link or /base_footprint")
                		rospy.signal_shutdown("tf Exception")

		while not rospy.is_shutdown():	
			cmd=Twist()
			cmd.linear.x = self.vx
			cmd.angular.z = self.az
			self.cmd_vel.publish(cmd)
	    		# wait for 0.1 seconds (10 HZ) and publish again
            		rate.sleep()
			#check if it reaches the goal cell or not
			self.reachTarget()	
			while(self.reached==True):
				pass

	def checkNeighbourValue(self):
		#update the neighbor value and choose the next cell, and the next local goal is the center of the cell
		max=0#initialize the max value to be 0
		self.nextLocalGoalIndex=0# initialize the index of the next local goal cell to be 0
	
		#calculate the value of neighbor cell in 8 directions, which surround the cell the turtlebot locates
		#Here we just get the distance from the neighbor cell to the goal cell, get the reciprocal and assign it to be the value
		up=1/(math.sqrt((self.currentCellx-self.goalcellx)*(self.currentCellx-self.goalcellx)+((self.currentCelly+1)-self.goalcelly)*((self.currentCelly+1)-self.goalcelly))+1)
		
		down=1/(math.sqrt((self.currentCellx-self.goalcellx)*(self.currentCellx-self.goalcellx)+((self.currentCelly-1)-self.goalcelly)*((self.currentCelly-1)-self.goalcelly))+1)

		left=1/(math.sqrt((self.currentCellx-1-self.goalcellx)*(self.currentCellx-1-self.goalcellx)+((self.currentCelly)-self.goalcelly)*((self.currentCelly)-self.goalcelly))+1)

		right=1/(math.sqrt((self.currentCellx+1-self.goalcellx)*(self.currentCellx+1-self.goalcellx)+((self.currentCelly)-self.goalcelly)*((self.currentCelly)-self.goalcelly))+1)

		upleft=1/(math.sqrt((self.currentCellx-1-self.goalcellx)*(self.currentCellx-1-self.goalcellx)+((self.currentCelly+1)-self.goalcelly)*((self.currentCelly+1)-self.goalcelly))+1)

		upright=1/(math.sqrt((self.currentCellx+1-self.goalcellx)*(self.currentCellx+1-self.goalcellx)+((self.currentCelly+1)-self.goalcelly)*((self.currentCelly+1)-self.goalcelly))+1)

		downleft=1/(math.sqrt((self.currentCellx-1-self.goalcellx)*(self.currentCellx-1-self.goalcellx)+((self.currentCelly-1)-self.goalcelly)*((self.currentCelly-1)-self.goalcelly))+1)

		downright=1/(math.sqrt((self.currentCellx+1-self.goalcellx)*(self.currentCellx+1-self.goalcellx)+((self.currentCelly-1)-self.goalcelly)*((self.currentCelly-1)-self.goalcelly))+1)
		
		#put this values into a list
		L = [right,upright,up,upleft,left,downleft,down,downright]
		#find out which one of the eight neighbors has the highest value, which means has the shortest distance to the goal cell
		for index, item in enumerate(L):
			if(item>max):
					#Once find the max, assign the index to self.nextLocalGoalIndex, which will be a number from 0 to 7,0 means the neighbor cell of the max value is the right one.
					#and the upperright is 1, and it goes on like this until the lower right is 7
        			self.nextLocalGoalIndex=index
				max=item
		print "self.nextLocalGoalIndex is "
		print self.nextLocalGoalIndex




	def updateTwist(self):
		#update the Twist message published according to self.nextLocalGoalIndex which contains the index of the neighbor cell with the largest value
		travelTime=0
		#translate the cuurent orientation of the Turtlebot from Quaternion to Euler angles in radians
		currentAngle=self.qtoeuler(self.currentPose)
		print "currentAngle ="
		print math.degrees(currentAngle)
		nextLocalGoalOrientation=math.radians(90)	
			#a.	First, get the current pose of Turtlebot and extract the x1,y1 coordinate and orientation (presented as ).
			#b.	Get the center x2,y2 coordinate of the center point in the local goal cell. For
			#c.	Calculate the distance between the (x1,y1) and (x2,y2).
			#d.	Convert the Quartenion to Euler angle, so we get the orientation of (x1,y1) and (x2,y2) in Euler angle formatting as YAW1 and YAW2
			#e.	Calculate the angle difference between YAW1 and YAW2
			#f.	Set the linear.x to be 0.2
			#g.	Calculate the how much time it will take to allow Turtlebot to travel through the distance between the (x1,y1) and (x2,y2), here we annotate it to be T
			#h.	Having known the T and YAW1 and YAW2, we set angular.z =(YAW1-YAW2)/T. In this way, we assume that when the Turtlebot travels through the required distance, it will turn the angle of (YAW1-YAW2).
		if(self.nextLocalGoalIndex==0):
			#if the local goal cell is the right one, we need to determine the angular.z for the Twist message.
			self.vx=0.2
			a=self.currentPose.pose.pose.position.x-((self.currentCellx+1)*self.stepsize-self.stepsize/2)
			b=self.currentPose.pose.pose.position.y-((self.currentCelly)*self.stepsize-self.stepsize/2)
			distance=math.sqrt(a*a+b*b)
			travelTime=distance/self.vx
			self.az=-(currentAngle-0)/travelTime

		elif(self.nextLocalGoalIndex==1):
			if(currentAngle>math.radians(-135)):
				self.vx=0.2
				a=self.currentPose.pose.pose.position.x-((self.currentCellx+1)*self.stepsize-self.stepsize/2)
				b=self.currentPose.pose.pose.position.y-((self.currentCelly+1)*self.stepsize-self.stepsize/2)
				distance=math.sqrt(a*a+b*b)
				travelTime=distance/self.vx
				self.az=-(currentAngle-math.radians(45))/travelTime
			else:
				self.vx=0.2
				a=self.currentPose.pose.pose.position.x-((self.currentCellx+1)*self.stepsize-self.stepsize/2)
				b=self.currentPose.pose.pose.position.y-((self.currentCelly+1)*self.stepsize-self.stepsize/2)
				distance=math.sqrt(a*a+b*b)
				travelTime=distance/self.vx
				self.az=-(currentAngle+math.pi+math.pi-math.radians(45))/travelTime

		
		elif(self.nextLocalGoalIndex==2):
			if(self.alignCell==1):
				# if self.alignCell =1 and self.nextLocalGoalIndex==2, that means the Turtlebot is now on the same column as the goal cell, so we need to stop moving forwards and starting turning to face the goal
				if(currentAngle>math.radians(-90)):
					# if the current orientation is greater than -90, we set the angular.z to be positive, so it's more time efficient
					self.vx=0.2
					self.az=0.0
					azt=-(currentAngle-math.radians(90))/2
					self.accurateAngle(self,-(currentAngle-math.radians(90)),azt)
				else:
					# if the current orientation is less than -90, we set the angular.z to be positive, so it's more time efficient
					self.vx=0.2
					self.az=0.0
					azt=-(math.pi-math.radians(90)+math.pi+currentAngle)/2
					#call the accurateAngle function to turn an accurtae angle.
					self.accurateAngle(self,-(math.pi-math.radians(90)+math.pi+currentAngle),azt)
			else:
				if(currentAngle>math.radians(-90)):
					self.vx=0.2
					a=self.currentPose.pose.pose.position.x-((self.currentCellx)*self.stepsize-self.stepsize/2)
					b=self.currentPose.pose.pose.position.y-((self.currentCelly+1)*self.stepsize-self.stepsize/2)
					distance=math.sqrt(a*a+b*b)
					travelTime=distance/self.vx
					self.az=-(currentAngle-math.radians(90))/travelTime
				else:
					self.vx=0.2
					a=self.currentPose.pose.pose.position.x-((self.currentCellx)*self.stepsize-self.stepsize/2)
					b=self.currentPose.pose.pose.position.y-((self.currentCelly+1)*self.stepsize-self.stepsize/2)
					distance=math.sqrt(a*a+b*b)
					travelTime=distance/self.vx
					self.az=-(math.pi-math.radians(90)+math.pi+currentAngle)/travelTime


		elif(self.nextLocalGoalIndex==3):
			if(currentAngle>math.radians(-45)):
				self.vx=0.2
				a=self.currentPose.pose.pose.position.x-((self.currentCellx-1)*self.stepsize-self.stepsize/2)
				b=self.currentPose.pose.pose.position.y-((self.currentCelly+1)*self.stepsize-self.stepsize/2)
				distance=math.sqrt(a*a+b*b)
				travelTime=distance/self.vx
				self.az=-(currentAngle-math.radians(135))/travelTime
			else:
				self.vx=0.2
				a=self.currentPose.pose.pose.position.x-((self.currentCellx-1)*self.stepsize-self.stepsize/2)
				b=self.currentPose.pose.pose.position.y-((self.currentCelly+1)*self.stepsize-self.stepsize/2)
				distance=math.sqrt(a*a+b*b)
				travelTime=distance/self.vx
				self.az=-(2*pi+currentAngle-math.radians(135))/travelTime
				print "haha"		

		elif(self.nextLocalGoalIndex==4):
			# if self.alignCell =1 and self.nextLocalGoalIndex==4, that means the Turtlebot is now on the same row as the goal cell, so we need to stop moving forwards and starting turning to face the goal
			if(self.alignCell==1):
				if(currentAngle>=0):
					# if the current orientation is greater than 0, we set the angular.z to be positive, so it's more time efficient
					self.vx=0.2
					self.az=0.0
					azt=(math.radians(180)-currentAngle)/2
					self.accurateAngle(self,math.radians(180)-currentAngle,azt)
				else:
					# if the current orientation is less than 0, we set the angular.z to be positive, so it's more time efficient
					self.vx=0.2
					self.az=0.0
					azt=-(math.radians(180)-abs(currentAngle))/2
					self.accurateAngle(self,-(math.radians(180)-abs(currentAngle)),azt)

			else:
				if(currentAngle>=0):
					self.vx=0.2
					a=self.currentPose.pose.pose.position.x-((self.currentCellx-1)*self.stepsize-self.stepsize/2)
					b=self.currentPose.pose.pose.position.y-((self.currentCelly)*self.stepsize-self.stepsize/2)
					distance=math.sqrt(a*a+b*b)
					travelTime=distance/self.vx
					self.az=(math.radians(180)-currentAngle)/travelTime
				else:
					self.vx=0.2
					a=self.currentPose.pose.pose.position.x-((self.currentCellx-1)*self.stepsize-self.stepsize/2)
					b=self.currentPose.pose.pose.position.y-((self.currentCelly)*self.stepsize-self.stepsize/2)
					distance=math.sqrt(a*a+b*b)
					travelTime=distance/self.vx
					self.az=-(math.radians(180)-abs(currentAngle))/travelTime


		elif(self.nextLocalGoalIndex==5):
			if(currentAngle<math.radians(45)):
				self.vx=0.2
				a=self.currentPose.pose.pose.position.x-((self.currentCellx-1)*self.stepsize-self.stepsize/2)
				b=self.currentPose.pose.pose.position.y-((self.currentCelly-1)*self.stepsize-self.stepsize/2)
				distance=math.sqrt(a*a+b*b)
				travelTime=distance/self.vx
				self.az=-(currentAngle-math.radians(-135))/travelTime
			else:
				self.vx=0.2
				a=self.currentPose.pose.pose.position.x-((self.currentCellx-1)*self.stepsize-self.stepsize/2)
				b=self.currentPose.pose.pose.position.y-((self.currentCelly-1)*self.stepsize-self.stepsize/2)
				distance=math.sqrt(a*a+b*b)
				travelTime=distance/self.vx
				self.az=(math.pi-currentAngle+math.pi+math.radians(-135))/travelTime
				print "haha"

		elif(self.nextLocalGoalIndex==6):
			# if self.alignCell =1 and self.nextLocalGoalIndex==6, that means the Turtlebot is now on the same column as the goal cell, so we need to stop moving forwards and starting turning to face the goal
			if(self.alignCell==1):
				print "align cell, adjust orientation"
				if(currentAngle<math.radians(90)):
					# if the current orientation is greater than 90, we set the angular.z to be positive, so it's more time efficient
					self.vx=0.2
					self.az=0.0
					azt=-(currentAngle-math.radians(-90))/2
					self.accurateAngle(self,-(currentAngle-math.radians(-90)),azt)
				else:
					# if the current orientation is less than 90, we set the angular.z to be positive, so it's more time efficient
					self.vx=0.2
					self.az=0.0
					azt=-(math.pi-currentAngle+math.pi+math.radians(-90))/2
					self.accurateAngle(self,-(math.pi-currentAngle+math.pi+math.radians(-90)),azt)
				
			else:
				if(currentAngle<math.radians(90)):
					self.vx=0.2
					a=self.currentPose.pose.pose.position.x-((self.currentCellx)*self.stepsize-self.stepsize/2)
					b=self.currentPose.pose.pose.position.y-((self.currentCelly-1)*self.stepsize-self.stepsize/2)
					distance=math.sqrt(a*a+b*b)
					travelTime=distance/self.vx
					self.az=-(currentAngle-math.radians(-90))/travelTime
				else:
					self.vx=0.2
					a=self.currentPose.pose.pose.position.x-((self.currentCellx)*self.stepsize-self.stepsize/2)
					b=self.currentPose.pose.pose.position.y-((self.currentCelly-1)*self.stepsize-self.stepsize/2)
					distance=math.sqrt(a*a+b*b)
					travelTime=distance/self.vx
					self.az=-(math.pi-currentAngle+math.pi+math.radians(-90))/travelTime
			

		elif(self.nextLocalGoalIndex==7):
			if(currentAngle<math.radians(135)):
				self.vx=0.2
				a=self.currentPose.pose.pose.position.x-((self.currentCellx+1)*self.stepsize-self.stepsize/2)
				b=self.currentPose.pose.pose.position.y-((self.currentCelly-1)*self.stepsize-self.stepsize/2)
				distance=math.sqrt(a*a+b*b)
				travelTime=distance/self.vx
				self.az=-(currentAngle-math.radians(-45))/travelTime
			else:
				self.vx=0.2

				a=self.currentPose.pose.pose.position.x-((self.currentCellx+1)*self.stepsize-self.stepsize/2)
				b=self.currentPose.pose.pose.position.y-((self.currentCelly-1)*self.stepsize-self.stepsize/2)
				distance=math.sqrt(a*a+b*b)
				travelTime=distance/self.vx
				self.az=-(math.pi-currentAngle+math.pi+math.radians(-45))/travelTime
				

		print "traveltime="
		print travelTime
		print "self.az="
		print math.degrees(self.az)

	def checkGoalCell(self):
		#check the starting point and which cell it locates
		gxloc=self.goalx/self.stepsize
		gyloc=self.goaly/self.stepsize
		gxcellNo=math.ceil(gxloc)
		gycellNo=math.ceil(gyloc)
		self.goalcellx=gxcellNo
		self.goalcelly=gycellNo
		print "goalx is"
		print gxloc,self.goalcellx	
		print "goaly is"
		print gyloc,self.goalcelly

	def qtoeuler(self,msg):
		#convert from Quartenion to Euler angle
		quaternion = (
    			msg.pose.pose.orientation.x,
    			msg.pose.pose.orientation.y,
    			msg.pose.pose.orientation.z,
    			msg.pose.pose.orientation.w)
		euler =transformations.euler_from_quaternion(quaternion)
		roll = euler[0]
		pitch = euler[1]
		yaw = euler[2]	
		return yaw

	def reachTarget(self):
		#check if the Turtlebot has reached the goal cell or not
		if(self.currentCellx==self.goalcellx or self.currentCelly==self.goalcelly):
			self.alignCell=1
		if(self.currentCellx==self.goalcellx and self.currentCelly==self.goalcelly):
			self.reached=True

   	def get_odom(self):
        	# Get the current transform between the odom and base frames
        	try:
			(trans, rot)  = self.tf_listener.lookupTransform(self.odom_frame, self.base_frame, rospy.Time(0))
        	except (tf.Exception, tf.ConnectivityException, tf.LookupException):
            		rospy.loginfo("TF Exception")
            		return
        	return (Point(*trans), quat_to_angle(Quaternion(*rot)))

	def accurateAngle(self,goal_angle,angular_speed):
		# turn the turtlebot to a certain angle
		angular_tolerance=math.radians(2)
		move_cmd.angular.z = angular_speed
		last_angle = rotation
		turn_angle = 0
       		while abs(turn_angle + angular_tolerance) < abs(goal_angle) and not rospy.is_shutdown():
                	# Publish the Twist message and sleep 1 cycle         
                	self.cmd_vel.publish(move_cmd)
                
                	r.sleep()
                
                	# Get the current rotation
                	(position, rotation) = self.get_odom()
                
                	# Compute the amount of rotation since the last lopp
                	delta_angle = normalize_angle(rotation - last_angle)
                
                	turn_angle += delta_angle
                	last_angle = rotation

def quat_to_angle(quat):
    		rot = PyKDL.Rotation.Quaternion(quat.x, quat.y, quat.z, quat.w)
    		return rot.GetRPY()[2]
        
def normalize_angle(angle):
    		res = angle
    		while res > pi:
        		res -= 2.0 * pi
    		while res < -pi:
        		res += 2.0 * pi
    		return res
rospy.init_node('moveG')
tdmap = myMoveG(12.4,8.38)
tdmap.start()	

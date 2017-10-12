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
		#update the currentCell\
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
		while not rospy.is_shutdown():
			
			cmd=Twist()
			cmd.linear.x = self.vx
			cmd.angular.z = self.az
			self.cmd_vel.publish(cmd)
	    		# wait for 0.1 seconds (10 HZ) and publish again
            		rate.sleep()
			self.reachTarget()	
			while(self.reached==True):
				pass

	def checkNeighbourValue(self):
		#update the neighbor value and choose the next cell, and the next local goal is the center of the cell
		max=0
		self.nextLocalGoalIndex=0
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
		
		L = [right,upright,up,upleft,left,downleft,down,downright]
		
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
			#negative angle and negative 
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
					self.vx=0.2
					self.az=0.0
					# using time to control how much angle it turns
					azt=-(currentAngle-math.radians(90))/2
					for i in range (0,20):
						cmdt=Twist()
						cmdt.linear.x = 0.0
						cmdt.angular.z = azt
						self.cmd_vel.publish(cmdt)
				else:
					self.vx=0.2
					self.az=0.0
					azt=-(math.pi-math.radians(90)+math.pi+currentAngle)/2
					for i in range (0,20):
						cmdt=Twist()
						cmdt.linear.x = 0.0
						cmdt.angular.z = azt
						self.cmd_vel.publish(cmdt)
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
			if(self.alignCell==1):
				if(currentAngle>=0):
					self.vx=0.2
					self.az=0.0
					azt=(math.radians(180)-currentAngle)/2
					for i in range (0,20):
						cmdt=Twist()
						cmdt.linear.x = 0.0
						cmdt.angular.z = azt
						self.cmd_vel.publish(cmdt)	
				else:
					self.vx=0.2
					self.az=0.0
					azt=-(math.radians(180)-abs(currentAngle))/2
					for i in range (0,20):
						cmdt=Twist()
						cmdt.linear.x = 0.0
						cmdt.angular.z = azt
						self.cmd_vel.publish(cmdt)
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
				self.az=math.pi-currentAngle+math.pi+math.radians(-135)/travelTime
				print "haha"

		elif(self.nextLocalGoalIndex==6):
			if(self.alignCell==1):
				if(currentAngle<math.radians(90)):
					self.vx=0.2
					self.az=0.0
					azt=-(currentAngle-math.radians(-90))/2
					for i in range (0,20):
						cmdt=Twist()
						cmdt.linear.x = 0.0
						cmdt.angular.z = azt
						self.cmd_vel.publish(cmdt)
				else:
					self.vx=0.2
					self.az=0.0
					azt=-(math.pi-currentAngle+math.pi+math.radians(-90))/2
					for i in range (0,20):
						cmdt=Twist()
						cmdt.linear.x = 0.0
						cmdt.angular.z = azt
						self.cmd_vel.publish(cmdt)
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
		gxloc=self.goalx/self.stepsize
		gyloc=self.goaly/self.stepsize
		gxcellNo=math.ceil(gxloc)
		gycellNo=math.ceil(gyloc)
		# in this cell
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
		#yaw: x to y
		roll = euler[0]
		pitch = euler[1]
		yaw = euler[2]	
		#print "euler"
		#print euler
		return yaw

	def reachTarget(self):
		if(self.currentCellx==self.goalcellx or self.currentCelly==self.goalcelly):
			self.alignCell=1
		if(self.currentCellx==self.goalcellx and self.currentCelly==self.goalcelly):
			self.reached=True

	
rospy.init_node('moveG')
tdmap = myMoveG(12.3,8.03)
tdmap.start()	

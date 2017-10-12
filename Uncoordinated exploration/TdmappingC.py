#!/usr/bin/env python
import roslib
import rospy
import Queue
import numpy as np
import math
from nav_msgs.msg import *
from std_msgs.msg import *
from sensor_msgs.msg import Image, LaserScan
import pickle
from adhoc_communication.srv import *
from nav_msgs.msg import OccupancyGrid, Odometry
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
import actionlib
from actionlib_msgs.msg import *
from geometry_msgs.msg import Pose, Point, Quaternion
from rospy.numpy_msg import numpy_msg
from go_to_specific_point_on_map import *
from tf import *
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
class Tdmapping:
	def __init__(self):
	# subsrcibe to the nav_msgs/OccupancyGrid nav_msgs/MapMetaData topic
	# Or call the service to dynamic_map
	#3d map data
	# OccupancyHistogram is made of original message plus the encoded
		rospy.Subscriber("/move_base/local_costmap/costmap", OccupancyGrid, self.generateMap) 
		rospy.Subscriber("/my3dMap",OccupancyGrid, self.processMap)
		rospy.Subscriber("amcl_pose", PoseWithCovarianceStamped, self.poseCallback)

		#variables
		self.resolution=0.05
       	 	self.tmp = OccupancyGrid()    
		self.encodedMap = OccupancyGrid()
		self.navigationMap = OccupancyGrid() 
		self.previousMsg=OccupancyGrid()
		self.selfMap=OccupancyGrid()    
       		self.tmp.header.frame_id = '/odom'
        	self.tmp.info.map_load_time = rospy.Time.now() # The time at which the map was loaded
        	self.tmp.info.resolution = self.resolution # The map resolution [m/cell]
        	self.tmp.info.width = 39 # Map width [cells]
        	self.tmp.info.height = 39  # Map height [cells]
		self.encodedMap=self.tmp
		self.navigationMap=self.tmp
		self.previoudMsg=self.tmp
		self.workQueue = Queue.Queue(2)
  		
		#info about this node
		self.id=1;
		self.name="ferdinand"
		self.neighborCount=2
		self.neighbourSet=['ferdinand','trinculo']
		self.count=0

		#flags
		self.storeDone=0
		self.processDone=0
		self.encodeDone=0
		self.moveDone=1

		#navigation goals
		self.goal=0.0
		self.goalx=0.0
		self.goaly=0.0
		self.oldpositionx=0.0
		self.oldpositiony=0.0
		self.positionx=15.5
		self.positiony=11.4
		self.rightTop=False
		self.rightBot=False
		self.leftTop=True
		self.leftBot=False
		self.goalxBound=0.0
		self.goalyBound=0.0
		# navigate tools
		self.navigator = GoToPose()
		self.currentPose=PoseWithCovarianceStamped()
	def generateMap(self,msg):
		#manipulate each data
		#msg.info is the MapMetaData
		#msg.data is one d array
		if(self.moveDone==1):
			self.selfMap=msg
			print "get a new costmap"
			#print len(msg.data)
			#self.encodedMap=msg
			for x in range(0,len(msg.data)):
			# apply the function on the tdmap.orginalMap= 
			#a=np.asarray(self.encodedMap.data)
			#b=np.asarray(msg.data)
			#self.encodedMap.data[x]=self.encodeGrid(msg.data[x])
			#a[x]=self.encodeGrid(b[x])
				self.encodedMap.data=msg.data
			self.moveDone=0
			print "encodeDone"
			self.encodeDone=1		
		
	def processMap(self,msg):
		print "getOneMsg"	
		#self.showMsg(msg)
		self.msgAccepted(msg)
	def showMsg(self,msg):
		print(msg)
			
				
	def start(self):
		#publish the 3d map
		rate = rospy.Rate(10)
		while not rospy.is_shutdown():
			# a is the message we create for 3d mapping
			if(self.storeDone):
				print "store all msg done,begin processing"
				self.processMsg(self.workQueue)
				self.storeDone=0
				
			if(self.encodeDone):
				self.broadcast(self.encodedMap)

				self.encodeDone=0
				self.encodedMap=self.tmp
				print "sendDone"
				#self.clearAll();	
					
			if(self.processDone):
				self.processDone=0
				self.MoveToTarget()
				self.navigationMap=self.tmp
					
				
				#self.clearAll();
					
			rate.sleep()

	#def clearAll(self):
		#clear all the variables
				
	def MoveToTarget(self):
		#go to specific point of the map
		self.findTarget()
		try:
       			#navigator = GoToPose()
        		# Customize the following values so they are appropriate for your location
        		position = {'x': self.goalx, 'y' : self.goaly}
        		quaternion = {'r1' : 0.000, 'r2' : 0.000, 'r3' : 0.000, 'r4' : 1.000}

       			rospy.loginfo("Go to (%s, %s) pose", position['x'], position['y'])
        		success = self.navigator.goto(position, quaternion)

       			if success:
            			rospy.loginfo("reached the desired pose")
				#self.findTarget()
				#self.processDone=1;
				
        		else:
            			rospy.loginfo("The base failed to reach the desired pose")
				rospy.loginfo("Trying to go back")
				self.goBack()
				self.positionx=self.oldpositionx
				self.positiony=self.oldpositiony
				if(self.rightTop==True):
					self.rightTop=not self.rightTop
					self.leftTop=not self.leftTop
				elif(self.leftTop==True):
					self.leftTop=not self.leftTop
					self.leftBot=not self.leftBot
				elif(self.leftBot==True):
					self.leftBot=not self.leftBot
					self.rightBot=not self.rightBot
				elif(self.rightBot==True):
					self.rightBot=not self.rightBot
					self.rightTop=not self.rightTop
				else:
					print "ran out of option"
				#self.findTarget()
				#self.processDone=1				
		
       		# Sleep to give the last log messages time to be sent
	       		rospy.sleep(1)

    		except rospy.ROSInterruptException:
   	    		rospy.loginfo("Ctrl-C caught. Quitting")	
		self.moveDone=1
	
	def broadcast(self,a):
		#send to all nodes at a certain rate
		self.mySend3dMapFunction("ferdinand", "my3dMap", a)
		#mySend3dMapFunction("gonzalo", "my3dmap", a)
		#mySend3dMapFunction("trinculo", "my3dmap", a)

		
		
	def encodeGrid(self,dat):
		encodedCell = 0;
		encodedCell = dat*0.5
		return encodedCell
	
	def msgAccepted(self,msg):
		if(self.storeDone==0):
			self.storeMsg(msg)
			

			
	# define one thread to store the msg.thread2
	def storeMsg(self,msg):
		if(self.workQueue.empty()):
			print "empty workqueue"
			self.previousMsg=msg
			self.workQueue.put(msg)
		if (msg!=self.previousMsg):
			print "normal processing"
			self.previousMsg=msg;
			self.workQueue.put(msg)
			self.count=self.count+1
		if(self.workQueue.full()):
			print "full workqueue"
			self.storeDone=1
		print self.workQueue
		print "count="
		print self.count
		print "storeOneMsgDone"
		
	
	
	# whenever a msg come in, we will see if it matches to determine to process it or not
	def processMsg(self,msgQueue):
		msg=OccupancyGrid()
		self.navigationMap=self.selfMap
		if not msgQueue.empty():
			print "not empty workqueue"
			msg=msgQueue.get()
			self.integrate(msg)			
		else:
			print "process done"
		self.processDone=1
		
		# process is done
					
		
	def integrate(self,msg):
		#self.navigationMap=msg
		#self.navigationMap.data=self.navigationMap.data*msg
		print "integrate done"
	
	#find targets to send 
	def findTarget(self):
		min=100;
		currentAngle=self.qtoeuler(self.currentPose)
		distance=0
		angletoaxis=0
		angletouse=0
		for x in range(0,len(self.navigationMap.data)):
			# apply the function on the tdmap.orginalMap= 
			
			#self.encodedMap.data[x]=self.encodeGrid(msg.data[x])
			#a[x]=b[x]
			if(self.navigationMap.data[x]<=min):
				min=self.navigationMap.data[x]
				#self.goal=x
				if(self.rightTop==True):
					#print "go rightTop"
					self.goalxBound=(x%39.0)/39.0*39.0*0.05-1.975/2
					self.goalyBound=(math.floor(x/39.0))/39.0*39.0*0.05-0.9875
					#print self.goalxBound,self.goalyBound
					if(0<self.goalxBound<1 and 0<self.goalyBound<1):
						#print "in the loop"
						distance=math.sqrt(self.goalxBound*self.goalxBound+self.goalyBound*self.goalyBound)
						angletoaxis=math.pi/2-math.atan2(self.goalyBound,self.goalxBound)
						angletouse=(math.pi/2-angletoaxis)-(math.pi/2-currentAngle)
						self.goalx=self.positionx+math.cos(angletouse)*distance
						self.goaly=self.positiony+math.sin(angletouse)*distance
						self.goal=x
				elif(self.leftTop==True):
					#print "go leftTop"
					self.goalxBound=(x%39.0)/39.0*39.0*0.05-1.975/2
					self.goalyBound=(math.floor(x/39.0))/39.0*39.0*0.05-0.9875
					if(-0.9875<self.goalxBound<0 and 0<self.goalyBound<0.9875):
						distance=math.sqrt(self.goalxBound*self.goalxBound+self.goalyBound*self.goalyBound)
						angletoaxis=-(math.pi/2-math.atan2(self.goalyBound,self.goalxBound))
						angletouse=math.atan2(self.goalyBound,self.goalxBound)-(currentAngle-math.pi/2)
						self.goalx=self.positionx+math.cos(angletouse)*distance
						self.goaly=self.positiony+math.sin(angletouse)*distance
						self.goal=x
				elif(self.leftBot==True):
					#print "go leftBot"
					self.goalxBound=(x%39.0)/39.0*39.0*0.05-1.975/2
					self.goalyBound=(math.floor(x/39.0))/39.0*39.0*0.05-0.9875
					if(-1.975/2<self.goalxBound<0 and -1.975/2<self.goalyBound<0):
						distance=math.sqrt(self.goalxBound*self.goalxBound+self.goalyBound*self.goalyBound)
						angletoaxis=math.pi/2-math.atan2(self.goalyBound,self.goalxBound)
						angletouse=(math.pi/2-angletoaxis)+(math.pi/2-currentAngle)
						self.goalx=self.positionx+math.cos(angletouse)*distance
						self.goaly=self.positiony+math.sin(angletouse)*distance
						self.goal=x
				elif(self.rightBot==True):
					#print "go rightBot"
					self.goalxBound=(x%39.0)/39.0*39.0*0.05-1.975/2
					self.goalyBound=(math.floor(x/39.0))/39.0*39.0*0.05-0.9875
					if(0<self.goalxBound<1.975/2 and -1.975/2<self.goalyBound<0):
						distance=math.sqrt(self.goalxBound*self.goalxBound+self.goalyBound*self.goalyBound)
						angletoaxis=-(math.pi/2-math.atan2(self.goalyBound,self.goalxBound))
						angletouse=math.atan2(self.goalyBound,self.goalxBound)+(currentAngle-math.pi/2)
						self.goalx=self.positionx+math.cos(angletouse)*distance
						self.goaly=self.positiony+math.sin(angletouse)*distance
						self.goal=x
				else:
					print "nope"
		print self.rightTop,self.leftTop,self.leftBot,self.rightBot		
		print "goalx is "
		print self.goalx
		print "goaly is "
		print self.goaly
		print "goal is"
		print self.goal
			#self.goalx=(self.goal%79.0)/79.0*79.0*0.05-1.975
			#self.goaly=(self.goal/79.0)/79.0*79.0*0.05-1.975
			#self.encodedMap.data=msg.data	
		self.oldpositionx=self.positionx
		self.oldpositiony=self.positiony
		self.positionx=self.goalx
		self.positiony=self.goaly
			
	
	
	# send to the target
	def mySend3dMapFunction(self,dst_robot, topic, data):
   	#rospy.wait_for_service('/adhoc_communication/send_string')
    		try:
			#add the send_3dmap service
        		send_OccupancyGrid = rospy.ServiceProxy('/adhoc_communication/send_map', SendOccupancyGrid)
       			resp1 = send_OccupancyGrid(dst_robot, topic, data)
        		return resp1.status
   		except rospy.ServiceException, e:
        		print "Service call failed: %s"%e


	def goBack(self):
		position = {'x': self.oldpositionx, 'y' : self.oldpositiony}
        	quaternion = {'r1' : 0.000, 'r2' : 0.000, 'r3' : 0.000, 'r4' : 1.000}

       		rospy.loginfo("Go to (%s, %s) pose", position['x'], position['y'])
        	success = self.navigator.goto(position, quaternion)
		if success:

            			rospy.loginfo("go back successfully")
				
        	else:
            			rospy.loginfo("goback fail")		
	

	def poseCallback(self,msg):
		self.currentPose=msg


	def qtoeuler(self,msg):

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
	
rospy.init_node('tdmapping')
tdmap = Tdmapping()
tdmap.processDone=1
#tdm.navigationMap=self.selfMap
tdmap.start()

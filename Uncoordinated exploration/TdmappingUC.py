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
from geometry_msgs.msg import Pose, Point, Quaternion,Twist
from rospy.numpy_msg import numpy_msg
from go_to_specific_point_on_map import *
from goforward_and_avoid_obstacle import *
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
import actionlib
from actionlib_msgs.msg import *
#uncoordinated and just go forward
class Tdmapping:
	def __init__(self):
	# subsrcibe to the nav_msgs/OccupancyGrid nav_msgs/MapMetaData topic
	# Or call the service to dynamic_map
	#3d map data
	# OccupancyHistogram is made of original message plus the encoded
		rospy.Subscriber("/move_base/local_costmap/costmap", OccupancyGrid, self.generateMap) 
		rospy.Subscriber("/my3dMap",OccupancyGrid, self.processMap)
		self.pub=rospy.Publisher("/mobile_base/commands/velocity",Twist, queue_size=10)
        	self.cmd_vel = rospy.Publisher('cmd_vel_mux/input/navi', Twist, queue_size=10)
	
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
		self.correct=0
		self.pubDone=0
		self.forwardDone=0

		#navigation goals
		self.goal=0.0
		self.goalx=0.0
		self.goaly=0.0
		self.oldpositionx=0.0
		self.oldpositiony=0.0
		self.positionx=15.5
		self.positiony=8.48
		self.rightTop=False
		self.rightBot=True
		self.leftTop=False
		self.leftBot=False
		self.goalxBound=0.0
		self.goalyBound=0.0
		self.angle=0.0
		self.distance=0.0
		# navigate tools
		self.navigator = GoToPose()
	def generateMap(self,msg):
		#manipulate each data
		#msg.info is the MapMetaData
		#msg.data is one d array
		if(self.moveDone==1):
			self.selfMap=msg
			print "get a new costmap"

			for x in range(0,len(msg.data)):
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
				#self.processDone=1
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
		print "now move"
		try:
			self.goForward(self.distance)

	       		rospy.sleep(1)

    		except rospy.ROSInterruptException:
   	    		rospy.loginfo("Ctrl-C caught. Quitting")	
		#self.moveDone=1
	
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
		maxinterestValue=-1;
		maxPoint=0;
		for x in range(0,len(self.navigationMap.data)):
			if(self.navigationMap.data[x]==100):
				interestValue=self.navigationMap.data[x-1]+self.navigationMap.data[x+1]+self.navigationMap.data[x]+self.navigationMap.data[x]
				if(interestValue>maxinterestValue):		
					maxinterestValue=interestValue
					maxPoint=x
					
		print "max point index is "
		print maxPoint	
		self.goalxBound=(maxPoint%39.0)/39.0*39.0*0.05-1.975/2
		print "x is "
		print self.goalxBound
		self.goalyBound=(int(maxPoint/39.0))/39.0*39.0*0.05-0.9875
		print "y is"
		print self.goalyBound
		self.angle=(math.atan2(self.goalyBound,self.goalxBound))
		print "original angle="
		print self.angle
		print math.degrees(self.angle)
		
		
		self.distance=math.sqrt(self.goalxBound*self.goalxBound+self.goalyBound*self.goalyBound)
		print "angle is "
		print self.angle
		print "distance is "
		print self.distance
		print "correcting pose"
		self.correctPose()
		#while(self.correct==0):
			#pass
		
		
		
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
	
	def correctPose(self):
		#rotate until the distance is the shortest,using PID controller
   		delay=0.0
		twist=Twist()
		if(0<self.angle<math.pi):
			twist.linear.x=0
			twist.angular.z = math.radians(30)
			delay=self.angle/math.radians(30)	
			print "corrected angle="
			print self.angle
			print math.degrees(self.angle)
		else:
			twist.linear.x=0
			twist.angular.z = math.radians(-30)
 			delay=(abs(self.angle))/math.radians(30)
			print "my corrected angle="
			print abs(self.angle)
			print math.degrees(abs(self.angle))
		print "delay ="
		print delay
		#rospy.Timer(rospy.Duration(delay),self.turncall,oneshot=True)
		r=rospy.Rate(10)
		#while(self.pubDone==0):
		count=2*delay/0.1
		print "count"
		print count
		for i in range(0,int(math.floor(count))):
			self.cmd_vel.publish(twist)
			r.sleep()
		#twist.angular.z = 0
   		#self.pub.publish(twist)
				
	def turncall(self,event):
		self.correct=1
		self.pubDone=1
		print "simple"

	def gocall(self,event):
		self.forwardDone=1
		print "simple go"
  		
	def goForward(self,distance):

        	move_cmd = Twist()
		# let's go forward at 0.2 m/s
       		move_cmd.linear.x = 0.2
		# let's turn at 0 radians/s
		move_cmd.angular.z = 0
		rospy.Timer(rospy.Duration(distance/0.2),self.gocall,oneshot=True)
		while (self.forwardDone==0):
			self.cmd_vel.publish(move_cmd)
rospy.init_node('tdmapping')
tdmap = Tdmapping()
tdmap.processDone=1

tdmap.start()

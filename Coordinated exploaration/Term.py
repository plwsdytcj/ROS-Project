#!/usr/bin/env python
import subprocess
import os
from time import *
import sys
import socket
import fcntl
import struct
    
#def termcmd(cmd):
#    print output

def get_ip_address(ifname):
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    return socket.inet_ntoa(fcntl.ioctl(
        s.fileno(),
        0x8915,  # SIOCGIFADDR
        struct.pack('256s', ifname[:15])
    )[20:24])

IP = get_ip_address('wlan0')

robot = "trinculo"
user = "turtlebot"
password = "turtlebot"
expmaster = "export ROS_MASTER_URI=http://%s:11311" % robot
expIP = "export ROS_IP=%s" % IP

def setenv():
        cmd(expmaster)
        cmd(expIP)


def ek(string): #enter key, note that this should only be one character unless if is a special character like "space"
        subprocess.Popen(["xdotool", "key", string])
        sleep(.1)

def newtab():
        ek("ctrl+shift+t")
        sleep(4)

def cmd(inputstr):
        for n in range(0, len(inputstr)):
                if inputstr[n] == " ":
                        ek("space")
                elif inputstr[n] == ".":
                        ek("period")
                elif inputstr[n] == "_":
                        ek("underscore")
                elif inputstr[n] == "&":
                        ek("ampersand")
                elif inputstr[n] == "=":
                        ek("equal")
                elif inputstr[n] == ":":
                        ek("colon")
                elif inputstr[n] == "/":
                        ek("slash")
                elif inputstr[n] == "-":
                        ek("minus")
                elif inputstr[n] == "+":
                        ek("plus")
                elif inputstr[n] == "~":
                        ek("asciitilde")
                
                else:
                        ek(inputstr[n])
        ek("Return")
        sleep(1)

def olsrd():
        newtab()
        cmd("sudo olsrd -i wlan0")
        sleep(1)
        cmd(password)
def rosip():
        cmd("export ROS_IP=" + IP)
        
def minimal(ip): #does core, waits a bit, then does minimal.
        #this way we can do other things without waiting for minimal to finish,
        #since roscore is much faster
        #if ip is 1 then it sets the ip before hand
        newtab()
        if ip == 1:
                rosip()
        cmd("roscore")
        newtab()
        if ip == 1:
                rosip()
        sleep(5)
        cmd("roslaunch turtlebot_bringup minimal.launch")



def adhoc():
        newtab()
        cmd("sudo su")
        cmd(password)
        cmd("cd /home/"+user+"/catkin_ws")
        cmd("source ./devel/setup.bash")
        sleep(5)
        cmd("cd build")
        rosip()
        cmd("sudo chown root adhoc_communication")
        cmd("sudo chmod +s adhoc_communication")
        cmd("rosrun adhoc_communication adhoc_communication")
        
def ros_mesh():
        olsrd()
        minimal(1)
        adhoc()
    

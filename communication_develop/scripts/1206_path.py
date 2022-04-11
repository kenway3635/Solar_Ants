#!/usr/bin/python3
import rospy
import paho.mqtt.client as mqtt
from std_msgs.msg import String , Int16 , Float32
from geometry_msgs.msg import Twist
import time
global cmd ,move_dict,path_dict
panel_length =2
velo = 0.15
line_time = panel_length / velo
Turn_time = 0.3
cmd = Twist()
move_dict = ["w","s","a","d","q"]
path_dict = ["spath",'epath']
def velocity_cmd(command):
    
    rospy.loginfo(type(command.data))
    if command.data == "w":
        cmd.linear.x += 0.02
        #cmd.angular.z = 0
    elif command.data == "s":
        cmd.linear.x -= 0.02
        #cmd.angular.z = 0
    elif command.data == "q":
        cmd.linear.x =  0
        cmd.angular.z = 0
    elif command.data == "a":
        cmd.angular.z -= 0.05
    elif command.data == "d":
        cmd.angular.z  += 0.05
    else :
        pass
    return cmd
def pathGO(pathKind):
    #s path 
    """
    if pathKind == "spath":
        while stop != True:
            cmd.linear.x = 1
            time.sleep(line_time)
            cmd.linear.x=0
            cmd.angular.z=1
            time.sleep(Turn_time)
            cmd.angular.z = 0
            cmd.linear.x=1
    """
            
def callback(data):
    move_dict = ["w","s","a","d","q"]
    rospy.loginfo("msg from mqtt %s",data)
    if data.data in move_dict: 
        pub_cmd = velocity_cmd(data)
        pub.publish(pub_cmd)
    elif data.data in path_dict:
        pass

def receive():
    rospy.Subscriber('/mqtt2ros',String,callback)
    rospy.spin()


rospy.init_node('mqtt2ros', anonymous=True)
pub = rospy.Publisher('/cmd_vel',Twist,queue_size=10)
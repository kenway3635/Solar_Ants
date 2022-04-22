#!/usr/bin/python3
from typing import Dict
import rospy
import paho.mqtt.client as mqtt
import os
from std_msgs.msg import String 
from geometry_msgs.msg import Twist
# 
global cmd
cmd = Twist()

def velocity_cmd(command):
    # 由callback呼叫,輸入轉換成dict的cmd_vel ( from Node-red )
    rospy.loginfo("linear=%f,angular=%f",command[0],command[1])
    rospy.loginfo(type(command[1]))
    cmd.linear.x = command[0]  
    cmd.angular.z = command[1]
    #send out the Twist.message
    #cmd.linear.x = 1
    return cmd


def CMDcallback(data):
    
    rospy.loginfo("msg from mqtt %s",data)
    # 把收到的mqtt data轉換成dict
    receive_data = eval(data.data)
    #rospy.loginfo("data =  %s",receive_data)
    #rospy.loginfo("data type is %s",type(receive_data))
    pub_cmd = velocity_cmd(receive_data)
    pub.publish(pub_cmd)

def OSCallback(OScommand):
    rospy.loginfo("Receive OS command %s!",OScommand.data)
    if OScommand.data == "reboot":
        os.system("sudo reboot")
        rospy.loginfo("receive reboot command !")
    elif OScommand.data == "shutdown":
        os.system("sudo shutdown now")
        rospy.loginfo("receive shutdown command !")

def receive():
    rospy.Subscriber('/mqtt2ros',String,CMDcallback)
    rospy.Subscriber("/OS",String,OSCallback)
    rospy.spin()

rospy.init_node('mqtt2ros', anonymous=True)
pub = rospy.Publisher('/new_cmd_vel',Twist,queue_size=3)

receive()

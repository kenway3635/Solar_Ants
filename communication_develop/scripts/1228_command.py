#!/usr/bin/python3
from typing import Dict
import rospy
import paho.mqtt.client as mqtt
import os ,time
from std_msgs.msg import String,Int64
#from geometry_msgs.msg import Twist
# 
#global cmd
#cmd = Twist()

'''
1214 version 
現在由 node-red畫面直接送出JSON object,
by stringify() ,直接送到robot的cmd_vel ,故此段棄置

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
'''
def OSCallback(OScommand):
    rospy.loginfo("Receive OS command %s!",OScommand.data)
    if OScommand.data == "reboot":
        os.system("sudo reboot")
        rospy.loginfo("receive reboot command !")
    elif OScommand.data == "shutdown":
        os.system("sudo shutdown now")
        rospy.loginfo("receive shutdown command !")
def ConnectionState(timeFromMQTT):
    rospy.loginfo(timeFromMQTT.data)
    payload = int(round(time.time()*1000)  - timeFromMQTT.data)
    pubConnect.publish(payload)

def receive():
    #rospy.Subscriber('/mqtt2ros',String,CMDcallback)
    rospy.Subscriber("/connection_stateSub",Int64,ConnectionState)
    rospy.Subscriber("/OS",String,OSCallback)
    rospy.spin()



rospy.init_node('mqtt2ros', anonymous=True)
#pub = rospy.Publisher('/cmd_vel',Twist,queue_size=10)
pubConnect = rospy.Publisher("/connection_statePub",Int64,queue_size=10)

receive()


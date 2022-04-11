#!/usr/bin/python3
import rospy
import paho.mqtt.client as mqtt
from std_msgs.msg import String , Int16 , Float32
from geometry_msgs.msg import Twist
'''設定接聽的mqtt topic
mqtt的topic利用bridge被轉換成rostopic 
這邊訂閱經過轉換的rostopic, 處理命令後轉發到/cmd_vel'''

""" launch 檔案內的mqtt bridge會自動訂閱 mqtt2ros (MQTT topic) 
轉換成 /mqtt2ros ( ROS topic )而這份py檔去收/mqtt2ros 的訊息後轉送到cmd/vel"""
global cmd

cmd = Twist()

#def on_connect(client,userdata,flasg,rc):
#    print("on connect,code",str(rc))
#    AMR_client.subscribe("mqtt2ros")


#AMR_client = mqtt.Client()
#AMR_client.on_connect = on_connect

def velocity_cmd(command):
    
    rospy.loginfo(type(command.data))
    if command.data == "w":
        cmd.linear.x += 0.01
        #cmd.angular.z = 0
    elif command.data == "s":
        cmd.linear.x -= 0.01
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


def callback(data):

    rospy.loginfo("msg from mqtt %s",data)
    pub_cmd = velocity_cmd(data)
    pub.publish(pub_cmd)


def receive():
    rospy.Subscriber('/mqtt2ros',String,callback)
    rospy.spin()


rospy.init_node('mqtt2ros', anonymous=True)
pub = rospy.Publisher('/cmd_vel',Twist,queue_size=10)
'''
try: 
    AMR_client.connect("127.0.0.1",1883,60)
    receive()
    AMR_client.loop_forever()
#rospy.Subscriber('mqtt2ros',String,callback)
#cmd = Twist()
except:
    print("connect fail")
'''
receive()

import rospy
from std_msgs.msg import String 
from geometry_msgs.msg import Twist
'''設定接聽的mqtt topic
mqtt的topic利用bridge被轉換成rostopic 
這邊訂閱經過轉換的rostopic, 處理命令後轉發到/cmd_vel'''

def callback(data):
    rospy.loginfo("msg from mqtt %s",data)


rospy.init_node('mqtt2ros', anonymous=True)
pub = rospy.Publisher('/cmd_vel',Twist,queue_size=1)
rospy.Subscriber('mqtt2ros',String,callback)
rate = rospy.Rate(10)
rospy.spin()
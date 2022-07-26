import rospy , json

from std_msgs.msg  import String,Int64 ,Int16 , Int32, Bool , Float32MultiArray,Float32 
from geometry_msgs.msg import Twist



def std_Parser(msg): 
    return  {"data":msg.data} 

def twist_Parser(msg): 
    return {"x":msg.linear.x,"z":msg.angular.z} 


    
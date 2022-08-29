#! /usr/bin/python3
import rospy , json
import  paho.mqtt.client as mqtt
from std_msgs.msg  import String,Int64 ,Int16 , Int32, Bool , Float32MultiArray,Float32 
from geometry_msgs.msg import Twist
import asyncio 
from collections import namedtuple 


def std_Parser(msg): 
    return  {"data":msg.data} 

def twist_Parser(msg): 
    return {"x":msg.linear.x,"z":msg.angular.z} 

    
class MongoBug_MQTT(mqtt.Client): 
    latch ={} 
    dataClass = {
        "String":String , "Int64":Int64 , "Int16":Int16 , "Int32":Int32 , "Twist":Twist,"Bool":Bool ,
        "Float32MultiArray":Float32MultiArray, "Float32":Float32
    }
    msg_type_parser ={
        "std":std_Parser , "twist":twist_Parser
    }

    def __init__(self,setting_file_path="Solar_setting.json", client_id="", clean_session=1, userdata=None, 
                 protocol=mqtt.MQTTv311, transport="tcp", reconnect_on_failure=True): 
        self.setting_file = setting_file_path 
        super().__init__(client_id, clean_session, userdata, protocol, transport, reconnect_on_failure)
        self._setting() 

    def _setting(self): 
        with open (self.setting_file ,"r") as file : 
            file = json.load(file) 
            self.IP = file["Server"]
            self.MQTTport = file["MQTTport"]
            self.username = file["username"] if file["username"] else None
            self.pwd = file["pwd"] if file["pwd"] else None 
            self.ROSTOPIC = file["ROS_TOPIC_list"]
        self.callbackArg = namedtuple("callbackArg",["topic","msgType"])
        
        self.reconnect_delay_set(min_delay=1,max_delay=180) 
        
    def irPub(self,msg,arg): 
        #rospy.loginfo(arg)
        self.publish(arg,json.dumps(msg.data),qos=0)
    
    def linePub(self,msg):
        self.publish("/line",json.dumps(msg.data),qos=0)
        rospy.loginfo("pub line")
        pass 
    #main function for ros thread
    def _reigister(self): 
        rospy.init_node("Ros_mongo",anonymous=True)
        self.connect(self.IP,self.MQTTport,80)
        #self.connect("localhost",1883,60)
        rospy.loginfo("mqtt connection success")
        rospy.Subscriber("/front_left_ir",Bool,self.irPub,callback_args=("/front_left_ir"))
        rospy.Subscriber("/back_right_ir",Bool,self.irPub,callback_args=("/back_right_ir"))
        rospy.Subscriber("/back_left_ir",Bool,self.irPub,callback_args=("/back_left_ir"))
        rospy.Subscriber("/front_right_ir",Bool,self.irPub,callback_args=("/front_right_ir"))
        rospy.Subscriber("/line",Bool,self.linePub)
        for topic in self.ROSTOPIC : 
            rospy.loginfo(f"Subscribe ROS topic : {topic}")
            
            self.latch.update({topic["Topic_name"]:None})
            
            rospy.Subscriber(
                topic["Topic_name"] , 
                self.dataClass[topic["data_class"]] , 
                #callback = self._mqttPublish,
                callback = self._updateLatch , 
                callback_args=self.callbackArg(topic["Topic_name"],topic["msg_type"]),
                queue_size=topic["quene_size"]
            )
        rospy.loginfo("DataBase register Done! ")
        rospy.Timer(rospy.Duration(2),self.mqttPublish)
        
        rospy.spin()


    # def irPub(self,msg,arg):
    #     self.publish(arg,json.dumps(msg.data),qos=1 )
   
    
    def _updateLatch(self,msg,arg):
        self.latch[arg.topic] = self.msg_type_parser[arg.msgType](msg)
        
    def mqttPublish(self,*args): 
        
        for key,item in self.latch.items(): 
            if item:  
                self.publish(key,json.dumps(item),qos=0)
            else: 
                pass 
            
    # async def asycPublish(self):
    #     print("do")
    #     await asyncio.sleep(2) 

if __name__ == "__main__": 
    
    setting_file = rospy.get_param("/json_file",None)
    #setting_file =None
    DataBaseClient = MongoBug_MQTT(setting_file_path=setting_file) if setting_file else MongoBug_MQTT() 
    DataBaseClient._reigister() 

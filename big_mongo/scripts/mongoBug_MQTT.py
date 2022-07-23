import rospy , json
import  paho.mqtt.client as mqtt
from std_msgs.msg  import String,Int64 ,Int16 , Int32, Bool , Float32MultiArray,Float32 
from geometry_msgs.msg import Twist
from ROS_msgParser import * 
from collections import namedtuple 


class MongoBug_MQTT(mqtt.Client): 
    
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
        
        
    #main function for ros thread
    def _reigister(self): 
        rospy.init_node("Ros_mongo",anonymous=True)
        self.connect(self.IP,self.MQTTport,80)
        #self.connect("localhost",1883,60)
        rospy.loginfo("mqtt connection success")
        
        for topic in self.ROSTOPIC : 
            rospy.loginfo(f"Subscribe ROS topic : {topic}")

            rospy.Subscriber(
                topic["Topic_name"] , 
                self.dataClass[topic["data_class"]] , 
                callback = self._mqttPublish,
                #callback_args=(topic["Topic_name"],topic["msg_type"]), 
                callback_args=self.callbackArg(topic["Topic_name"],topic["msg_type"]),
                queue_size=topic["quene_size"]
            )
        rospy.loginfo("DataBase register Done! ")
        rospy.spin()

    
    def _mqttPublish(self,msg,arg): 
        #msg = self.msg_type_parser[arg[1]](msg) 
        #self.publish(arg[0],json.dumps(msg),qos=0) 
        #print(f"publish {arg.topic}")
        msg = self.msg_type_parser[arg.msgType](msg)
        self.publish(arg.topic,json.dumps(msg),qos=0) 
    

if __name__ == "__main__": 
    
    #setting_file = rospy.get_param("/json_file",None)
    setting_file =None
    DataBaseClient = MongoBug_MQTT(setting_file_path=setting_file) if setting_file else MongoBug_MQTT() 
    DataBaseClient._reigister() 
#! /user/bin/python3
from pymongo import MongoClient
import paho.mqtt.client as mqtt 
import json 
from datetime import datetime   


class MongoServer(mqtt.Client): 
    
    def __init__(self,setting_file_path="Solar_setting.json", client_id="", clean_session=1, userdata=None, 
                protocol=mqtt.MQTTv311, transport="tcp", reconnect_on_failure=True): 
        # loading parameter
        self.setting_file = setting_file_path
        
        # create a mqtt client 
        super().__init__(client_id, clean_session, userdata, protocol, transport, reconnect_on_failure)
        self.on_connect = MongoServer._mqtt_Onconnect 
        self.on_message = MongoServer.on_message
        
        self.setting() 
        
    def on_message(self,userdata,msg):
        print(MongoServer.JsonDecode(msg) )
        
    @staticmethod 
    def JsonDecode(msg) -> dict: 
        return msg.topic , json.loads(msg.payload.decode("utf-8"))
    
    def setting(self): 
        with open(self.setting_file ,"r") as file : 
            file = json.load(file) 
            self.collection_list = file["Collection_list"]
            self.DataBaseName = file["DataBase"]
            self.MQTTport = file["MQTTport"]
            self.MQTT_topic = file["ROS_TOPIC_list"]  # MQTT&ROS use same topic name , for this script , it mean mqtt topic 
            self.IP = file["Server"]
            self.username = file["username"] if file["username"] else None
            self.pwd = file["pwd"] if file["pwd"] else None 
            
        self.Database = MongoClient(self.IP,27017,username=self.username,password=self.pwd)
        print(f"connection Database {self.DataBaseName} , {self.IP},{self.username},{self.pwd}")
        #self.Database = MongoClient("192.168.0.82",27017,username=self.username,password=self.pwd)
        self.Database_handler = {}  # create the connection to every collection 
        self.collection_reference = {}  # use to  find corresponding collection by mqtt topic
        for collection in self.collection_list : 
            self.Database_handler[collection] = self.Database[self.DataBaseName][collection]
            
        for topic in self.MQTT_topic: 
            self.collection_reference.update({topic["Topic_name"]:topic["collection"]})
            
        print(f"connect to {self.IP}")
        self.connect(self.IP,self.MQTTport,60)

    def writeDatabase(self,userdata,msg): 
        print("do")
        topic , msg = MongoServer.JsonDecode(msg) 
        msg.update({"date":datetime.now().isoformat()})
        print(f"Write a message into {topic}, {msg}")
        # write into database 
        self.Database_handler[self.collection_reference[topic]].insert_one(msg)
        
              
    def _mqtt_Onconnect(self,userdata,flags,rc): 
        print(f"Connection with result:{str(rc)}")
        for topic in self.MQTT_topic: 
            print(f"subscribe {topic} ")
            self.subscribe(topic["Topic_name"],qos=0)
            self.message_callback_add(topic["Topic_name"],MongoServer.writeDatabase)
       

if __name__ =="__main__": 
    
    setting_file = "Solar_setting.json"
    Database = MongoServer(setting_file_path=setting_file)
    Database.loop_forever()
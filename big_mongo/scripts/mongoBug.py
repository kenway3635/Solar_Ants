#-*- coding:UTF-8 -*- 
import rospy ,os ,time ,json,sys
from pymongo import MongoClient 
from std_msgs.msg import String,Int64 ,Int16 , Int32, Bool , Float32MultiArray
from geometry_msgs.msg import Twist
from big_mongo.mongoWriter import MongoWriter 
from pathlib import Path


class MongoBug() : 
    
    def __init__(self,setting_file_path="Solar_setting.json") : 

        self.DataBase = MongoClient("192.168.0.82",27017,username="root",password="kangli0306")
        self.setting_file= setting_file_path 
        self.Database_handler = {}
        self.ROS_topic = None
        self.setting() 
        self.dataClass = {
            "String":String , "Int64":Int64 , "Int16":Int16 , "Int32":Int32 , "Twist":Twist,"Bool":Bool ,
            "Float32MultiArray":Float32MultiArray
        }
        self.msg_class_forWrite = {
            "std":MongoWriter.write_std  , "geometry":MongoWriter.write_geometry , 
            "stdFloat32MultiArray":MongoWriter.write_Float32MultiArray 
            #"std":self.write_std,
            #"geometry":self.write_geometry
        }
        
    # 打開設定,讀取setting file內的資料庫名稱, 使用的資料庫collection names,以及要紀錄的rostopic name
    # 設定self.Database_handler其各自對應到不同collection所用interface 
    def setting(self): 
        with open(self.setting_file,"r") as file  : 
            
            file = json.load(file)
            collection_list = file["Collection_list"]
            DataBaseName = file["DataBase"]
            ROS_topic = file["ROS_TOPIC_list"]
            
        for collection in collection_list : 
            self.Database_handler[collection] = self.DataBase[DataBaseName][collection]

        self.ROS_topic = ROS_topic 
        #print(self.Database_handler)

    def register(self): 
        rospy.init_node("Ros_mongo",anonymous=True)
        
        for topic in self.ROS_topic: 
            print(topic)
            rospy.Subscriber(topic["Topic_name"],self.dataClass[topic["data_class"]], callback= self.msg_class_forWrite[topic["msg_type"]]
                             ,callback_args=(self.Database_handler[topic["collection"]],topic["Topic_name"]),queue_size=topic["quene_size"])   

            # old version , writer include in ths script 
            # rospy.Subscriber(topic["Topic_name"],self.dataClass[topic["data_class"]], callback= self.msg_class_forWrite[topic["msg_type"]]
            #      ,callback_args=(topic["Topic_name"],topic["collection"]),queue_size=topic["quene_size"])   

        rospy.loginfo("DataBase register Done! ")
        rospy.spin()

#if __name__ == "__main__": 

setting_file_path = rospy.get_param("/json_file",None)

MongoC = MongoBug(setting_file_path=setting_file_path)  if setting_file_path else MongoBug()
MongoC.register() 

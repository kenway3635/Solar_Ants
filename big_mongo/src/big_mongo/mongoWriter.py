#-*- coding:UTF-8 -*- 

import rospy 
from datetime import datetime 
class MongoWriter():
    
    #argument : tuple ( databaseHandle ,ros topicname , collection)
    @staticmethod
    def genTime():
        return datetime.now().isoformat()
    
    @staticmethod
    def write_std(msg,arguments):
        #arguments[0][arguments[2]].insert_one({arguments[1]:msg.data})
        arguments[0].insert_one({arguments[1]:msg.data, "date":MongoWriter.genTime()})
        rospy.loginfo(msg.data)
        
    
    @staticmethod
    def write_geometry(msg,arguments):
        msg = {
            "x_velocity":msg.linear.x , "z_velocity":msg.angular.z 
        }
        arguments[0].insert_one({arguments[1]:msg.data, "date":MongoWriter.genTime()})
        rospy.loginfo(msg) 
        
    @staticmethod 
    def write_Float32MultiArray(msg,arguments): 
        msg = { 
            "Brush":msg.data[0] , "Battery":msg.data[1] , "electrcity_box":msg.datap[2]
        }
        arguments[0].insert_one({arguments[1]:msg.data, "date":MongoWriter.genTime()})
        rospy.loginfo(msg)
        
    @staticmethod
    def write_nextGoal(DatabaseHandle,msg,arguments):
        msg ={
            "timestamp": msg.timestamp,
            "vehicle":{"vehicle_id":msg.vehicle_id , "platform":msg.platform },
            "uuid":msg.uuid ,
            "attribute":msg.attr , 
            "pose":{"x":msg.x , "y":msg.y, "rotation":msg.yaw },
            "quaternion":[msg.x,msg.y,msg.z,msg.w],
            "goal":msg.goal
        }
        arguments[0].insert_one({arguments[1]:msg.data, "date":MongoWriter.genTime()})
        
    @staticmethod
    def write_arrival(DatabaseHandle,msg,arguments): 
        msg = {
            "timestamp":msg.timestamp,
            "vehicle":{"vehicle_id":msg.vehicle_id , "platform":msg.platform },
            "uuid":msg.uuid ,
            "attribute":msg.attr 
        }
        arguments[0].insert_one({arguments[1]:msg.data, "date":MongoWriter.genTime()})
        rospy.loginfo(msg) 


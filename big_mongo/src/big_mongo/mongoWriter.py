#-*- coding:UTF-8 -*- 

import rospy 
class MongoWriter():
    
    @staticmethod
    def write_std(DatabaseHandle,msg,arguments):
        DatabaseHandle[arguments[1]].insert_one({arguments[0]:msg.data})
        rospy.loginfo(msg.data)
    
    @staticmethod
    def write_geometry(DatabaseHandle,msg,arguments):
        msg = {
            "x_velocity":msg.linear.x , "z_velocity":msg.angular.z 
        }
        DatabaseHandle[arguments[1]].insert_one({arguments[0]})
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
        DatabaseHandle[arguments[1]].insert_one({arguments[0]})
        
    @staticmethod
    def write_arrival(DatabaseHandle,msg,arguments): 
        msg = {
            "timestamp":msg.timestamp,
            "vehicle":{"vehicle_id":msg.vehicle_id , "platform":msg.platform },
            "uuid":msg.uuid ,
            "attribute":msg.attr 
        }
        DatabaseHandle[arguments[1]].insert_one({arguments[0]})
        rospy.loginfo(msg) 
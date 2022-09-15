
from os import times_result
from pymongo import MongoClient 
from datetime import datetime 

import matplotlib.pyplot as plt
import json ,csv 

database_server = MongoClient("192.168.0.82",27017,username="root",password="kangli0306")
database = database_server["SolarAnt_Data"]



Format_forDatatime = lambda time : [int(i) for i in time.spilt("/")]
    

def createJson(data,outputDict,collection,fileName ="output.json"):
    outputDict.update({str(datetime.fromisoformat(data["date"])):data["/voltage"]})
   
def TimeScope(Start,End): 
    return  {
            "date":{
                "$gte":datetime(*Start).isoformat() ,
                "$lte":datetime(*End).isoformat() 
            
            } 
        }

def effective(data ,size =1 ): 
    return (len(data) == size) 

def getTemperature(DB,timeStart,timeEnd ,show=True , *item): 
    Title = f"Temperature from {timeStart} to {timeEnd}"
    collection = "temperature" if not item else item 


    timeStart  = Format_forDatatime(timeStart) 
    timeEnd = Format_forDatatime(timeEnd) 
    
    
    if len(collection) == 1: 
        
        Data = DB[collection].find(TimeScope(timeStart,timeEnd))
        Data_length = DB[collection].count_document(TimeScope(timeStart,timeEnd)) 
        
    else : 
        Data = dict()
        for sub_collection in collection : 
            sub_collectionData =  DB[sub_collection].find(TimeScope(timeStart,timeEnd))
            Data.update( {sub_collection:sub_collectionData})
    
    
    

    if len(collection) == 1 :
        
        value_set = list()  
        time_set = list() 
        for i , data in enumerate(Data): 
            if i % 100 == 0:  
                if effective(data["data"] ): 
                    
                    value_set.append(data["data"]) 
                    time = datetime.fromisoformat(data["date"])
                    time_set.append(f"{time:%H:%M:%S}")

        if show: 
            plt.plot(time_set, value_set) 
            plt.ylim(25,30) 
            plt.legend(["temperature"])
    else: 
        
        value_set = [list() for i in range(len(collection))]
        time_set = list() 
        
        for i , sub_collection in enumerate(collection): 

            for j , data in enumerate(Data[sub_collection]) :  

                if j%100 == 0 : 

                    if effective(data["data"]) : 
                        
                        value_set[i].append(data["data"])
                        if i == 0 : 
                            time = datetime.fromisoformat(data["date"])
                            time_set.append(f"{time:%H:%M:%S}")
        if show: 
            plt.plot(time_set,value_set)  # note maybe have bug for data dimension ! 0915
            plt.ylim(30,60) 
            plt.legend(collection.keys())
    
    if show: 
        plt.xticks(rotation=70)
        plt.title(Title) 
        plt.xlabel("sample data")
        plt.grid()
        plt.show()



def getVoltage(DB,timeStart=None,timeEnd= None ,show=True ) : 
    Title = f"Voltage from {timeStart} to {timeEnd}"
    collection = "voltage"  


    def effective(data): 
        return bool(data["data"])
    
    timeStart  = Format_forDatatime(timeStart) 
    timeEnd = Format_forDatatime(timeEnd) 
        

    Data = DB[collection].find(TimeScope(timeStart,timeEnd))
    Data_length = DB[collection].cound_document(TimeScope(timeStart,timeEnd))
    
    value_set = list()
    time_set = list() 
    
    for i , data in enumerate(Data): 
        
        if i % 100 == 0 : 
            if effective(data["data"]): 
                value_set.append(data["data"])
                time = datetime.fromisoformat(data["date"])
                time_set.append(f"{time:%H:%M:%S}")
    if show: 
        plt.plot(time_set,value_set)
        plt.ylim(25,30) 
        plt.legend(["voltage"])
        plt.xticks(rotation=70)
        plt.title(Title) 
        plt.xlabel("sample data")
        plt.grid()
        plt.show()
        
    
        

def getPower(DB,timeStart=None,timeEnd= None ,show=True ) : 
    Title = f"Power from {timeStart} to {timeEnd}"
    collection = "Power"  


    def effective(data): 
        return bool(data["data"])
    
    timeStart  = Format_forDatatime(timeStart) 
    timeEnd = Format_forDatatime(timeEnd) 
        

    Data = DB[collection].find(TimeScope(timeStart,timeEnd))
    Data_length = DB[collection].cound_document(TimeScope(timeStart,timeEnd))
    
    value_set = list()
    time_set = list() 
    
    for i , data in enumerate(Data): 
        
        if i % 100 == 0 : 
            if effective(data["data"]): 
                value_set.append(data["data"])
                time = datetime.fromisoformat(data["date"])
                time_set.append(f"{time:%H:%M:%S}")
    if show: 
        plt.plot(time_set,value_set)
        plt.ylim(25,30) 
        plt.legend(["Power"])
        plt.xticks(rotation=70)
        plt.title(Title) 
        plt.xlabel("sample data")
        plt.grid()
        plt.show()
        
    
        
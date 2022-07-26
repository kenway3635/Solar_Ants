from os import times_result
from pymongo import MongoClient 
from datetime import datetime 
import matplotlib.pyplot as plt
import json ,csv 

database_server = MongoClient("192.168.0.82",27017,username="root",password="kangli0306")
database = database_server["SolarAnt_Data"]


def timeZone_Fix(timeList): 
    if timeList[3]  -8 <=0:
        timeList[2] -1  
        timeList[3] += 16
    else: 
        timeList[3] -=8
    print(timeList)
    return timeList


def createJson(data,outputDict,collection,fileName ="output.json"):
    outputDict.update({str(datetime.fromisoformat(data["date"])):data["/voltage"]})
   
    
    
### time格式要用 "years/mouth/day/hours/minutes"
def getData(collection,timeStart=None ,timeEnd=None):
    fileName = collection +".json"
    if timeStart and timeEnd: 
        fileName = collection+timeStart+"__to__"+timeEnd+".json"
        transInt = lambda list : [int(i) for i in list]
        #timeStart = timeZone_Fix(transInt(timeStart.split("/")))
        #timeEnd = timeZone_Fix(transInt(timeEnd.split("/")))
    
    
        timeStart = transInt(timeStart.split("/"))
        timeEnd = transInt(timeEnd.split("/"))
        
        Data = database[collection].find({
            "date":{"$gte":datetime(*timeStart).isoformat(),
                    "$lte":datetime(*timeEnd).isoformat()}
        })
        
        Data_length = database[collection].count_documents({
            "date":{"$gte":datetime(*timeStart).isoformat(),
                    "$lte":datetime(*timeEnd).isoformat()}
        })
        
    else:
        Data = database[collection].find() 

    output = {}
    global Title
    Title = collection + Data[0]["date"] +" / "+ Data[Data_length-1]["date"]
    for i,data in enumerate(Data) : 
        
        if i %10 == 0:
            #print(data)
            #print(datetime.fromisoformat(data["date"]) , type(datetime.fromisoformat(data["date"])))
            #output.update({str(datetime.fromisoformat(data["date"])):data["/temperature"]})
            temp_set.append(data["data"])
            #time_set.append(data["date"])
            #time_set=["" for i in range(len(temp_set))]
    print(output)
    with open("test.json","w") as file : 
        json.dump(output,file)
    
 
temp_set= [] 
getData("temperature","2022/07/26/10/00","2022/07/26/11/20")
time_set=[i for i in range(len(temp_set))]
plt.plot(time_set,temp_set) ,
plt.legend(["Brush","Battery","I.MX8MP","Electric Box"])
plt.title(Title) 
# plt.xlabel("Sample data")

plt.show()
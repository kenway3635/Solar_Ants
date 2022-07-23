from pymongo import MongoClient 

database = MongoClient("192.168.0.82",27017,username="root",password="kangli0306") 
db = database["Solar_test"] 
co = db["temp"] 

data = co.find() 
for i in data: 
    print(i)

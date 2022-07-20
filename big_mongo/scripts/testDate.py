from pymongo  import MongoClient
import time 
from datetime import datetime 
database = MongoClient("192.168.0.82",27017,username="root",password="kangli0306")
collection = database["testTime"]["ttt"]



def test_get(): 
    msg = {
        "content" : "Hello3",
        "date": datetime.now().isoformat()
    }
    return msg
#collection.insert_one(test_get())


data = collection.find({"date":{"$gte":datetime(2022,7,18,12,10).isoformat()} ,"content":"Hello3" } )

for i in data: 
    print(i)
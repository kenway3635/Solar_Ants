import paho.mqtt.client as mqtt


def on_connect(client,userdata,flasg,rc):
    print("on connectm,code",str(rc))
    AMRsub_client.subscribe("command")

def on_message(client, userdata, msg):
    # 轉換編碼utf-8才看得懂中文
    #print("from the",msg.topic)
    #print("msg is : ,",msg.payload.decode("utf-8"))
    print("resending :",msg.payload.decode("utf-8"))
    AMRpub_client.publish("mqtt2ros",msg.payload.decode("utf-8"))

AMRsub_client = mqtt.Client(client_id="",)
AMRpub_client = mqtt.Client()
AMRsub_client.on_connect = on_connect
AMRsub_client.on_message = on_message

try: 
    AMRsub_client.connect("192.168.0.140",1883,60)
    AMRpub_client.connect("127.0.0.1",1883,60)
    print("try to connect")
except:
    print("connect fail")

AMRsub_client.loop_forever()
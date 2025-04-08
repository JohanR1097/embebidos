import paho.mqtt.client as mqtt
from time import sleep

def on_connect(client, userdata, flags, rc):
    print(f"Connected")
    client.subscribe("rpi/topic")
    
def on_message(client, userdata, msg):
    print(msg.payload.decode())
    
client = mqtt.Client()
client.on_connect = on_connect
client.on_message = on_message

client.connect("localhost", 1883, 60)
client.loop_start()

while True:
    client.publish("robot/matriz", "on")
    sleep(1)

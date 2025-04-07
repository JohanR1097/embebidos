import network
import json
from time import sleep
from umqtt.robust import MQTTClient

SSID = "iPhone de Johan"
PWD = "Johan2024"

def wifi_connect():
    wlan = network.WLAN(network.STA_IF)
    wlan.active(True)
    wlan.connect(SSID,PWD)
    while not wlan.isconnected():
        sleep(1)
    print ("connected to WIFI")
    
def message (topic, msg):
    print (msg.decode())
    
client = MQTTClient ("Quice","172.20.10.2")
client.set_callback(message)


wifi_connect()
client.connect()
client.subscribe(b"test/topic")

while True:
    client.wait_msg()
    #client.publish(b"rpi/topic", "no, fore es hetero")
    data = json.dumps({"distance": 30})
    client.publish(b"rpi/topic", data)
    

import time
import paho.mqtt.client as mqtt
import os
import sys

try:
    image_filename = sys.argv[1]
except:
    print("first argument must be a valid webp file")
    exit(1)

topic = "plm/83F228/rx" # CHANGE ME
mqttClient = mqtt.Client("python_mqtt")
#mqttClient.username_pw_set("publish_user", 'pazz') # CHANGE ME
mqttClient.connect('tdm.wildc.net', 1883) # CHANGE ME
mqttClient.loop_start()

exists = os.path.isfile(image_filename)
if exists:
    file_size = os.stat(image_filename)
    print("Size of file :", file_size.st_size, "bytes")
    in_file = open(image_filename, "rb")
    info = mqttClient.publish(
        topic=topic,
        payload="START",
        qos=0,
        retain=False,
    )
    info.wait_for_publish()
    if info.is_published():
        print("Published START")

    time.sleep(1)
    info = mqttClient.publish(
        topic=topic,
        payload=file_size.st_size,
        qos=0,
        retain=False,
    )
    info.wait_for_publish()
    if info.is_published():
        print("Published " + str(file_size.st_size))

    time.sleep(1)
    info = mqttClient.publish(
        topic=topic,
        payload=in_file.read(),
        qos=0,
        retain=False,
    )
    info.wait_for_publish()
    if info.is_published():
        print("Published binary file")

    time.sleep(1)
    info = mqttClient.publish(
        topic=topic,
        payload="FINISH",
        qos=0,
        retain=False,
    )
    info.wait_for_publish()
    if info.is_published():
        print("Published topic: END")

mqttClient.loop_stop()    #Stop loop 
mqttClient.disconnect()
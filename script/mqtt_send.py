import time
from compas_eve import Publisher
from compas_eve import Topic
from compas_eve.mqtt import MqttTransport

info = dict({
    "step": 33,
    "type": 2,
    "part_id": 7,
    "robot_id": 1,
    "assembly_calibration": 1
})

# MQTT_SERVER  = "175.16.1.9" #"broker.emqx.io"
MQTT_SERVER  = "localhost" #"broker.emqx.io"
topic = Topic("/ust/zivid/")
tx = MqttTransport(MQTT_SERVER)
publisher = Publisher(topic, transport=tx)
print(f"Publishing message: {info}")
publisher.publish(info)
time.sleep(1)
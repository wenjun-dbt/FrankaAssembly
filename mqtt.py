import time
from compas_eve import Publisher
from compas_eve import Topic
from compas_eve.mqtt import MqttTransport

info = dict({
    "pos": [0.0, 1.0, 0.1, 1.0, 0.9, 9.8],
    "world": 1.0
})

topic = Topic("/compas_eve/zivid/")
tx = MqttTransport("broker.emqx.io")
publisher = Publisher(topic, transport=tx)
print(f"Publishing message: {info}")
publisher.publish(info)
time.sleep(1)
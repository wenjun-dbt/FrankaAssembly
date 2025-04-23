import time
from compas_eve import Publisher
from compas_eve import Topic
from compas_eve.mqtt import MqttTransport

topic = Topic("/compas_eve/zivid/")
tx = MqttTransport("broker.emqx.io")
publisher = Publisher(topic, transport=tx)
msg = dict(text=f"Hello world")
print(f"Publishing message: {msg}")
publisher.publish(msg)
time.sleep(1)
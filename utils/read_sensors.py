import time
import datetime
import paho.mqtt.client as mqtt

MQTT_URL = "192.168.50.13"

class MqttWrapper:
    def __init__(self):
        self.epoch = datetime.datetime.utcfromtimestamp(0)
        self.flag_connected = 0
        self.client = mqtt.Client("", True, None, mqtt.MQTTv311)
        self.client.on_connect = self.on_connect
        self.client.on_disconnect = self.on_disconnect
        self.client.on_message = self.on_message
        self.client.username_pw_set(username="HF_PILOT_ONE_RC")
        self.client.connect(MQTT_URL, 1883, 60)
        self.client.loop_start()
        self.data = None
        self.client.subscribe("HP_PILOT_ONE/sensors")
        
        while self.flag_connected == 0:
            print("conecting...")
            time.sleep(1)
        print("Connected to MQTT server!") 
        
        

    def stop(self):
        self.client.loop_stop()

    def on_message(self, mqttc, obj, msg):
        print(msg.topic + " " + str(msg.qos) + " " + str(msg.payload))

    def checkIfConnected(self):
        """checks if connected and tries to reconnect. Warning! this func is blocking!"""
        if self.flag_connected == 0:
            print("Reconecting...")
            self.client.reconnect()
            while self.flag_connected == 0:
                print("connecting...")
                time.sleep(1)

    def on_connect(self, client, userdata, flags, rc):
        if rc == 0:
            print("connected!")
            self.flag_connected = 1
        else:
            print("error connecting!")

    def on_disconnect(self, client, userdata, rc):
        self.flag_connected = 0
    
    def loop_forever(self):
        self.client.loop_forever()

def main():
    mqtt = MqttWrapper()
    mqtt.checkIfConnected()
    while(True):
        time.sleep(1)


if __name__ == "__main__":
    main()
import time
import datetime
import struct
import numpy as np
import paho.mqtt.client as mqtt

MQTT_URL = "192.168.50.13"

class SensorParser:
    def __init__(self):
        self.acc = np.zeros(3, dtype=np.float32)
        self.gyro = np.zeros(3, dtype=np.float32)
        self.mag = np.zeros(3, dtype=np.float32)
        self.gps_avalaible = False
        self.lat = 0
        self.lon = 0
        self.sat_amount = 0

    def unpack(self, data):
        arr = []
        for i in range(0, len(data), 2):
            high = 16 * (data[i] - 48 if data[i] < 65 else data[i] - 65 + 10)
            low = data[i+1] - 48 if data[i+1] < 65 else data[i+1] - 65 + 10
            arr.append(high+low)
        arr = bytearray(arr)

        [
            self.acc[0], 
            self.acc[1],
            self.acc[2],
            self.gyro[0],
            self.gyro[1],
            self.gyro[2],
            self.mag[0],
            self.mag[1],
            self.mag[2],
            self.lat,
            self.lon,
            self.sat_amount,
            self.gps_avalaible,
        ] = struct.unpack('<fff fff fff ddHH xxxx', arr)

    def print(self):
        print(self.acc, self.gyro, self.mag, self.lat, self.lon, self.sat_amount, self.gps_avalaible)

sensors = SensorParser()

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
        if msg.topic == "HP_PILOT_ONE/sensors":
            sensors.unpack(msg.payload)
            sensors.print()

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
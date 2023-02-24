import time
import datetime
import struct
import numpy as np
import paho.mqtt.client as mqtt
from vpython import arrow, rate, color, vector

MQTT_URL = "192.168.50.13"

class MovingAverage:
    def __init__(self, size=60):
        self.bufferSize = size
        self.buffer = [np.zeros(3, dtype=np.float32)]*self.bufferSize
        self.index = 0
        self.maxed = False

    def insert(self, val):
        self.buffer[self.index] = val
        self.index += 1
        if self.index >= self.bufferSize:
            self.index = 0
            self.maxed = True

    def get(self):
        end = self.bufferSize if self.maxed else self.index + 1
        sum = np.zeros(3, dtype=np.float32)
        for i in range(end):
            sum += self.buffer[i]
        return sum / end

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
        self.sensor_callback = None
        
        while self.flag_connected == 0:
            print("conecting...")
            time.sleep(1)
        print("Connected to MQTT server!")
    
    def set_sensor_callback(self, clbck):
        self.sensor_callback = clbck
        
    def stop(self):
        self.client.loop_stop()

    def on_message(self, mqttc, obj, msg):
        if msg.topic == "HP_PILOT_ONE/sensors":
            if self.sensor_callback is not None:
                self.sensor_callback(msg.payload)

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
    arr = arrow(pos=vector(0,0,0), axis=vector(1,0,0), color=color.red)
    time.sleep(2)
    
    mqtt = MqttWrapper()
    mqtt.checkIfConnected()
    sensors = SensorParser()
    acc_average = MovingAverage(size=20)

    def callback(data):
        sensors.unpack(data)
        sensors.print()
        acc_average.insert(sensors.acc / np.linalg.norm(sensors.acc))
    mqtt.set_sensor_callback(callback)
        
    # file = open("mag_data.csv", 'w')
    # file.write("x y z\n")
    
    while(True):
        rate(60)
        v = acc_average.get()
        arr.axis = vector(v[1], v[0], v[2])
        
        # file.write("{:.7f} {:.7f} {:.7f}\n".format(sensors.mag[0], sensors.mag[1], sensors.mag[2]))
        # file.flush()
        time.sleep(1 / 100)


if __name__ == "__main__":
    main()
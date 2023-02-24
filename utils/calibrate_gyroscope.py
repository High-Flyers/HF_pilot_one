import time
import numpy as np
from MqttWrapper import MqttWrapper, SensorParser
from math_utils import get_rot_mat, LowPassFilter

drone_transform = get_rot_mat(0, np.pi, -0.75*np.pi)

CAL_TIME = 20.0

GYRO_OFFSET_PITCH = -3.2743889
GYRO_OFFSET_ROLL = -1.2538174
GYRO_OFFSET_YAW = -1.9277307
 
def pitch_roll__yaw_from_gyro_corrected(gyro, p, r, y, dt):
    normalized = (180.0 / np.pi) * np.matmul(drone_transform, gyro)
    pitch = p + (normalized[1] + GYRO_OFFSET_PITCH) * dt
    roll = r + (normalized[0] + GYRO_OFFSET_ROLL) * dt
    yaw = y + (normalized[2] + GYRO_OFFSET_YAW) * dt
    return pitch, roll, yaw


def pitch_roll__yaw_from_gyro(gyro, p, r, y, dt):
    normalized = (180.0 / np.pi) * np.matmul(drone_transform, gyro)
    pitch = p + normalized[1] * dt
    roll = r + normalized[0] * dt
    yaw = y + normalized[2] * dt
    return pitch, roll, yaw

pitch_raw_gyro, roll_raw_gyro, yaw_raw_gyro = 0.0, 0.0, 0.0
last_update = time.time()

def main():    
    mqtt = MqttWrapper()
    mqtt.checkIfConnected()
    sensors = SensorParser()
   

    def callback(data):
        global pitch_raw_gyro, roll_raw_gyro, yaw_raw_gyro, last_update
        sensors.unpack(data)
        now = time.time()
        dt = now - last_update
        last_update = now
        pitch_raw_gyro, roll_raw_gyro, yaw_raw_gyro = pitch_roll__yaw_from_gyro(sensors.gyro, pitch_raw_gyro, roll_raw_gyro, yaw_raw_gyro, dt)

    mqtt.set_sensor_callback(callback)

    while(mqtt.flag_connected == 0):
        time.sleep(0.5)

    print("---- CALIBRATION STARTED!! DON'T MOVE THE DRONE!!! ----")
    time.sleep(CAL_TIME)
    offset_pitch = -pitch_raw_gyro / CAL_TIME
    offset_roll = -roll_raw_gyro / CAL_TIME
    offset_yaw = -yaw_raw_gyro / CAL_TIME

    print("{:.7f} {:.7f} {:.7f}".format(offset_pitch, offset_roll, offset_yaw))



        





if __name__ == "__main__":
    main()
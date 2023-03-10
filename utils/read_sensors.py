import time
import numpy as np
from MqttWrapper import MqttWrapper, SensorParser
from math_utils import LowPassFilter, ComplimentaryFilter, get_rot_mat
import matplotlib.pyplot as plt
import matplotlib.animation as animation

GYRO_OFFSET_PITCH = -3.2743889
GYRO_OFFSET_ROLL = -1.2538174
GYRO_OFFSET_YAW = -1.9277307
 
drone_transform = get_rot_mat(0, np.pi, -0.75*np.pi)

def pitch_roll_from_acc(acc):
    normalized = np.matmul(drone_transform, acc)
    pitch = (180.0 / np.pi) * np.arctan2(normalized[0], np.sqrt(normalized[1]**2 + normalized[2]**2))
    roll = (180.0 / np.pi) * np.arctan2(normalized[1], normalized[2])
    return pitch, roll

def pitch_roll_yaw_dt_from_gyro(gyro):
    normalized = (180.0 / np.pi) * np.matmul(drone_transform, gyro)
    pitch_dt = normalized[1] + GYRO_OFFSET_PITCH
    roll_dt = normalized[0] + GYRO_OFFSET_ROLL
    yaw_dt = normalized[2] + GYRO_OFFSET_YAW
    return pitch_dt, roll_dt, yaw_dt

def pitch_roll__yaw_from_gyro(gyro, p, r, y, dt):
    p_t, r_t, y_t = pitch_roll_yaw_dt_from_gyro(gyro)
    pitch = p + p_t * dt
    roll = r + r_t * dt
    yaw = y + y_t * dt
    return pitch, roll, yaw



pitch_comb, roll_comb = 0.0, 0.0

def main():    
    mqtt = MqttWrapper()
    mqtt.checkIfConnected()
    sensors = SensorParser()
    acc_filter = LowPassFilter(init_val=np.zeros(3, dtype=np.float32), alpha=0.1)
    pitch_compl = ComplimentaryFilter(alpha=0.3)
    roll_compl = ComplimentaryFilter(alpha=0.3)

    def callback(data):
        global pitch_comb, roll_comb
        sensors.unpack(data)
        # sensors.print()
        pitch_acc, roll_acc = pitch_roll_from_acc(acc_filter.get(sensors.acc))
        pitch_dt, roll_dt, _ = pitch_roll_yaw_dt_from_gyro(sensors.gyro)
        pitch_comb = pitch_compl.update(pitch_acc, pitch_dt)
        roll_comb = roll_compl.update(roll_acc, roll_dt)

    mqtt.set_sensor_callback(callback)
        
    # file = open("mag_data.csv", 'w')
    # file.write("x y z\n")

    xs = []
    y1s = []
    y2s = []

    fig = plt.figure()
    ax1 = fig.add_subplot(1,1,1)

    def animate(i):
        ax1.clear()
        ax1.plot(xs, y1s, 'r-')
        ax1.plot(xs, y2s, 'b-')
    
    ani = animation.FuncAnimation(fig, animate, interval=int(1000 / 30))
    plt.ion()
    plt.show()

    while(True):
        # print("{:.2f} {:.2f}\t{:.2f} {:.2f} {:.2f}".format(pitch_acc, roll_acc, pitch_raw_gyro, roll_raw_gyro, yaw_raw_gyro))
        print("{:.2f} {:.2f}".format(pitch_comb, roll_comb))

        xs.append(time.time())
        y1s.append(pitch_comb)
        y2s.append(roll_comb)

        xs = xs[-4*30:]
        y1s = y1s[-4*30:]
        y2s = y2s[-4*30:]
    
        # file.write("{:.7f} {:.7f} {:.7f}\n".format(sensors.mag[0], sensors.mag[1], sensors.mag[2]))
        # file.flush()
        # time.sleep(1 / 30)
        plt.pause(1 / 30)


if __name__ == "__main__":
    main()
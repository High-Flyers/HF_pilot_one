import numpy as np
import matplotlib.pyplot as plt

'''
algorithm taken from: https://www.appelsiini.net/2018/calibrate-magnetometer/
'''

data = np.genfromtxt("mag_data.csv", delimiter=' ')
mag_x = data[1:-1,0]
mag_y = data[1:-1,1]
mag_z = data[1:-1,2]

offset_x = (np.max(mag_x) + np.min(mag_x)) / 2
offset_y = (np.max(mag_y) + np.min(mag_y)) / 2
offset_z = (np.max(mag_z) + np.min(mag_z)) / 2

corrected_x = mag_x - offset_x
corrected_y = mag_y - offset_y
corrected_z = mag_z - offset_z

print("{:.7f} {:.7f} {:.7f}".format(offset_x, offset_y, offset_z))

plt.figure(1)
plt.subplot(211)
plt.scatter(mag_x, mag_y)
plt.scatter(mag_x, mag_z)
plt.scatter(mag_y, mag_z)
plt.subplot(212)
plt.scatter(corrected_x, corrected_y)
plt.scatter(corrected_x, corrected_z)
plt.scatter(corrected_y, corrected_z)
plt.show()
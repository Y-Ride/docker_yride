import os
import matplotlib.pyplot as plt
import numpy as np
import pandas as pd
import sys

# Change the working directory to script's directory
os.chdir(os.path.dirname(os.path.abspath(__file__)))
print(f"Working directory: {os.getcwd()}")

# sys.exit()
# Load the data
data_folder = './results'
df_imu = pd.read_csv(f'{data_folder}/timestamps_imu.txt', delimiter=',', header=0)
print(df_imu.keys()[1])
print(df_imu.head())
df_lidar = pd.read_csv(f'{data_folder}/timestamps_lidar.txt', delimiter=',', header=0)
print(df_lidar.head())

timestamps_imu = df_imu[df_imu.keys()[1]].values * 1e-9
timestamps_lidar = df_lidar[df_lidar.keys()[1]].values * 1e-9

timestamps_imu = timestamps_imu - timestamps_imu[0]
timestamps_lidar = timestamps_lidar - timestamps_lidar[0]

imu_y = np.array([0] * len(timestamps_imu))
lidar_y = np.array([1] * len(timestamps_lidar))

print(timestamps_imu[:10])
print(timestamps_lidar[:10])

# Plot the data
plt.figure()
plt.scatter(timestamps_imu, imu_y, label='IMU')
plt.scatter(timestamps_lidar, lidar_y, label='Lidar')
plt.xlabel('Timestamp (s)')
# plt.ylabel('Timestamp (s)')
plt.xlim([0, 10])
plt.legend()
plt.show()


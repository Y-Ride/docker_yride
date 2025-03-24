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
data_folder = './results/outside_linear'
df_imu = pd.read_csv(f'{data_folder}/timestamps_imu.txt', delimiter=',', header=0)
print(df_imu.keys())
print(df_imu.head())
df_lidar = pd.read_csv(f'{data_folder}/timestamps_lidarImu.txt', delimiter=',', header=0)
print(df_lidar.head())

timestamps_imu = df_imu[df_imu.keys()[1]].values * 1e-9
timestamps_lidar = df_lidar[df_lidar.keys()[1]].values * 1e-9

initial_time= timestamps_imu[0] # Need to subtract off a constant to get them to align correctly
timestamps_imu = timestamps_imu - initial_time
timestamps_lidar = timestamps_lidar - initial_time

# imu_y = np.array([0] * len(timestamps_imu))
# lidar_y = np.array([1] * len(timestamps_lidar))

# Set y values to be angular velocity x
imu_y_x = df_imu[df_imu.keys()[2]].values
lidar_y_x = df_lidar[df_lidar.keys()[2]].values

# Set y values to be angular velocity y
imu_y_y = df_imu[df_imu.keys()[3]].values
lidar_y_y = df_lidar[df_lidar.keys()[3]].values

# Set y values to be angular velocity z
imu_y_z = df_imu[df_imu.keys()[4]].values
lidar_y_z = df_lidar[df_lidar.keys()[4]].values

print(timestamps_imu[:10])
print(timestamps_lidar[:10])

# Plot the x data
plt.figure()
plt.scatter(timestamps_imu, imu_y_x, label='IMU x')
plt.scatter(timestamps_lidar, lidar_y_x, label='Lidar x')
plt.xlabel('Timestamp (s)')
# plt.xlabel('Time Published (s)')
# plt.xlim([0, 10])
plt.legend()
plt.show()

# plot the y data
plt.figure()
plt.scatter(timestamps_imu, imu_y_y, label='IMU y')
plt.scatter(timestamps_lidar, lidar_y_y, label='Lidar y')
plt.xlabel('Timestamp (s)')
# plt.xlim([0, 10])
plt.legend()
plt.show()

# plot the z data
plt.figure()
plt.scatter(timestamps_imu, imu_y_z, label='IMU z')
plt.scatter(timestamps_lidar, -lidar_y_z, label='Lidar -z')
plt.xlabel('Timestamp (s)')
# plt.xlim([0, 10])
plt.legend()
plt.show()




import csv
import time
import numpy as np
import io
import pynmea2
import serial
from threading import Thread
from collections import deque


import gps_data_collect as gps_data

gps_data = gps_data.GPSData("/dev/ttyACM0")
gps_data.run()

# setting up csv file
time.sleep(1.25)

# generate filename based on current time
filename = time.strftime("%Y-%m-%d_%H-%M-%S", time.localtime())
filename = 'data/unprocessed_data/' + filename + '.csv'

with open(filename, 'w') as f:
    writer = csv.writer(f)
    while True:
        try:
            rtk_gps_string = gps_data.get_NMEA_string()
            data = [str(time.time()), rtk_gps_string]
            print(str(rtk_gps_string))
            # print(str(heading) + " " + str(imu_gps))
            writer.writerow(data)
            time.sleep(1)
        except serial.SerialException as e:
            print('Device error: {}'.format(e))
            break

        except pynmea2.ParseError as e:
            print('Parse error: {}'.format(e))
            continue

        except KeyboardInterrupt:
            print("exiting nicely")
            break

gps_data.close_connection()

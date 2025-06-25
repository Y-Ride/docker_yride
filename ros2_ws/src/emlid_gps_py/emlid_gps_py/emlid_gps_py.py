from rclpy.node import Node
import rclpy
from std_msgs.msg import String
import pynmea2
import serial  # pip install pyserial
from threading import Thread
from collections import deque
import time
import threading

from emlid_gps_py import gps_data_collect as gps_data
from builtin_interfaces.msg import Time


class EmlidGpsPy(Node):
    def __init__(self):
        super().__init__('emlid_gps_py')
        self.publisher_nmea_ = self.create_publisher(String, 'nmea', 10)
        # self.publisher_navsatfix_ = self.create_publisher(std_msgs.msg.NavSatFix, 'navsatfix', 10)

        self.port_ = self.declare_parameter("device_port", "/dev/ttyACM0")
        self.port = self.get_parameter("device_port").get_parameter_value().string_value
        
        self.gps_data = gps_data.GPSData(self.port)
        self.gps_data.run()

        self.timer = self.create_timer(0.010, self.nmea_read_and_pub)
    
    # def publish_parsed_data(self, msg):
    #     if isinstance(msg, pynmea2.types.talker.GGA):
    #         navsat_msg = std_msgs.msg.NavSatFix()

    #         # Header
    #         navsat_msg.header.stamp = get_clock().now().to_msg()
    #         navsat_msg.header.frame_id = "emlid"

    #         # Status
    #         navsat_msg.status.status = std_msgs.msg.NavSatStatus.STATUS_FIX if int(msg.gps_qual) > 0 else std_msgs.msg.NavSatStatus.STATUS_NO_FIX
    #         navsat_msg.status.service = std_msgs.msg.NavSatStatus.SERVICE_GPS

    #         # Position
    #         navsat_msg.latitude = msg.latitude
    #         navsat_msg.longitude = msg.longitude
    #         navsat_msg.altitude = float(msg.altitude)

    #         self.publisher_navsatfix_.publish(navsat_msg)

    def nmea_read_and_pub(self):
        # while rclpy.ok():
        try:
            rtk_gps_string = self.gps_data.get_NMEA_string()
            if rtk_gps_string is not None:
                self.get_logger().info(str(rtk_gps_string))
                msg = String()
                msg.data = str(rtk_gps_string)
                self.publisher_nmea_.publish(msg)
                # print(str(rtk_gps_string))
                # self.publish_parsed_data(rtk_gps_string)
            # time.sleep(1)
        except serial.SerialException as e:
            self.get_logger().error(f'Device error {e}')
            return
        except pynmea2.ParseError as e:
            self.get_logger().warn(f'Parse error: {e}')

        except KeyboardInterrupt:
            self.get_logger().info('Exiting Nicely')
            return
        
    def destroy_node(self):
        self.gps_data.close_connection()
        super().destroy_node()





def main(args=None):
    rclpy.init(args=args)
    emlid = EmlidGpsPy()
    try:
        rclpy.spin(emlid)
    except KeyboardInterrupt:
        pass
    finally:
        emlid.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()

import rclpy
import rclpy.logging
from rclpy.node import Node
import numpy as np
import matplotlib.pyplot as plt
from sensor_msgs.msg import NavSatFix
from rclpy.executors import ExternalShutdownException
import threading
import time
import os

class LLARefNode(Node):
    def __init__(self):
        super().__init__('lla_ref_node')
        self.sub = self.create_subscription(
            NavSatFix,
            "/inertial_sense/gps1/NavSatFix",
            self.lla_callback,
            10
        )
        self.sub # prevent unused variable warning
        self.firstpass = True
        self.lat = []
        self.lon = []
        self.alt = []
        self.cnt = 0

        # Timer
        self.timer = None
        self.timer_duration = 5.0

        self.get_logger().info("Initialized 'lla_ref_node'!")

    def reset_timer(self):
        if self.timer is not None:
            self.timer.cancel()
        self.timer = self.create_timer(self.timer_duration, self.plot_data)

    def lla_callback(self, msg):
        if self.firstpass:
            self.get_logger().info("Received first GPS msg!")
            self.firstpass = False
        self.cnt += 1
        self.lat.append(msg.latitude)
        self.lon.append(msg.longitude)
        self.alt.append(msg.altitude)
        if (self.cnt % 100) == 0:
            print("Total msgs ", self.cnt)
            print("\tLat -> ", msg.latitude)
            print("\tLon -> ", msg.longitude)
            print("\tAlt -> ", msg.altitude)
        self.reset_timer()

    def plot_data(self):
        self.get_logger().info(f"No data was received in the last {self.timer_duration} sec. Generating plots...")
        if not self.lat or not self.lon or not self.alt:
            self.get_logger().warn("No data was received. Can't make plots")
            return
        
        self.get_logger().info("Plotting final results")
        
        data_list = [self.lat, self.lon, self.alt]
        labels = ["Latitude", "Longitude", "Altitude"]
        
        fig, axis = plt.subplots(3, 1, figsize=(10, 8))
        for i, (data, label) in enumerate(zip(data_list, labels)):
            axis[i].plot(data, label=label)
            axis[i].set_xlabel("Time Step")
            axis[i].set_ylabel(label) 
            # axis[i].legend()
            axis[i].grid(True)
        
        plt.suptitle("LLA_Ref Data")
        plt.tight_layout()
        plt.subplots_adjust(top=0.9)
        # plt.show()
        plt_filename = os.path.join('/tmp', 'lla_ref_plot.png')
        plt.savefig(plt_filename)
        self.get_logger().info(f"Saved plot to {plt_filename}")
        plt.close(fig)


        # Clear data list to allow for continuous running
        self.firstpass = True
        self.lat = []
        self.lon = []
        self.alt = []
        self.timer.cancel()
        self.timer = None
        self.get_logger().info("Variables reset. Copy plot to new location and send me more data :)")
        print("")


def main(args=None):
    rclpy.init(args=args)
    try:
        lla_ref = LLARefNode()

        rclpy.spin(lla_ref)
    except (KeyboardInterrupt, ExternalShutdownException):
        rclpy.shutdown()


if __name__ == '__main__':
    main()
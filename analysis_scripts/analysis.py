#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from custom_interface.msg import GPSmsg
import matplotlib.pyplot as plt

class GPS_Plotter(Node):
    def __init__(self):
        super().__init__('gps_plotter')
    
        self.subscription = self.create_subscription(
            GPSmsg,
            'gps',
            self.listener_callback,
            10)
        
        self.easting_values = []
        self.northing_values = []
        
        plt.ion() 
        self.figure, self.axis = plt.subplots()
        self.axis.set_xlabel("UTM Easting (meters)")
        self.axis.set_ylabel("UTM Northing (meters)")
        self.line, = self.axis.plot([], [], color='red', marker='o', linestyle='-', linewidth=2, markersize=5)

    def listener_callback(self, msg):
        
        self.easting_values.append(msg.utm_easting)
        self.northing_values.append(msg.utm_northing)
        min_easting = min(self.easting_values)
        min_northing = min(self.northing_values)
        
        adjusted_easting = [value - min_easting for value in self.easting_values]
        adjusted_northing = [value - min_northing for value in self.northing_values]

        self.line.set_xdata(adjusted_easting)
        self.line.set_ydata(adjusted_northing)
        self.axis.relim()  
        self.axis.autoscale_view() 
        plt.draw()
        plt.pause(0.01)    


def main(args=None):
    rclpy.init(args=args)
    
    gps_plotter = GPS_Plotter()

    try:
        rclpy.spin(gps_plotter)
    finally:
        gps_plotter.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

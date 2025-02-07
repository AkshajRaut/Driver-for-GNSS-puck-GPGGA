#!/usr/bin/env python3

import serial
import rclpy
from rclpy.node import Node
from custom_interface.msg import GPSmsg
from std_msgs.msg import Header
import utm

class GPSDriver(Node):
    def __init__(self):
        super().__init__('gps_driver')
        self.get_logger().info("Driver started")
        self.declare_parameter('port','/dev/ttyACM0')
        self.declare_parameter('baudrate', 4800)
        port = self.get_parameter('port').value
        baud = self.get_parameter('baudrate').value
        self.port = serial.Serial(port, baud, timeout=3)
        self.get_logger().info("Using Serial port: "+port)

        self.publisher = self.create_publisher(GPSmsg, 'gps', 10)
        self.timer = self.create_timer(0.1, self.timer_callback)


    def timer_callback(self):
        line = self.port.readline().decode().strip()
        if line.startswith('$GPGGA'):
            data = line.split(',')
            self.get_logger().info('Timestamp '+data[1])
            self.get_logger().info('Lat '+data[2])
            self.get_logger().info('Long '+data[4])
            self.get_logger().info('Alt '+data[9])
            
            utc = data[1]
            hours = int(utc[0:2])
            mins = int(utc[2:4])
            secs = float(utc[4:])
            utc_secs = int(hours*3600 + mins*60 + secs)
            utc_nsecs = int((secs-int(secs))*1e9)

            latitude = float(data[2])
            lat_dir = data[3]

            longitude = float(data[4])
            lon_dir = data[5]

            altitude = float(data[9])

            lat_deg = int(latitude//100)
            lat_min = latitude%100
            lat = lat_deg + lat_min/60
            if lat_dir == 'S':
                lat = -lat

            lon_deg = int(longitude//100)
            lon_min = longitude%100
            lon = lon_deg + lon_min/60
            if lon_dir == 'W':
                lon = -lon
            
            
            utm_c = utm.from_latlon(lat, lon)
            utm_easting = utm_c[0]
            utm_northing = utm_c[1]
            zone = utm_c[2]
            letter = utm_c[3]

            self.get_logger().info('Easting '+str(utm_easting))
            self.get_logger().info('Northing '+str(utm_northing))
            self.get_logger().info('Zone '+ str(zone))
            self.get_logger().info('Letter '+letter)

            msg = GPSmsg()
            msg.header = Header()
            msg.header.stamp.sec = utc_secs
            msg.header.stamp.nanosec = utc_nsecs
            msg.header.frame_id = "GPS1_Frame"
            msg.latitude = lat
            msg.longitude = lon
            msg.altitude = altitude
            msg.utm_easting = utm_easting
            msg.utm_northing = utm_northing
            msg.zone = zone
            msg.letter = letter

            self.publisher.publish(msg)
            self.get_logger().info("Published GPS Data "+str(msg))

def main(args=None):
    rclpy.init(args=args)
    node = GPSDriver()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.port.close()
        node.destroy_node()
        rclpy.shutdown()
        

if __name__ == '__main__':
    main()


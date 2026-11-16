import rclpy
from rclpy.node import Node
import serial
import utm
from gps_msg_interface.msg import GPSmsg


class GPS_Driver(Node):
    def __init__(self):
        super().__init__("gps_driver_node_lol")
        self.declare_parameter("port", "/dev/ttyUSB0")
        serial_port = self.get_parameter("port").value
        self.port=serial.Serial(serial_port, baudrate=4800)
        self.publisher=self.create_publisher(GPSmsg,'gps',10)
        self.timer = self.create_timer(0.1, self.timer_callback)
        self.get_logger().info("intiializing gps driver")


    def timer_callback(self):
        self.get_logger().info("timer triggered")

        line = self.port.readline().decode().strip()

        lat=None
        lon=None
        alt=None

        if line.startswith('$GPGGA'):

            gpgga_split = line.split(',')

            timestamp = gpgga_split[1]
            lat=float(gpgga_split[2])
            lon=float(gpgga_split[4])
            ns=gpgga_split[3]
            ew=gpgga_split[5]

            alt=float(gpgga_split[9])

            dec_lat=int(lat/100) + ((lat%100)/60.0)
            dec_lon=int(lon/100) + ((lon%100)/60.0)

            if ew=='W':
                dec_lon=-dec_lon

            if ns=='S':
                dec_lat=-dec_lat

            parsed_message = GPSmsg()
            parsed_message.latitude=dec_lat
            parsed_message.longitude=dec_lon
            parsed_message.altitude=float(alt)


            easting,northing,zone_number,zone_letter = utm.from_latlon(dec_lat,dec_lon)

            #the following conversion is AI
            hours = int(timestamp[0:2])
            minutes = int(timestamp[2:4])
            seconds = int(timestamp[4:6])
            total_seconds = (hours * 3600) + (minutes * 60) + seconds
            fraction = float("0." + timestamp.split(".")[1]) if "." in timestamp else 0.0
            nanoseconds = int(fraction * 1e9)
            ##end AI

            parsed_message.header.stamp.sec = total_seconds
            parsed_message.header.stamp.nanosec = nanoseconds
            parsed_message.utm_easting=easting
            parsed_message.utm_northing=northing
            parsed_message.zone=zone_number
            parsed_message.letter=zone_letter
            parsed_message.header.frame_id = 'GPS1_Frame'
            self.publisher.publish(parsed_message)
        

def main(args=None):
    rclpy.init(args=args)

    gps_driver = GPS_Driver()

    rclpy.spin(gps_driver)

    gps_driver.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()


#yudai's driver
import math
import rclpy
from rclpy.node import Node
import serial

from std_msgs.msg import Header
from sensor_msgs.msg import Imu, MagneticField
from imu_msgs.msg import IMUmsg


def vn_checksum(sentence_body):
    '''XOR checksum of everything between $ and *, as hex, for commands we send to the VN-100.'''
    checksum = 0
    for ch in sentence_body:
        checksum ^= ord(ch)
    return format(checksum, '02X')


def euler_to_quaternion(roll, pitch, yaw):
    '''roll, pitch, yaw in RADIANS -> quaternion (x, y, z, w). Standard ZYX (yaw-pitch-roll) convention.'''
    cy, sy = math.cos(yaw * 0.5), math.sin(yaw * 0.5)
    cp, sp = math.cos(pitch * 0.5), math.sin(pitch * 0.5)
    cr, sr = math.cos(roll * 0.5), math.sin(roll * 0.5)

    w = cr * cp * cy + sr * sp * sy
    x = sr * cp * cy - cr * sp * sy
    y = cr * sp * cy + sr * cp * sy
    z = cr * cp * sy - sr * sp * cy
    return x, y, z, w


class IMUDriverNode(Node):
    def __init__(self):
        super().__init__('imu_driver')
        SENSOR_NAME = "imu"

        self.declare_parameter('port', '/dev/ttyUSB0')
        self.declare_parameter('baudrate', 115200)

        serial_port = self.get_parameter('port').value
        serial_baud = self.get_parameter('baudrate').value

        self.port = serial.Serial(serial_port, serial_baud, timeout=3.0)
        self.get_logger().debug("Using IMU on port " + serial_port + " at " + str(serial_baud))
        self.get_logger().debug("Initializing IMU driver ...")

        self.set_output_rate_40hz()

        self.imu_pub = self.create_publisher(IMUmsg, SENSOR_NAME, 10)

        self.get_logger().debug("Initialization complete")
        self.get_logger().info("Publishing IMU data on /imu.")

        self.sleep_time = 0.01
        self.timer = self.create_timer(self.sleep_time, self.timer_callback)

    def set_output_rate_40hz(self):
        '''Write the VN-100's async data output rate register to 40 Hz.
        This sets the DEVICE's own output rate -- not a sleep() in our loop.'''
        body = "VNWRG,07,40"
        cmd = "$%s*%s\r\n" % (body, vn_checksum(body))
        self.port.write(cmd.encode('ascii'))
        self.get_logger().debug("Sent rate-register command: " + cmd.strip())

    def timer_callback(self):
        line = self.port.readline().decode('ascii', errors='replace').strip()

        if line == '':
            self.get_logger().debug("IMU: No data")
            return

        if not line.startswith('$VNYMR'):
            return

        body = line.split('*')[0]
        fields = body.split(',')
        if len(fields) < 13:
            self.get_logger().warn("Malformed VNYMR sentence: " + line)
            return

        try:
            yaw_deg = float(fields[1])
            pitch_deg = float(fields[2])
            roll_deg = float(fields[3])
            mag_x_g = float(fields[4])
            mag_y_g = float(fields[5])
            mag_z_g = float(fields[6])
            accel_x = float(fields[7])
            accel_y = float(fields[8])
            accel_z = float(fields[9])
            gyro_x = float(fields[10])
            gyro_y = float(fields[11])
            gyro_z = float(fields[12])
        except Exception:
            self.get_logger().warn("Data exception: " + line)
            return

        roll = math.radians(roll_deg)
        pitch = math.radians(pitch_deg)
        yaw = math.radians(yaw_deg)
        qx, qy, qz, qw = euler_to_quaternion(roll, pitch, yaw)

        GAUSS_TO_TESLA = 1e-4

        now = self.get_clock().now().to_msg()

        header = Header()
        header.stamp = now
        header.frame_id = "IMU1_Frame"

        imu_msg = Imu()
        imu_msg.header = header
        imu_msg.orientation.x = qx
        imu_msg.orientation.y = qy
        imu_msg.orientation.z = qz
        imu_msg.orientation.w = qw
        imu_msg.angular_velocity.x = gyro_x
        imu_msg.angular_velocity.y = gyro_y
        imu_msg.angular_velocity.z = gyro_z
        imu_msg.linear_acceleration.x = accel_x
        imu_msg.linear_acceleration.y = accel_y
        imu_msg.linear_acceleration.z = accel_z

        mag_msg = MagneticField()
        mag_msg.header = header
        mag_msg.magnetic_field.x = mag_x_g * GAUSS_TO_TESLA
        mag_msg.magnetic_field.y = mag_y_g * GAUSS_TO_TESLA
        mag_msg.magnetic_field.z = mag_z_g * GAUSS_TO_TESLA

        out = IMUmsg()
        out.header = header
        out.imu = imu_msg
        out.mag_field = mag_msg
        out.raw = line

        self.imu_pub.publish(out)


def main(args=None):
    rclpy.init(args=args)
    imu_driver_node = IMUDriverNode()
    try:
        rclpy.spin(imu_driver_node)
    except KeyboardInterrupt:
        pass
    finally:
        imu_driver_node.port.close()
        imu_driver_node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

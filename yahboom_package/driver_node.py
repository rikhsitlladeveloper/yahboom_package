import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import String
from sensor_msgs.msg import Imu
import serial
import time
from yahboom_interfaces.msg import Status

class MotorDriverNode(Node):

    FUNC_PWM_VACUUM_START=0x01
    FUNC_PWM_VACUUM_STOP=0x02
    FUNC_PWM_VACUUM_SET=0x03
    FUNC_PWM_BRUSH_START=0x04
    FUNC_PWM_BRUSH_STOP=0x05
    FUNC_PWM_BRUSH_DIR=0x06
    FUNC_PWN_BRUSH_SET=0x07
    FUNC_PWM_MAIN_BRUSH_START=0x13
    FUNC_PWM_MAIN_BRUSH_STOP=0x14
    FUNC_PWN_MAIN_BRUSH_SET=0x15
    FUNC_HEAD=0xFF
    FUNC_DEVICE_ID=0xFC
    FUNC_IMU_READ_GYRO=0x0E
    FUNC_IMU_READ_ACCEL=0x0F
    FUNC_IMU_READ_MAGENTO=0x10
    FUNC_IMU_READ_ALL_IN_ONE=0x11
    FUNC_SERVO_MOTOR_CHANGE_DIRECTION=0x12

    def __init__(self):
        super().__init__('motor_driver_node')

        # Load parameters from YAML file
        self.declare_parameter('serial_port', '/dev/ttyROS_BOARD')
        self.declare_parameter('baudrate', 115200)

        serial_port = self.get_parameter('serial_port').get_parameter_value().string_value
        baudrate = self.get_parameter('baudrate').get_parameter_value().integer_value

        # Initialize serial connection
        self.ser = serial.Serial(serial_port, baudrate)
        timer_period = 0.1 
        self.timer = self.create_timer(timer_period, self.telemetry_callback)
        self.publisher_ = self.create_publisher(Imu, 'imu/data', 10)
        self.publisher_ = self.create_publisher(Status, 'motor/status', 10)
        # Create subscriber to /cmd_vel
        self.subscription = self.create_subscription(
            Twist,
            'cmd_vel',
            self.cmd_vel_callback,
            10)
        self.subscription  # prevent unused variable warning

        self.subscription_commands = self.create_subscription(
            String,
            'commands',
            self.commands_callback,
            10)
        self.subscription_commands
        self.imu_msg = Imu()
        self.status = Status()
    def telemetry_callback(self):
        if self.ser.in_waiting > 0:
            data = self.ser.readline()
            data_str = data.decode('utf-8').strip()
            parts = data_str.split()
            print(data_str)
            identifier = ' '.join(parts[:4])
            if identifier == '200 252 7 1':
                # IMU gyro
                gx, gy, gz = float(parts[4]), float(parts[5]), float(parts[6])
                self.imu_msg.angular_velocity.x = gx
                self.imu_msg.angular_velocity.y = gy
                self.imu_msg.angular_velocity.z = gz
            elif identifier == '200 252 7 3':
                # IMU acceleration
                ax, ay, az = float(parts[4]), float(parts[5]), float(parts[6])
                self.imu_msg.linear_acceleration.x = ax
                self.imu_msg.linear_acceleration.y = ay
                self.imu_msg.linear_acceleration.z = az
            elif identifier == '200 252 6 7':
                # Vacuum motor
                self.status.vacuum_motor = float(parts[4]), float(parts[5])
            elif identifier == '200 252 5 9':
                # Brush motor
                self.status.brush_motor = float(parts[4])
            elif identifier == '200 252 5 11':
                # Long Brush motor
                self.status.long_brush_motor = float(parts[4])
            elif identifier == '200 252 9 13 1':
                # Servo Left
                self.status.servo_motor[0] = float(parts[5])
            elif identifier == '200 252 9 13 2':
                # Servo Right
                self.status.servo_motor[1] = float(parts[5])
            

        self.imu_msg.header.stamp = self.get_clock().now().to_msg()
        self.imu_msg.header.frame_id = 'imu_link'           
        self.publisher_.publish(self.imu_msg)

    def make_packet(self, function_id, data=None):
        packet = [255, 252, 4, function_id]

        if data is not None:
            for i in data:
                hex_part = i.to_bytes(1, byteorder='big')
                packet.append(hex_part[0])
                packet[2] += 1

        return bytearray(packet)

    def send_packet(self, packet):
        print("Sending packet: ", packet)
        self.ser.write(packet)
        time.sleep(0.1)
        

    def set_side_brush(self, command, rpm):
        if command:
            self.send_packet(self.make_packet(self.FUNC_PWN_BRUSH_SET, [rpm]))
            self.send_packet(self.make_packet(self.FUNC_PWM_BRUSH_START, []))
        else:
            self.send_packet(self.make_packet(self.FUNC_PWM_BRUSH_STOP, []))
    
    def set_bottom_brush(self, command, rpm):
        if command:
            self.send_packet(self.make_packet(self.FUNC_PWN_MAIN_BRUSH_SET, [rpm]))
            self.send_packet(self.make_packet(self.FUNC_PWM_MAIN_BRUSH_START, []))
        else:
            self.send_packet(self.make_packet(self.FUNC_PWM_MAIN_BRUSH_STOP, []))

    def set_vacuum(self, command):
        if command:
            self.send_packet(self.make_packet(self.FUNC_PWM_VACUUM_START, []))
        else:
            self.send_packet(self.make_packet(self.FUNC_PWM_VACUUM_STOP, []))

    
    # def send_command(self, linear_x , angular_z):
    
    
    def commands_callback(self, msg):
        # Send received command directly to motor driver over serial
        if msg.data == "bottom_brush_on":
            self.set_bottom_brush(command=True, rpm=60)
        elif msg.data == "bottom_brush_off":
            self.set_bottom_brush(command=False, rpm=60)
        elif msg.data == "side_brush_on":
            self.set_side_brush(command=True, rpm=60)
        elif msg.data == "side_brush_off":
            self.set_side_brush(command=False, rpm=60)
        elif msg.data == "vacuum_on":
            self.set_vacuum(command=True)
        elif msg.data == "vacuum_off":
            self.set_vacuum(command=False)
        
        else:
            self.get_logger().info(f'Invalid string command: {msg.data}')
        
    def cmd_vel_callback(self, msg):
        # Convert Twist message to motor driver command
        linear_velocity = msg.linear.x
        angular_velocity = msg.angular.z
        
        
def main(args=None):
    rclpy.init(args=args)
    node = MotorDriverNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

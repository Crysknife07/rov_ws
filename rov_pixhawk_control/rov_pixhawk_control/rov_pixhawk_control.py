import rclpy
from rclpy import Node
from sensor_msgs.msg import Joy,Imu
from mavros_msgs.msg import OverrideRCIn
import math
import csv
import os
from datetime import datetime

class RovPixhawkControlNode(Node):
    def __init__(self):
        #informal name of nodes in ROS2
        super().__init__('rov_pixhawk_control_node')

        # =========config parameter==========
        self.cfg_left_joy_axis_fwd = 1       #left_joy_shangxia
        self.cfg_left_joy_axis_yaw = 2       #left_joy_zuoyou
        self.cfg_right_joy_axis_fwd = 3      #right_joy_shangxia
        self.cfg_right_joy_axis_yaw = 4      #right joy_zuoyou
        self.cfg_pwm_center = 1500
        self.cfg_pwm_range = 400
        self.cfg_log_interval = 10

        # ==========state ==============
        self.state_log_counter = 0

        # ===========(Pub/Sub)================
        #Publisher
        self.pub_rc_override = self.create_publisher(
            OverrideRCIn,
            '/mavros/rc/ovveride',
            10
        )

        #Subscriber:monitor joystick input
        self.sub_joy_cmd = self.create_subscription(
            Joy,
            '/joy',
            self.joy_callback,
            10
        )

        #Subscriber: monitor pixhawk attitude
        self.sub_imu_data = self.create_subscription(
            Imu,
            '/mavros/imu/data',
            self.imu_callback,
            10
        )

        #==================File/Writer================
        log_filename = f"rov_imu_log_{datetime.now().strftime('%Y%m%d_%H%M%S')}.csv"
        self.file_csv_log = open(log_filename, mode = 'w', newline = '')
        self.writer_csv_log = csv.writer(self.file_csv_log)
        self.write_csv_log.writerow(['Timestamp','Roll(deg)','Pitch(deg)','Yaw(deg)'])
        self.get_logger().info(f"ROV node active succeed, data been logged in :{os.path.abspath(log_filename)}")

    def joy_callback(self, msg_joy):

        msg_rc = OverrideRCIn()
        #initial all the 18 Channels to 65535
        msg_rc.channels = [65535] * 18

        #drag the data from the joystick
        left_axis_fwd_val = msg_joy.axes[self.cfg_left_joy_axis_fwd]
        left_axis_yaw_val = msg_joy.axes[self.cfg_left_joy_axis_yaw]
        right_axis_fwd_val = msg_joy.axes[self.cfg_right_joy_axis_fwd]
        right_axis_yaw_val = msg_joy.axes[self.cfg_right_joy_axis_yaw]

        #transfer joystick input into pwm input
        left_axis_pwm_fwd = int(self.cfg_pwm_center + (left_axis_fwd_val * self.cfg_pwm_range))
        left_axis_pwm_yaw = int(self.cfg_pwm_center + (left_axis_yaw_val * self.cfg_pwm_range))
        right_axis_pwm_fwd = int(self.cfg_pwm_center + (right_axis_fwd_val * self.cfg_pwm_range))
        right_axis_pwm_yaw = int(self.cfg_pwm_center + (right_axis_yaw_val * self.cfg_pwm_range))

        #put PWM to channels
        msg_rc.channels[4] = left_axis_pwm_fwd
        msg_rc.channels[5] = left_axis_pwm_yaw
        msg_rc.channels[6] = right_axis_pwm_fwd
        msg_rc.channels[7] = right_axis_pwm_yaw

        #send control data
        self.pub_rc_override.publish(msg_rc)

    def imu_callback(self, msg_imu):
        """receive attitude data, resolve and store"""
        q = msg_imu.orientation
        roll, pitch, yaw = self.quaternion_to_eular(q.x, q.y, q.z, q.w)

        self.state_log_counter += 1

        #when it reaches the settable time interval, log and print it
        if self.state_log_counter >= self.cfg_log_interval:
            timestamp = self.get_clock().now().to_msg().sec

            #print to csv
            self.writer_csv_log.writerow([timestamp, round(roll,2). round(pitch, 2), round(yaw, 2)])

            #print to controller
            self.get_logger().info(f'ROV zitai -> R:{roll:.1f}...')

            #reset the counter
            self.state_log_counter = 0

    def quaternion_to_euler(self, x, y, z, w):

        sinr_cosp = 2 * (w * x + y * z)
        cosr_cosp = 1 - 2 * (x *x + y * y)
        roll = math.atan2(sinr_cosp, cosr_cosp)

        sinp = math.sqrt(1 + 2 * (w * y - x * z))
        cosp = math.sqrt(1 - 2 * (w * y - x * z))
        pitch = 2 * math.atan2(sinp, cosp) - math.pi / 2

        siny_cosp = 2 * (w * z + x * y)
        cosy_cosp = 1 - 2 * (y * y + z * z)
        yaw = math.atan2(siny_cosp, cosy_cosp)

        return math.degrees(roll), math.degrees(pitch), math.degrees(yaw)
    
    def destroy_node(self):
        # clear work when the node shut dowm #
        self.file_csv_log.colse()
        self.get_logger().info("CSV has been safely saved and closed")
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = RovPixhawkControlNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
"""ROV Attitude Publisher — reads Pixhawk IMU via MAVROS and reports attitude.

Outputs:
  - /rov/attitude      (geometry_msgs/Vector3Stamped)  – ROS2 topic
  - UDP broadcast       (JSON)                          – non-ROS topside GUI
  - Local CSV log       (rov_attitude_log_*.csv)        – offline analysis
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Vector3Stamped, Vector3
from std_msgs.msg import Header
from .utils import quaternion_to_euler

import csv
import json
import math
import socket
from datetime import datetime


class AttitudePublisherNode(Node):
    """Publish ROV attitude (roll/pitch/yaw) from /mavros/imu/data."""

    def __init__(self):
        super().__init__('attitude_publisher_node')

        # ==================== Publish Rate ====================
        self.cfg_publish_rate = 50.0  # Hz (每 0.02 s 发一帧)

        # ==================== UDP Config ====================
        self.cfg_enable_udp  = True
        self.cfg_udp_host    = '255.255.255.255'   # 广播地址
        self.cfg_udp_port    = 5005

        # ==================== CSV Logging Config ====================
        self.cfg_csv_interval = 10   # 每 N 帧写一行 CSV

        # ==================== State ====================
        self.state_frame_count   = 0
        self.state_latest_imu    = None

        # ==================== UDP Socket ====================
        if self.cfg_enable_udp:
            self._sock_udp = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            self._sock_udp.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
            self.get_logger().info(
                f"UDP broadcast → {self.cfg_udp_host}:{self.cfg_udp_port}"
            )

        # ==================== Publishers ====================
        self.pub_attitude = self.create_publisher(
            Vector3Stamped,
            '/rov/attitude',
            10
        )

        # ==================== Subscribers ====================
        self.sub_imu = self.create_subscription(
            Imu,
            '/mavros/imu/data',
            self.imu_callback,
            10
        )

        # ==================== CSV Logging ====================
        log_filename = f"rov_attitude_log_{datetime.now().strftime('%Y%m%d_%H%M%S')}.csv"
        self.file_csv = open(log_filename, mode='w', newline='')
        self.writer_csv = csv.writer(self.file_csv)
        self.writer_csv.writerow([
            'Timestamp', 'Roll(deg)', 'Pitch(deg)', 'Yaw(deg)',
            'Roll(rad)', 'Pitch(rad)', 'Yaw(rad)'
        ])
        self.get_logger().info(
            f"Attitude publisher active, log → {log_filename}"
        )

        # ==================== Timer (fixed-rate publish) ====================
        period = 1.0 / self.cfg_publish_rate
        self.timer = self.create_timer(period, self.timer_callback)

    # =====================================================================
    #  Store latest IMU sample
    # =====================================================================

    def imu_callback(self, msg_imu):
        """Buffer the latest IMU message (processed on timer)."""
        self.state_latest_imu = msg_imu

    # =====================================================================
    #  Fixed-rate publish (ROS2 topic + UDP + CSV)
    # =====================================================================

    def timer_callback(self):
        """Convert latest quaternion to Euler, publish, broadcast, log."""
        if self.state_latest_imu is None:
            return  # no IMU data yet

        q   = self.state_latest_imu.orientation
        now = self.get_clock().now()

        roll_deg, pitch_deg, yaw_deg = quaternion_to_euler(q.x, q.y, q.z, q.w)
        roll_rad  = math.radians(roll_deg)
        pitch_rad = math.radians(pitch_deg)
        yaw_rad   = math.radians(yaw_deg)

        # ---- ROS2 topic ----
        msg = Vector3Stamped()
        msg.header = Header(stamp=now.to_msg(), frame_id='rov_attitude')
        msg.vector = Vector3(x=roll_deg, y=pitch_deg, z=yaw_deg)
        self.pub_attitude.publish(msg)

        # ---- UDP broadcast (JSON) ----
        if self.cfg_enable_udp:
            payload = json.dumps({
                'ts':        now.nanoseconds / 1e9,
                'roll':      roll_deg,
                'pitch':     pitch_deg,
                'yaw':       yaw_deg,
                'roll_rad':  roll_rad,
                'pitch_rad': pitch_rad,
                'yaw_rad':   yaw_rad,
            })
            try:
                self._sock_udp.sendto(
                    payload.encode('utf-8'),
                    (self.cfg_udp_host, self.cfg_udp_port)
                )
            except OSError:
                pass  # network not ready — non-critical

        # ---- CSV log (throttled) ----
        self.state_frame_count += 1
        if self.state_frame_count >= self.cfg_csv_interval:
            self.writer_csv.writerow([
                now.nanoseconds / 1e9,
                round(roll_deg, 2), round(pitch_deg, 2), round(yaw_deg, 2),
                round(roll_rad, 4), round(pitch_rad, 4), round(yaw_rad, 4),
            ])
            self.state_frame_count = 0

    # =====================================================================
    #  Cleanup
    # =====================================================================

    def destroy_node(self):
        self.file_csv.close()
        if self.cfg_enable_udp:
            self._sock_udp.close()
        self.get_logger().info("Attitude publisher shut down")
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = AttitudePublisherNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

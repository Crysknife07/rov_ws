import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy, Imu
from mavros_msgs.msg import OverrideRCIn
from .utils import apply_dead_zone, cmd_to_pwm, quaternion_to_euler
import csv
import os
from datetime import datetime


# =========================================================================
#  Xbox Button Index Reference (sensor_msgs/Joy.buttons)
# =========================================================================
BTN_A      = 0
BTN_B      = 1
BTN_X      = 2
BTN_Y      = 3
BTN_LB     = 4
BTN_RB     = 5
BTN_BACK   = 6
BTN_START  = 7
BTN_XBOX   = 8
BTN_LSTICK = 9
BTN_RSTICK = 10

# =========================================================================
#  Operation Modes
# =========================================================================
MODE_MANUAL    = 0   # 默认手动：摇杆直驱推进器
MODE_AUX       = 1   # 辅助设备：灯开关 + 舵机控制
MODE_ATT_HOLD  = 2   # 定姿保持（预留框架）
MODE_DEPTH_HOLD = 3  # 定深保持（预留框架）

MODE_NAMES = {
    MODE_MANUAL:    "Manual",
    MODE_AUX:       "Auxiliary (Lights & Servo)",
    MODE_ATT_HOLD:  "Attitude Hold (reserved)",
    MODE_DEPTH_HOLD: "Depth Hold (reserved)",
}


class RovPixhawkControlNode(Node):
    """ROV thruster control node — joystick → thruster allocation matrix → Pixhawk RC override.

    Xbox controller with mode-switching support:
      - Mode 0 (Manual):   axes → thruster mixing (default)
      - Mode 1 (Aux):      buttons → lights toggle + servo hold-to-rotate
      - Mode 2-3:          reserved for attitude/depth hold
    """

    def __init__(self):
        super().__init__('rov_pixhawk_control_node')

        # ==================== Joystick Axis Mapping ====================
        self.cfg_axis_surge      = 1   # 左杆上下 → 进退
        self.cfg_axis_yaw        = 2   # 左杆左右 → 转艏
        self.cfg_axis_pitch      = 3   # 右杆上下 → 俯仰
        self.cfg_axis_roll       = 4   # 右杆左右 → 横滚
        self.cfg_axis_heave_up   = 5   # LT → 上浮 (1=released, -1=pressed)
        self.cfg_axis_heave_down = 6   # RT → 下潜 (1=released, -1=pressed)

        # ==================== Thruster Channel Mapping ====================
        # T1-T4: 垂直推进器 (带 ±30° 倾角)
        self.cfg_chn_vert_fl = 4        # T1 垂直左前
        self.cfg_chn_vert_fr = 5        # T2 垂直右前
        self.cfg_chn_vert_rl = 6        # T3 垂直左后
        self.cfg_chn_vert_rr = 7        # T4 垂直右后
        # T5-T6: 前进推进器 (水平)
        self.cfg_chn_fwd_l   = 8        # T5 前进左
        self.cfg_chn_fwd_r   = 9        # T6 前进右

        # ==================== Mixing Gains ====================
        self.cfg_gain_surge = 1.0
        self.cfg_gain_yaw   = 1.0
        self.cfg_gain_pitch = 1.0
        self.cfg_gain_roll  = 1.0
        self.cfg_gain_heave = 1.0

        # ==================== PWM Config ====================
        self.cfg_pwm_center   = 1500
        self.cfg_pwm_range    = 400
        self.cfg_pwm_min      = 1100
        self.cfg_pwm_max      = 1900
        self.cfg_dead_zone    = 0.05

        # ==================== Auxiliary Device Config ====================
        self.cfg_servo_channel    = 12      # MAIN OUT 13 (MAVLink index 12)
        self.cfg_light1_channel   = 13      # MAIN OUT 14 (MAVLink index 13)
        self.cfg_light2_channel   = 13      # 暂共用 ch13，后续改 14
        self.cfg_servo_rate       = 5       # 每帧 PWM 步长 (μs)
        self.cfg_servo_pwm_min    = 1100    # 舵机限幅
        self.cfg_servo_pwm_max    = 1900

        # ==================== Logging Config ====================
        self.cfg_log_interval = 10          # 每 N 帧记录一次 IMU 数据

        # ==================== Mode State ====================
        self.state_current_mode = MODE_MANUAL
        self.state_light1_on    = False
        self.state_light2_on    = False
        self.state_servo_pwm    = 1500       # 舵机当前 PWM

        # ==================== Edge Detection ====================
        self.state_prev_buttons = [0] * 15  # 上一帧按键状态
        self.state_log_counter  = 0

        # ==================== Publishers ====================
        self.pub_rc_override = self.create_publisher(
            OverrideRCIn,
            '/mavros/rc/override',
            10
        )

        # ==================== Subscribers ====================
        self.sub_joy_cmd = self.create_subscription(
            Joy,
            '/joy',
            self.joy_callback,
            10
        )

        self.sub_imu_data = self.create_subscription(
            Imu,
            '/mavros/imu/data',
            self.imu_callback,
            10
        )

        # ==================== CSV Logging ====================
        log_filename = f"rov_imu_log_{datetime.now().strftime('%Y%m%d_%H%M%S')}.csv"
        self.file_csv_log = open(log_filename, mode='w', newline='')
        self.writer_csv_log = csv.writer(self.file_csv_log)
        self.writer_csv_log.writerow(
            ['Timestamp', 'Roll(deg)', 'Pitch(deg)', 'Yaw(deg)', 'Mode']
        )
        self.get_logger().info(
            f"ROV node active, logging to: {os.path.abspath(log_filename)}"
        )

    # =====================================================================
    #  Edge Detector
    # =====================================================================

    def _edge_rising(self, btn_index):
        """Return True on the rising edge (0→1) of a button."""
        prev = self.state_prev_buttons[btn_index]
        return prev == 0 and self._cur_buttons[btn_index] == 1

    # =====================================================================
    #  Mode Switch
    # =====================================================================

    def _handle_mode_switch(self):
        """Cycle to the next mode on Back button rising edge."""
        if self._edge_rising(BTN_BACK):
            self.state_current_mode = (self.state_current_mode + 1) % 4
            name = MODE_NAMES.get(self.state_current_mode, "Unknown")
            self.get_logger().info(f"Mode → {self.state_current_mode} ({name})")

    # =====================================================================
    #  Mode 1: Auxiliary Device Control
    # =====================================================================

    def _handle_aux_mode(self, msg_rc):
        """Handle lights and servo control when in Mode 1."""

        # --- Light 1 toggle (A button, edge-triggered) ---
        if self._edge_rising(BTN_A):
            self.state_light1_on = not self.state_light1_on
            pwm = self.cfg_pwm_max if self.state_light1_on else self.cfg_pwm_min
            msg_rc.channels[self.cfg_light1_channel] = pwm
            self.get_logger().info(
                f"Light 1 → {'ON' if self.state_light1_on else 'OFF'} "
                f"(ch{self.cfg_light1_channel}, PWM={pwm})"
            )

        # --- Light 2 toggle (B button, edge-triggered) ---
        if self._edge_rising(BTN_B):
            self.state_light2_on = not self.state_light2_on
            pwm = self.cfg_pwm_max if self.state_light2_on else self.cfg_pwm_min
            msg_rc.channels[self.cfg_light2_channel] = pwm
            self.get_logger().info(
                f"Light 2 → {'ON' if self.state_light2_on else 'OFF'} "
                f"(ch{self.cfg_light2_channel}, PWM={pwm})"
            )

        # --- Servo hold-to-rotate (X / Y, level-triggered) ---
        if self._cur_buttons[BTN_X] == 1:
            self.state_servo_pwm += self.cfg_servo_rate
            if self.state_servo_pwm > self.cfg_servo_pwm_max:
                self.state_servo_pwm = self.cfg_servo_pwm_max
            msg_rc.channels[self.cfg_servo_channel] = self.state_servo_pwm

        if self._cur_buttons[BTN_Y] == 1:
            self.state_servo_pwm -= self.cfg_servo_rate
            if self.state_servo_pwm < self.cfg_servo_pwm_min:
                self.state_servo_pwm = self.cfg_servo_pwm_min
            msg_rc.channels[self.cfg_servo_channel] = self.state_servo_pwm

        # --- Servo centre (LB, edge-triggered) ---
        if self._edge_rising(BTN_LB):
            self.state_servo_pwm = self.cfg_pwm_center
            msg_rc.channels[self.cfg_servo_channel] = self.cfg_pwm_center
            self.get_logger().info("Servo → centre (PWM=1500)")

    # =====================================================================
    #  Persist auxiliary channel values across frames
    # =====================================================================

    def _persist_aux_channels(self, msg_rc):
        """Always write the current aux channel states to the RC message.

        This ensures lights stay on/off and servos hold position across
        frames, even when no button is currently pressed.
        """
        # Lights
        if self.state_light1_on:
            msg_rc.channels[self.cfg_light1_channel] = self.cfg_pwm_max
        else:
            msg_rc.channels[self.cfg_light1_channel] = self.cfg_pwm_min

        # TODO: when light2 gets its own channel, uncomment below
        # if self.state_light2_on:
        #     msg_rc.channels[self.cfg_light2_channel] = self.cfg_pwm_max
        # else:
        #     msg_rc.channels[self.cfg_light2_channel] = self.cfg_pwm_min

        # Servo hold position
        msg_rc.channels[self.cfg_servo_channel] = self.state_servo_pwm

    # =====================================================================
    #  Thruster Allocation Matrix
    # =====================================================================

    def joy_callback(self, msg_joy):
        # Preserve button snapshot for edge detection
        if len(msg_joy.buttons) >= 11:
            self._cur_buttons = list(msg_joy.buttons)
        else:
            # Pad short button arrays (defensive)
            self._cur_buttons = list(msg_joy.buttons) + [0] * (15 - len(msg_joy.buttons))

        # --- Step 0: prepare RC override message ---
        msg_rc = OverrideRCIn()
        msg_rc.channels = [65535] * 18    # 65535 = MAVLink "do not override"

        # --- Step 1: mode switch handling ---
        self._handle_mode_switch()

        # --- Step 2: read axes through dead-zone ---
        surge_raw = apply_dead_zone(
            msg_joy.axes[self.cfg_axis_surge], self.cfg_dead_zone)
        yaw_raw   = apply_dead_zone(
            msg_joy.axes[self.cfg_axis_yaw], self.cfg_dead_zone)
        pitch_raw = apply_dead_zone(
            msg_joy.axes[self.cfg_axis_pitch], self.cfg_dead_zone)
        roll_raw  = apply_dead_zone(
            msg_joy.axes[self.cfg_axis_roll], self.cfg_dead_zone)

        # LT / RT: 1.0 = released, -1.0 = fully pressed
        lt_raw = apply_dead_zone(
            msg_joy.axes[self.cfg_axis_heave_up], self.cfg_dead_zone)
        rt_raw = apply_dead_zone(
            msg_joy.axes[self.cfg_axis_heave_down], self.cfg_dead_zone)

        # --- Step 3: compute DOF commands ---
        surge_cmd = self.cfg_gain_surge * surge_raw
        yaw_cmd   = self.cfg_gain_yaw   * yaw_raw
        pitch_cmd = self.cfg_gain_pitch * pitch_raw
        roll_cmd  = self.cfg_gain_roll  * roll_raw

        # LT: released=1→heave=0, pressed=-1→heave=+1 (上浮)
        heave_up   = max(0.0, (-lt_raw + 1.0) / 2.0)
        # RT: released=1→heave=0, pressed=-1→heave=-1 (下潜)
        heave_down = max(0.0, (-rt_raw + 1.0) / 2.0)
        heave_cmd  = self.cfg_gain_heave * (heave_up - heave_down)

        # --- Step 4: mixing matrix ---
        # Forward thrusters (T5, T6): surge + differential yaw
        t5 = surge_cmd + yaw_cmd   # 左前进
        t6 = surge_cmd - yaw_cmd   # 右前进

        # Vertical thrusters (T1-T4): heave + pitch (front/rear) + roll (left/right)
        t1 = heave_cmd + pitch_cmd + roll_cmd   # T1 垂直左前
        t2 = heave_cmd + pitch_cmd - roll_cmd   # T2 垂直右前
        t3 = heave_cmd - pitch_cmd + roll_cmd   # T3 垂直左后
        t4 = heave_cmd - pitch_cmd - roll_cmd   # T4 垂直右后

        # Clamp all thruster commands to [-1, 1] to prevent wind-up
        t1 = max(-1.0, min(1.0, t1))
        t2 = max(-1.0, min(1.0, t2))
        t3 = max(-1.0, min(1.0, t3))
        t4 = max(-1.0, min(1.0, t4))
        t5 = max(-1.0, min(1.0, t5))
        t6 = max(-1.0, min(1.0, t6))

        # --- Step 5: convert to PWM and assign thruster channels ---
        msg_rc.channels[self.cfg_chn_vert_fl] = cmd_to_pwm(
            t1, self.cfg_pwm_center, self.cfg_pwm_range,
            self.cfg_pwm_min, self.cfg_pwm_max)
        msg_rc.channels[self.cfg_chn_vert_fr] = cmd_to_pwm(
            t2, self.cfg_pwm_center, self.cfg_pwm_range,
            self.cfg_pwm_min, self.cfg_pwm_max)
        msg_rc.channels[self.cfg_chn_vert_rl] = cmd_to_pwm(
            t3, self.cfg_pwm_center, self.cfg_pwm_range,
            self.cfg_pwm_min, self.cfg_pwm_max)
        msg_rc.channels[self.cfg_chn_vert_rr] = cmd_to_pwm(
            t4, self.cfg_pwm_center, self.cfg_pwm_range,
            self.cfg_pwm_min, self.cfg_pwm_max)
        msg_rc.channels[self.cfg_chn_fwd_l]   = cmd_to_pwm(
            t5, self.cfg_pwm_center, self.cfg_pwm_range,
            self.cfg_pwm_min, self.cfg_pwm_max)
        msg_rc.channels[self.cfg_chn_fwd_r]   = cmd_to_pwm(
            t6, self.cfg_pwm_center, self.cfg_pwm_range,
            self.cfg_pwm_min, self.cfg_pwm_max)

        # --- Step 6: auxiliary device control (Mode 1) ---
        if self.state_current_mode == MODE_AUX:
            self._handle_aux_mode(msg_rc)

        # --- Step 7: persist auxiliary channel states ---
        self._persist_aux_channels(msg_rc)

        # --- Step 8: publish ---
        self.pub_rc_override.publish(msg_rc)

        # --- Step 9: save button state for next-frame edge detection ---
        self.state_prev_buttons = list(self._cur_buttons)

    # =====================================================================
    #  IMU / Attitude Logging
    # =====================================================================

    def imu_callback(self, msg_imu):
        """Receive attitude data, convert to Euler angles, and log periodically."""
        q = msg_imu.orientation
        roll, pitch, yaw = quaternion_to_euler(q.x, q.y, q.z, q.w)

        self.state_log_counter += 1

        if self.state_log_counter >= self.cfg_log_interval:
            timestamp = self.get_clock().now().nanoseconds / 1e9

            # Write to CSV
            self.writer_csv_log.writerow(
                [timestamp, round(roll, 2), round(pitch, 2), round(yaw, 2),
                 self.state_current_mode]
            )

            # Print to console
            mode_name = MODE_NAMES.get(self.state_current_mode, "?")
            self.get_logger().info(
                f'Attitude → R:{roll:.1f}  P:{pitch:.1f}  Y:{yaw:.1f}  [{mode_name}]'
            )

            self.state_log_counter = 0

    # =====================================================================
    #  Cleanup
    # =====================================================================

    def destroy_node(self):
        """Safely close CSV log file before shutdown."""
        self.file_csv_log.close()
        self.get_logger().info("CSV log saved and closed")
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

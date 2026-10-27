#!/usr/bin/env python3
import time
import math
import subprocess
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Twist

class RobotBehaviorNode(Node):
    """
    Finite State Machine controlling TurtleBot 4 behaviors based on hand gestures:
    - Wave: Greeting behavior (turn left-right)
    - Pointing Up: Select forward-backward 4x (1.2m each way)
    - Victory: Select circular trajectory (full 360 degree revolution)
    - Thumbs Up: Confirm and execute selected behavior
    - Open Palm: Immediate Emergency Stop
    - Closed Fist: Cancel current behavior and return to IDLE
    """

    STATE_IDLE = "IDLE (CHO CHON CU CHI)"
    STATE_PENDING_FWD_BWD = "PENDING: TIEN-LUI 4 LAN (Cho Thumbs Up)"
    STATE_PENDING_CIRCLE = "PENDING: QUY DAO TRON 360 (Cho Thumbs Up)"
    STATE_PENDING_WAVE = "PENDING: VAY TAY CHAO LAI (Cho Thumbs Up)"
    STATE_EXECUTING_FWD_BWD = "DANG CHAY: TIEN-LUI 4 LAN"
    STATE_EXECUTING_CIRCLE = "DANG CHAY: QUY DAO TRON 360"
    STATE_EXECUTING_WAVE = "DANG CHAY: VAY TAY CHAO LAI"
    STATE_EMERGENCY_STOP = "EMERGENCY STOP (DUNG KHAN CAP)"

    def __init__(self):
        super().__init__('robot_behavior_node')

        # Parameters
        if not self.has_parameter('use_sim_time'):
            try:
                self.declare_parameter('use_sim_time', True)
            except Exception:
                pass
        self.declare_parameter('linear_speed', 0.35)
        self.declare_parameter('angular_speed', 0.50)
        self.declare_parameter('watchdog_timeout', 15.0)

        self.linear_speed = self.get_parameter('linear_speed').value
        self.angular_speed = self.get_parameter('angular_speed').value
        self.watchdog_timeout = self.get_parameter('watchdog_timeout').value

        # Publishers: Both /cmd_vel and /diffdrive_controller/cmd_vel_unstamped
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.diff_vel_pub = self.create_publisher(Twist, '/diffdrive_controller/cmd_vel_unstamped', 10)
        self.status_pub = self.create_publisher(String, '/robot_behavior_status', 10)

        # Subscriber
        self.gesture_sub = self.create_subscription(
            String,
            '/detected_gesture',
            self.gesture_callback,
            10
        )

        # State machine
        self.state = self.STATE_IDLE
        self.pending_behavior = None
        self.last_gesture_time = self._get_time()
        self.cooldown_until = 0.0

        # Execution tracking
        self.exec_start_time = 0.0
        self.fwd_bwd_cycle = 0
        self.fwd_bwd_total_cycles = 4
        self.fwd_bwd_phase = 0  # 0: fwd, 1: pause, 2: bwd, 3: pause
        self.phase_start_time = 0.0

        # Periodic control loop timer at 20 Hz (50 ms)
        self.timer = self.create_timer(0.05, self.control_loop)
        self.get_logger().info('=====================================================')
        self.get_logger().info('-> robot_behavior_node SAN SANG!')
        self.get_logger().info(f'-> Toc do: linear={self.linear_speed} m/s, angular={self.angular_speed} rad/s')
        self.get_logger().info('=====================================================')
        self._publish_status()

    def _get_time(self):
        nanos = self.get_clock().now().nanoseconds
        if nanos > 1000000:
            return nanos / 1e9
        return time.time()

    def gesture_callback(self, msg):
        gesture = msg.data.strip()
        now = self._get_time()
        self.last_gesture_time = now

        # Cooldown check to prevent noisy jitter triggers AND allow hand lower transition
        if now < self.cooldown_until:
            return

        # 1. Open_Palm: Immediate Emergency Stop from any state
        if gesture == "Open_Palm":
            if self.state != self.STATE_EMERGENCY_STOP:
                self.get_logger().warn('>>> [CU CHI: OPEN PALM] !!! DUNG KHAN CAP (EMERGENCY STOP) !!!')
                self._stop_robot()
                self.state = self.STATE_EMERGENCY_STOP
                self.pending_behavior = None
                self.cooldown_until = now + 1.2
                self._publish_status()
            return

        # 2. Closed_Fist: Cancel current action & reset to IDLE
        if gesture == "Closed_Fist":
            if self.state != self.STATE_IDLE:
                self.get_logger().info('>>> [CU CHI: CLOSED FIST] HUY HANH VI -> Tro ve trang thai cho (IDLE).')
                self._stop_robot()
                self.state = self.STATE_IDLE
                self.pending_behavior = None
                self.cooldown_until = now + 1.2
                self._publish_status()
            return

        # 3. State: IDLE or EMERGENCY_STOP -> Select behavior
        if self.state in [self.STATE_IDLE, self.STATE_EMERGENCY_STOP]:
            if gesture == "Pointing_Up":
                self.state = self.STATE_PENDING_FWD_BWD
                self.pending_behavior = "FWD_BWD"
                self.cooldown_until = now + 1.2
                self.get_logger().info('>>> [CU CHI: POINTING UP] Da chon: TIEN-LUI 4 LAN!')
                self.get_logger().info('>>> [HUONG DAN] Hay gio ngon cai (THUMBS UP) de robot bat dau chay!')
                self._publish_status()
            elif gesture == "Victory":
                self.state = self.STATE_PENDING_CIRCLE
                self.pending_behavior = "CIRCLE"
                self.cooldown_until = now + 1.2
                self.get_logger().info('>>> [CU CHI: VICTORY] Da chon: QUY DAO TRON 360 DO!')
                self.get_logger().info('>>> [HUONG DAN] Hay gio ngon cai (THUMBS UP) de robot bat dau chay!')
                self._publish_status()
            elif gesture == "Wave":
                self.state = self.STATE_PENDING_WAVE
                self.pending_behavior = "WAVE"
                self.cooldown_until = now + 1.2
                self.get_logger().info('>>> [CU CHI: WAVE] Da chon: VAY TAY CHAO LAI!')
                self.get_logger().info('>>> [HUONG DAN] Hay gio ngon cai (THUMBS UP) de robot bat dau chay!')
                self._publish_status()

        elif self.state in [self.STATE_PENDING_FWD_BWD, self.STATE_PENDING_CIRCLE, self.STATE_PENDING_WAVE]:
            if gesture == "Thumbs_Up":
                self.get_logger().info('=====================================================')
                self.get_logger().info('>>> [CU CHI: THUMBS UP] DA XAC NHAN! ROBOT BAT DAU DI CHUYEN! <<<')
                self.get_logger().info('=====================================================')
                # 2.0s grace cooldown so user can lower hand without false triggering Open_Palm
                self.cooldown_until = now + 2.0
                self._start_execution()
            elif gesture == "Pointing_Up" and self.state != self.STATE_PENDING_FWD_BWD:
                self.state = self.STATE_PENDING_FWD_BWD
                self.pending_behavior = "FWD_BWD"
                self.cooldown_until = now + 1.0
                self.get_logger().info('-> Doi sang lua chon: TIEN-LUI 4 LAN. Cho Thumbs Up...')
                self._publish_status()
            elif gesture == "Victory" and self.state != self.STATE_PENDING_CIRCLE:
                self.state = self.STATE_PENDING_CIRCLE
                self.pending_behavior = "CIRCLE"
                self.cooldown_until = now + 1.0
                self.get_logger().info('-> Doi sang lua chon: QUY DAO TRON 360 DO. Cho Thumbs Up...')
                self._publish_status()

    def _start_execution(self):
        now = self._get_time()
        self.exec_start_time = now
        self.phase_start_time = now

        if self.pending_behavior == "FWD_BWD":
            self.state = self.STATE_EXECUTING_FWD_BWD
            self.fwd_bwd_cycle = 1
            self.fwd_bwd_phase = 0  # Phase 0: Forward
            self.get_logger().info(f'-> [Chu ky 1/{self.fwd_bwd_total_cycles}] Dang TIEN ve phia truoc (v={self.linear_speed} m/s)...')
        elif self.pending_behavior == "CIRCLE":
            self.state = self.STATE_EXECUTING_CIRCLE
            self.get_logger().info(f'-> Dang di chuyen theo QUY DAO TRON 360 DO (v={self.linear_speed} m/s, w={self.angular_speed} rad/s)...')
        elif self.pending_behavior == "WAVE":
            self.state = self.STATE_EXECUTING_WAVE
            self.get_logger().info('-> Dang thuc hien hanh vi CHAO LAI (Xoay trai - phai)...')

        self._publish_status()

    def control_loop(self):
        now = self._get_time()

        # Watchdog check: only if node hasn't received ANY communication in watchdog_timeout
        if self.state in [self.STATE_EXECUTING_FWD_BWD, self.STATE_EXECUTING_CIRCLE, self.STATE_EXECUTING_WAVE]:
            if (now - self.last_gesture_time) > self.watchdog_timeout:
                self.get_logger().warn(f'Mat tin hieu tu camera qua {self.watchdog_timeout}s! Robot tu dong dung an toan.')
                self._stop_robot()
                self.state = self.STATE_IDLE
                self._publish_status()
                return

        # Execute active behaviors
        if self.state == self.STATE_EXECUTING_FWD_BWD:
            self._execute_forward_backward(now)
        elif self.state == self.STATE_EXECUTING_CIRCLE:
            self._execute_circle(now)
        elif self.state == self.STATE_EXECUTING_WAVE:
            self._execute_wave(now)
        else:
            self._send_cmd(0.0, 0.0)

    def _execute_forward_backward(self, now):
        """
        4 cycles of clear Forward & Backward motion:
        Phase 0: Forward (linear.x = self.linear_speed, 3.5s -> ~1.22m)
        Phase 1: Pause (0.5s)
        Phase 2: Backward (linear.x = -self.linear_speed, 3.5s -> ~1.22m)
        Phase 3: Pause (0.5s)
        """
        elapsed_in_phase = now - self.phase_start_time
        fwd_duration = 3.5
        pause_duration = 0.5
        bwd_duration = 3.5

        if self.fwd_bwd_phase == 0:
            if elapsed_in_phase < fwd_duration:
                self._send_cmd(self.linear_speed, 0.0)
            else:
                self._send_cmd(0.0, 0.0)
                self.fwd_bwd_phase = 1
                self.phase_start_time = now
        elif self.fwd_bwd_phase == 1:
            if elapsed_in_phase < pause_duration:
                self._send_cmd(0.0, 0.0)
            else:
                self.fwd_bwd_phase = 2
                self.phase_start_time = now
                self.get_logger().info(f'-> [Chu ky {self.fwd_bwd_cycle}/{self.fwd_bwd_total_cycles}] Dang LUI ve sau (v=-{self.linear_speed} m/s)...')
        elif self.fwd_bwd_phase == 2:
            if elapsed_in_phase < bwd_duration:
                self._send_cmd(-self.linear_speed, 0.0)
            else:
                self._send_cmd(0.0, 0.0)
                self.fwd_bwd_phase = 3
                self.phase_start_time = now
        elif self.fwd_bwd_phase == 3:
            if elapsed_in_phase < pause_duration:
                self._send_cmd(0.0, 0.0)
            else:
                if self.fwd_bwd_cycle < self.fwd_bwd_total_cycles:
                    self.fwd_bwd_cycle += 1
                    self.fwd_bwd_phase = 0
                    self.phase_start_time = now
                    self.get_logger().info(f'-> [Chu ky {self.fwd_bwd_cycle}/{self.fwd_bwd_total_cycles}] Dang TIEN ve phia truoc (v={self.linear_speed} m/s)...')
                else:
                    self.get_logger().info('=====================================================')
                    self.get_logger().info('-> HOAN THANH XUAT SAC 4 CHU KY TIEN-LUI! Tro ve trang thai cho.')
                    self.get_logger().info('=====================================================')
                    self._stop_robot()
                    self.state = self.STATE_IDLE
                    self.pending_behavior = None
                    self._publish_status()

    def _execute_circle(self, now):
        """
        Executes circular trajectory:
        linear.x = self.linear_speed, angular.z = self.angular_speed
        Duration: Exactly 1 full revolution 360 degrees (2*pi / angular_speed)
        """
        elapsed = now - self.exec_start_time
        circle_duration = (2.0 * math.pi) / max(0.01, abs(self.angular_speed))

        if elapsed < circle_duration:
            self._send_cmd(self.linear_speed, self.angular_speed)
        else:
            self.get_logger().info('=====================================================')
            self.get_logger().info('-> HOAN THANH QUY DAO TRON 360 DO! Tro ve trang thai cho.')
            self.get_logger().info('=====================================================')
            self._stop_robot()
            self.state = self.STATE_IDLE
            self.pending_behavior = None
            self._publish_status()

    def _execute_wave(self, now):
        """
        Wave Greeting Behavior:
        Rotates left (+w for 1.0s) -> right (-w for 2.0s) -> left back to zero (+w for 1.0s)
        Total 4.0s. Net rotation = 0 rad.
        """
        elapsed = now - self.exec_start_time
        w = self.angular_speed

        if elapsed < 1.0:
            self._send_cmd(0.0, w)
        elif elapsed < 3.0:
            self._send_cmd(0.0, -w)
        elif elapsed < 4.0:
            self._send_cmd(0.0, w)
        else:
            self.get_logger().info('=====================================================')
            self.get_logger().info('-> HOAN THANH HANH VI CHAO LAI (WAVE GREETING)! Tro ve trang thai cho.')
            self.get_logger().info('=====================================================')
            self._stop_robot()
            self.state = self.STATE_IDLE
            self.pending_behavior = None
            self._publish_status()

    def _send_cmd(self, linear_x, angular_z):
        twist = Twist()
        twist.linear.x = float(linear_x)
        twist.angular.z = float(angular_z)
        self.cmd_vel_pub.publish(twist)
        self.diff_vel_pub.publish(twist)

    def _stop_robot(self):
        self._send_cmd(0.0, 0.0)

    def _publish_status(self):
        msg = String()
        msg.data = self.state
        self.status_pub.publish(msg)

    def destroy_node(self):
        self._stop_robot()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = RobotBehaviorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

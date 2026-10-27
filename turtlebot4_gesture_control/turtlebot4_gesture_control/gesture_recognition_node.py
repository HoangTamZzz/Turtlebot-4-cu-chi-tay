#!/usr/bin/env python3
import time
import os
import subprocess
import cv2
import numpy as np

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

from turtlebot4_gesture_control.gesture_classifier import GestureClassifier

class GestureRecognitionNode(Node):
    """
    ROS 2 Node for Hand Gesture Recognition using MediaPipe.
    Purely vision-based: reads from camera, classifies gestures, and publishes /detected_gesture.
    Enhanced with Auto-Brightness optimization and robust debouncing filter.
    """

    def __init__(self):
        super().__init__('gesture_recognition_node')

        # Parameters
        self.declare_parameter('camera_id', 0)
        self.declare_parameter('video_source', '')
        self.declare_parameter('confidence_threshold', 0.55)
        self.declare_parameter('debounce_frames', 5)
        self.declare_parameter('show_gui', True)
        self.declare_parameter('model_path', '')

        self.camera_id = self.get_parameter('camera_id').value
        self.video_source = self.get_parameter('video_source').value
        self.confidence_threshold = self.get_parameter('confidence_threshold').value
        self.debounce_frames = self.get_parameter('debounce_frames').value
        self.show_gui = self.get_parameter('show_gui').value
        model_path_param = self.get_parameter('model_path').value
        model_path = model_path_param if model_path_param else None

        # Publishers
        self.gesture_pub = self.create_publisher(String, '/detected_gesture', 10)
        self.image_pub = self.create_publisher(Image, '/camera/gesture_annotated', 10)

        # Subscriber for robot behavior status feedback
        self.robot_status = "IDLE (CHO CHON CU CHI)"
        self.status_sub = self.create_subscription(
            String,
            '/robot_behavior_status',
            self.robot_status_callback,
            10
        )

        self.bridge = CvBridge()

        # Initialize AI Gesture Classifier
        self.get_logger().info('Kh?i t?o MediaPipe Gesture Classifier...')
        self.classifier = GestureClassifier(model_path=model_path)

        # Debouncing filter variables
        self.last_raw_gesture = "None"
        self.current_raw_gesture = "None"
        self.current_confidence = 0.0
        self.stable_gesture = "None"
        self.gesture_streak = 0
        self.none_streak = 0
        self.last_published_gesture = "None"
        self.last_publish_time = time.time()
        self.auto_bright_active = False

        # Camera setup
        self.cap = None
        self.camera_connected = False
        self.last_reconnect_attempt = 0.0
        self._find_and_open_camera()

        # Timer for capture and processing at 30 FPS
        self.timer = self.create_timer(1.0 / 30.0, self.timer_callback)
        self.get_logger().info('-> gesture_recognition_node s?n s?ng! H? th?ng CH? nh?n l?nh t? c? ch? tay.')

    def _get_wsl_host_ip(self):
        try:
            output = subprocess.check_output("ip route show default | awk '{print $3}'", shell=True)
            ip = output.decode('utf-8').strip()
            if ip:
                return ip
        except Exception:
            pass
        return "172.26.96.1"

    def _find_and_open_camera(self):
        # 1. Specified video_source
        if self.video_source:
            self.cap = cv2.VideoCapture(self.video_source)
            if self.cap.isOpened():
                self.camera_connected = True
                self.get_logger().info(f'K?t n?i video_source th?nh c?ng: {self.video_source}')
                return

        # 2. Local Linux webcam (/dev/video*)
        if os.path.exists(f'/dev/video{self.camera_id}'):
            self.cap = cv2.VideoCapture(self.camera_id)
            if self.cap.isOpened():
                self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
                self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
                self.camera_connected = True
                self.get_logger().info(f'M? webcam /dev/video{self.camera_id} th?nh c?ng!')
                return

        # 3. Windows Camera Streamer Bridge (WSL2 to Windows host)
        host_ip = self._get_wsl_host_ip()
        stream_urls = [
            f"http://{host_ip}:8080/video",
            "http://localhost:8080/video",
            "http://127.0.0.1:8080/video"
        ]

        for url in stream_urls:
            try:
                test_cap = cv2.VideoCapture(url)
                if test_cap.isOpened():
                    ret, test_frame = test_cap.read()
                    if ret and test_frame is not None:
                        self.cap = test_cap
                        self.camera_connected = True
                        self.get_logger().info(f'K?t n?i th?nh c?ng t?i Windows Laptop Camera Streamer ({url})!')
                        return
                test_cap.release()
            except Exception:
                pass

        self.camera_connected = False

    def robot_status_callback(self, msg):
        self.robot_status = msg.data

    def timer_callback(self):
        now = time.time()
        frame = None

        if self.camera_connected and self.cap is not None:
            ret, frame = self.cap.read()
            if not ret or frame is None:
                self.camera_connected = False
                if self.cap:
                    self.cap.release()
                self.cap = None

        if not self.camera_connected:
            if now - self.last_reconnect_attempt > 2.0:
                self.last_reconnect_attempt = now
                self._find_and_open_camera()

            frame = self._render_no_camera_screen()
            landmarks = None
            bbox = None
            gesture = "None"
            conf = 0.0
        else:
            frame = cv2.flip(frame, 1)

            # Auto-Brightness / Auto-Contrast enhancement for dark lighting
            try:
                gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
                avg_lum = float(np.mean(gray))
                if avg_lum < 110.0:
                    alpha = min(2.0, max(1.15, 125.0 / max(avg_lum, 25.0)))
                    beta = int((110.0 - avg_lum) * 0.45)
                    frame = cv2.convertScaleAbs(frame, alpha=alpha, beta=beta)
                    self.auto_bright_active = True
                else:
                    self.auto_bright_active = False
            except Exception:
                self.auto_bright_active = False

            gesture, conf, landmarks, bbox = self.classifier.classify(frame)

        self.current_raw_gesture = gesture
        self.current_confidence = conf

        # Debounce filter logic
        if gesture == self.last_raw_gesture and conf >= self.confidence_threshold:
            self.gesture_streak += 1
        else:
            self.gesture_streak = 1
            self.last_raw_gesture = gesture

        # Latch stable gesture
        if self.gesture_streak >= self.debounce_frames and gesture != "None":
            self.stable_gesture = gesture
            self.none_streak = 0
        elif gesture == "None":
            self.none_streak += 1
            # Revert to "None" only after 8 consecutive frames of absence to prevent flickering
            if self.none_streak >= 8:
                self.stable_gesture = "None"

        # Continuous heartbeat publishing every 0.20s or immediately on state change
        if self.stable_gesture != self.last_published_gesture or (now - self.last_publish_time) > 0.20:
            msg = String()
            msg.data = self.stable_gesture
            self.gesture_pub.publish(msg)
            self.last_published_gesture = self.stable_gesture
            self.last_publish_time = now

        # Render HUD
        display_frame = self._draw_hud(frame, landmarks, bbox)

        # Publish ROS Image
        try:
            img_msg = self.bridge.cv2_to_imgmsg(display_frame, encoding="bgr8")
            self.image_pub.publish(img_msg)
        except Exception:
            pass

        # OpenCV GUI Window
        if self.show_gui:
            try:
                cv2.imshow("TurtleBot 4 Hand Gesture Recognition - AI Vision", display_frame)
                key = cv2.waitKey(1) & 0xFF
                if key == ord('q'):
                    rclpy.shutdown()
            except Exception as e:
                self.get_logger().warn(f"Kh?ng th? hi?n th? GUI cv2 ({e}). Ti?p t?c ? ch? ?? headless.", once=True)
                self.show_gui = False

    def _draw_hud(self, frame, landmarks, bbox):
        vis = frame.copy()
        h, w, _ = vis.shape

        # 1. Draw hand landmarks
        if landmarks is not None:
            for lm in landmarks:
                cx, cy = int(lm.x * w), int(lm.y * h)
                cv2.circle(vis, (cx, cy), 4, (0, 255, 255), -1)
                cv2.circle(vis, (cx, cy), 6, (0, 165, 255), 1)

            if bbox:
                x1, y1, x2, y2 = bbox
                cv2.rectangle(vis, (x1, y1), (x2, y2), (0, 255, 100), 2)

        # 2. Top Header Bar
        cv2.rectangle(vis, (0, 0), (w, 85), (20, 20, 20), -1)
        cv2.line(vis, (0, 85), (w, 85), (60, 60, 60), 1)

        cv2.putText(vis, "TURTLEBOT 4 - DIEU KHIEN BANG CU CHI TAY (AI VISION)",
                    (15, 26), cv2.FONT_HERSHEY_DUPLEX, 0.60, (0, 255, 200), 1, cv2.LINE_AA)

        cam_status_str = "CAMERA LIVE (30 FPS)" if self.camera_connected else "DANG DOI WEBCAM..."
        cam_status_color = (0, 255, 100) if self.camera_connected else (0, 165, 255)
        cv2.putText(vis, cam_status_str, (w - 240, 26),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.48, cam_status_color, 1, cv2.LINE_AA)

        if self.auto_bright_active:
            cv2.putText(vis, "[AUTO-BRIGHTNESS ON]", (w - 240, 48),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.38, (0, 220, 255), 1, cv2.LINE_AA)

        # Robot Behavior Status line with distinct color
        status_color = (0, 255, 255) if "PENDING" in self.robot_status else ((0, 255, 0) if "DANG CHAY" in self.robot_status else (255, 255, 255))
        cv2.putText(vis, f"Trang thai Robot: {self.robot_status}",
                    (15, 56), cv2.FONT_HERSHEY_DUPLEX, 0.52, status_color, 1, cv2.LINE_AA)

        # Prominent Prompt Banner when in PENDING confirmation mode
        if "PENDING" in self.robot_status:
            cv2.rectangle(vis, (0, 85), (w, 115), (0, 180, 220), -1)
            cv2.putText(vis, ">>> HAY GIO NGON CAI (THUMBS UP) DE ROBOT BAT DAU CHAY! <<<",
                        (20, 106), cv2.FONT_HERSHEY_DUPLEX, 0.55, (0, 0, 0), 2, cv2.LINE_AA)

        # 3. Active Gesture Info Panel (Bottom Left)
        box_y1 = h - 135
        cv2.rectangle(vis, (15, box_y1), (390, h - 15), (15, 15, 15), -1)
        cv2.rectangle(vis, (15, box_y1), (390, h - 15), (70, 70, 70), 1)

        vn_gesture_meanings = {
            "None": "Khong phat hien cu chi tay",
            "Wave": "Vay tay -> Chao lai (Xoay trai-phai)",
            "Pointing_Up": "Gio 1 ngon -> Chon Tien-Lui 4 lan",
            "Victory": "Gio 2 ngon -> Chon Quy dao tron 360",
            "Thumbs_Up": "Gio ngon cai -> XAC NHAN VA CHAY",
            "Open_Palm": "Xoe ban tay -> DUNG KHAN CAP",
            "Closed_Fist": "Nam tay -> HUY & TRO VE CHO"
        }

        badge_colors = {
            "None": (130, 130, 130),
            "Wave": (255, 150, 0),
            "Pointing_Up": (0, 200, 255),
            "Victory": (255, 0, 200),
            "Thumbs_Up": (0, 255, 0),
            "Open_Palm": (0, 0, 255),
            "Closed_Fist": (0, 165, 255)
        }
        color = badge_colors.get(self.stable_gesture, (200, 200, 200))

        cv2.putText(vis, f"CU CHI: {self.stable_gesture.upper()}",
                    (25, box_y1 + 30), cv2.FONT_HERSHEY_DUPLEX, 0.70, color, 2, cv2.LINE_AA)

        meaning = vn_gesture_meanings.get(self.stable_gesture, "")
        cv2.putText(vis, meaning, (25, box_y1 + 60),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.48, (220, 220, 220), 1, cv2.LINE_AA)

        conf_pct = int(self.current_confidence * 100)
        cv2.putText(vis, f"Do tin cay: {conf_pct}%", (25, box_y1 + 95),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.48, (200, 200, 200), 1, cv2.LINE_AA)

        cv2.rectangle(vis, (155, box_y1 + 83), (370, box_y1 + 97), (40, 40, 40), -1)
        bar_len = int(215 * min(1.0, self.current_confidence))
        cv2.rectangle(vis, (155, box_y1 + 83), (155 + bar_len, box_y1 + 97), color, -1)

        # 4. Gesture Reference Guide Panel (Top Right)
        ref_x = w - 300
        ref_y = 125
        cv2.rectangle(vis, (ref_x, ref_y), (w - 15, ref_y + 130), (15, 15, 15), -1)
        cv2.rectangle(vis, (ref_x, ref_y), (w - 15, ref_y + 130), (60, 60, 60), 1)

        cv2.putText(vis, "DANH MUC CU CHI DIEU KHIEN:", (ref_x + 10, ref_y + 20),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.40, (0, 255, 200), 1, cv2.LINE_AA)
        ref_items = [
            "- Vay tay (Wave): Chao lai",
            "- Gio 1 ngon (Pointing Up): Tien-lui 4x",
            "- Gio 2 ngon (Victory): Di vong tron 360",
            "- Gio ngon cai (Thumbs Up): Xac nhan",
            "- Xoe ban tay (Open Palm): Dung khan",
            "- Nam tay (Closed Fist): Huy lenh"
        ]
        for idx, text in enumerate(ref_items):
            cv2.putText(vis, text, (ref_x + 10, ref_y + 38 + idx * 15),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.35, (200, 200, 200), 1, cv2.LINE_AA)

        return vis

    def _render_no_camera_screen(self):
        canvas = np.zeros((480, 640, 3), dtype=np.uint8)
        canvas[:] = (30, 30, 35)

        cx, cy = 320, 240
        cv2.circle(canvas, (cx, cy - 30), 50, (60, 60, 70), -1)
        cv2.circle(canvas, (cx, cy - 30), 50, (0, 165, 255), 2)
        cv2.putText(canvas, "CAM", (cx - 28, cy - 22), cv2.FONT_HERSHEY_DUPLEX, 0.8, (0, 165, 255), 2)

        cv2.putText(canvas, "DANG CHO KET NOI WEBCAM...",
                    (cx - 180, cy + 50), cv2.FONT_HERSHEY_DUPLEX, 0.70, (255, 255, 255), 1)

        cv2.putText(canvas, "Neu chay trong WSL2, hay mo terminal Windows chay: run_camera_windows.bat",
                    (cx - 250, cy + 85), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 220, 255), 1)

        cv2.putText(canvas, "He thong se tu dong nhan webcam ngay khi duoc mo!",
                    (cx - 200, cy + 115), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (180, 180, 180), 1)

        return canvas

    def destroy_node(self):
        if self.cap is not None and self.cap.isOpened():
            self.cap.release()
        cv2.destroyAllWindows()
        if self.classifier:
            self.classifier.close()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = GestureRecognitionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

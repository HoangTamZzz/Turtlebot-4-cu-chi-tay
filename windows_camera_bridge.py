#!/usr/bin/env python3
"""
Windows Camera Bridge for WSL2 ROS 2 Gesture Control
Streams laptop webcam 0 over HTTP MJPEG on port 8080 so WSL2 OpenCV can read it with 0 latency.
Includes Automatic Brightness and Contrast Optimization for dark webcams.
"""
import sys
import time

# Ensure UTF-8 output on Windows console
if sys.platform == 'win32':
    try:
        sys.stdout.reconfigure(encoding='utf-8')
    except Exception:
        pass

import cv2
import numpy as np
from http.server import BaseHTTPRequestHandler, HTTPServer
from socketserver import ThreadingMixIn

cap = None

class ThreadedHTTPServer(ThreadingMixIn, HTTPServer):
    daemon_threads = True

class CamHandler(BaseHTTPRequestHandler):
    def do_GET(self):
        global cap
        if self.path.startswith('/video'):
            self.send_response(200)
            self.send_header('Content-type', 'multipart/x-mixed-replace; boundary=--frame')
            self.send_header('Access-Control-Allow-Origin', '*')
            self.end_headers()

            encode_param = [int(cv2.IMWRITE_JPEG_QUALITY), 80]
            while True:
                if cap is None or not cap.isOpened():
                    break
                ret, frame = cap.read()
                if not ret or frame is None:
                    time.sleep(0.01)
                    continue

                # Auto Brightness and Contrast Optimization for dark webcams
                try:
                    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
                    avg_lum = float(np.mean(gray))
                    if avg_lum < 110.0:
                        alpha = min(2.0, max(1.15, 125.0 / max(avg_lum, 25.0)))
                        beta = int((110.0 - avg_lum) * 0.45)
                        frame = cv2.convertScaleAbs(frame, alpha=alpha, beta=beta)
                except Exception:
                    pass

                ret, jpeg = cv2.imencode('.jpg', frame, encode_param)
                if not ret:
                    continue
                try:
                    self.wfile.write(b'--frame\r\n')
                    self.wfile.write(b'Content-Type: image/jpeg\r\n')
                    self.wfile.write(f'Content-Length: {len(jpeg)}\r\n\r\n'.encode('utf-8'))
                    self.wfile.write(jpeg.tobytes())
                    self.wfile.write(b'\r\n')
                except (BrokenPipeError, ConnectionResetError):
                    break
        elif self.path == '/status':
            self.send_response(200)
            self.send_header('Content-type', 'text/plain')
            self.end_headers()
            self.wfile.write(b'CAMERA_STREAMER_OK')
        else:
            self.send_response(404)
            self.end_headers()

    def log_message(self, format, *args):
        # Silence console log spam
        pass

def main():
    global cap
    print("=" * 60)
    print("   WINDOWS CAMERA STREAMER -> WSL2 TURTLEBOT 4 GESTURE")
    print("   (Auto-Brightness: Tu dong nang do sang khi camera toi)")
    print("=" * 60)
    print("-> Dang mo Webcam laptop (Camera 0)...")
    cap = cv2.VideoCapture(0, cv2.CAP_DSHOW)
    if not cap.isOpened():
        cap = cv2.VideoCapture(0)

    if not cap.isOpened():
        print("[LOI] Khong the mo webcam laptop! Vui long kiem tra quyen camera.")
        sys.exit(1)

    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
    cap.set(cv2.CAP_PROP_FPS, 30)

    try:
        cap.set(cv2.CAP_PROP_AUTO_EXPOSURE, 0.75)
    except Exception:
        pass

    print("-> Webcam mo thanh cong (640x480 @ 30fps) - Auto Brightness ON!")
    port = 8080
    server = ThreadedHTTPServer(('0.0.0.0', port), CamHandler)
    print(f"-> Dang phat video stream tai: http://localhost:{port}/video")
    print("-> WSL2 ROS 2 se tu dong ket noi ngay khi duoc mo!")
    print("-> Giu cua so nay mo trong suot qua trinh thuc hanh.")
    print("=" * 60)
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        print("\nDang dung camera streamer...")
    finally:
        if cap:
            cap.release()
        server.server_close()

if __name__ == '__main__':
    main()
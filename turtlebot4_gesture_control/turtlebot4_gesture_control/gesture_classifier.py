#!/usr/bin/env python3
import time
import os
import math
from collections import deque
import numpy as np
import mediapipe as mp
from mediapipe.tasks import python
from mediapipe.tasks.python import vision

class GestureClassifier:
    """
    Robust Multi-Modal Hand Gesture Classifier:
    1. MediaPipe Tasks GestureRecognizer (Official Google Model)
    2. Dynamic Spatio-Temporal Wave Motion Detector (Wrist/Palm oscillation)
    3. Geometric Landmark Fallback Classifier (Finger extension heuristics)
    """

    GESTURE_NONE = "None"
    GESTURE_WAVE = "Wave"
    GESTURE_POINTING_UP = "Pointing_Up"
    GESTURE_VICTORY = "Victory"
    GESTURE_THUMBS_UP = "Thumbs_Up"
    GESTURE_OPEN_PALM = "Open_Palm"
    GESTURE_CLOSED_FIST = "Closed_Fist"

    def __init__(self, model_path=None):
        if model_path is None:
            model_path = "/home/hoangtam/ros2_ws/src/Turtlebot_4_cuchitay/turtlebot4_gesture_control/models/gesture_recognizer.task"
        
        self.model_path = model_path
        self.recognizer = None
        self._init_recognizer()

        # Motion tracking for Dynamic Wave Gesture
        self.wrist_history = deque(maxlen=30)
        self.last_wave_time = 0.0

    def _init_recognizer(self):
        if os.path.exists(self.model_path):
            try:
                base_options = python.BaseOptions(model_asset_path=self.model_path)
                options = vision.GestureRecognizerOptions(
                    base_options=base_options,
                    running_mode=vision.RunningMode.IMAGE,
                    num_hands=1,
                    min_hand_detection_confidence=0.45,
                    min_hand_presence_confidence=0.45,
                    min_tracking_confidence=0.45
                )
                self.recognizer = vision.GestureRecognizer.create_from_options(options)
                print(f"[GestureClassifier] Successfully loaded model from {self.model_path}")
            except Exception as e:
                print(f"[GestureClassifier] Warning: Could not init GestureRecognizer: {e}")
                self.recognizer = None
        else:
            print(f"[GestureClassifier] Model file not found at {self.model_path}, using geometric fallback.")

    def close(self):
        if self.recognizer:
            try:
                self.recognizer.close()
            except Exception:
                pass
            self.recognizer = None

    def classify(self, bgr_image):
        """
        Processes a BGR OpenCV image and returns:
        (gesture_name, confidence, landmarks, bounding_box)
        """
        now = time.time()
        rgb_image = bgr_image[:, :, ::-1]
        h, w, _ = bgr_image.shape

        mp_image = mp.Image(image_format=mp.ImageFormat.SRGB, data=rgb_image)

        recognized_gesture = self.GESTURE_NONE
        confidence = 0.0
        landmarks = None
        bbox = None

        if self.recognizer is not None:
            try:
                result = self.recognizer.recognize(mp_image)
                if result and result.gestures and len(result.gestures) > 0:
                    top_gesture = result.gestures[0][0]
                    category_name = top_gesture.category_name
                    raw_conf = float(top_gesture.score)

                    # Map MediaPipe standard category names
                    if category_name == "Pointing_Up":
                        recognized_gesture = self.GESTURE_POINTING_UP
                        confidence = raw_conf
                    elif category_name == "Victory":
                        recognized_gesture = self.GESTURE_VICTORY
                        confidence = raw_conf
                    elif category_name in ["Thumb_Up", "Thumbs_Up"]:
                        recognized_gesture = self.GESTURE_THUMBS_UP
                        confidence = raw_conf
                    elif category_name == "Open_Palm":
                        recognized_gesture = self.GESTURE_OPEN_PALM
                        confidence = raw_conf
                    elif category_name == "Closed_Fist":
                        recognized_gesture = self.GESTURE_CLOSED_FIST
                        confidence = raw_conf

                if result and result.hand_landmarks and len(result.hand_landmarks) > 0:
                    landmarks = result.hand_landmarks[0]
            except Exception:
                pass

        # If landmarks were detected, check Wave motion and geometric verification
        if landmarks is not None and len(landmarks) >= 21:
            wrist = landmarks[0]
            wrist_x = wrist.x
            wrist_y = wrist.y

            # Compute bounding box
            xs = [lm.x * w for lm in landmarks]
            ys = [lm.y * h for lm in landmarks]
            bbox = (int(min(xs) - 15), int(min(ys) - 15), int(max(xs) + 15), int(max(ys) + 15))

            # Update wrist history for Wave detection
            self.wrist_history.append((now, wrist_x, wrist_y))

            # 1. Dynamic Wave Gesture check
            is_wave, wave_conf = self._detect_wave(landmarks)
            if is_wave:
                recognized_gesture = self.GESTURE_WAVE
                confidence = max(confidence, wave_conf)
            elif recognized_gesture == self.GESTURE_NONE or confidence < 0.60:
                # 2. Geometric Landmark Heuristic check
                geo_gesture, geo_conf = self._geometric_classification(landmarks)
                if geo_gesture != self.GESTURE_NONE and geo_conf > confidence:
                    recognized_gesture = geo_gesture
                    confidence = geo_conf
        else:
            self.wrist_history.clear()

        return recognized_gesture, confidence, landmarks, bbox

    def _detect_wave(self, landmarks):
        """
        Detects waving motion:
        Hand is mostly open and wrist/palm oscillates horizontally.
        """
        now = time.time()
        valid_history = [p for p in self.wrist_history if (now - p[0]) <= 1.2]
        if len(valid_history) < 8:
            return False, 0.0

        xs = [p[1] for p in valid_history]
        dx_total = max(xs) - min(xs)

        reversals = 0
        diffs = [xs[i+1] - xs[i] for i in range(len(xs)-1)]
        signs = [1 if d > 0.003 else (-1 if d < -0.003 else 0) for d in diffs]
        signs = [s for s in signs if s != 0]

        for i in range(len(signs)-1):
            if signs[i] * signs[i+1] < 0:
                reversals += 1

        wrist = landmarks[0]
        middle_tip = landmarks[12]
        hand_is_upright = middle_tip.y < wrist.y

        if reversals >= 2 and dx_total > 0.06 and hand_is_upright:
            conf = min(0.95, 0.5 + 0.1 * reversals + dx_total * 2.0)
            return True, conf

        return False, 0.0

    def _geometric_classification(self, landmarks):
        """
        Robust geometric classifier based on 21 hand landmarks:
        - Pointing_Up: Index finger extended, others curled
        - Victory: Index & Middle extended, others curled
        - Thumbs_Up: Thumb pointing up, other 4 fingers curled
        - Open_Palm: All 5 fingers extended outward
        - Closed_Fist: All 4 fingers curled tightly into palm
        """
        wrist = landmarks[0]

        def dist_sq(lm1, lm2):
            return (lm1.x - lm2.x)**2 + (lm1.y - lm2.y)**2

        def is_finger_extended(tip_idx, pip_idx, mcp_idx):
            tip = landmarks[tip_idx]
            pip = landmarks[pip_idx]
            mcp = landmarks[mcp_idx]
            d_tip_wrist = dist_sq(tip, wrist)
            d_pip_wrist = dist_sq(pip, wrist)
            d_tip_mcp = dist_sq(tip, mcp)
            d_pip_mcp = dist_sq(pip, mcp)
            return (d_tip_wrist > d_pip_wrist * 1.15) and (d_tip_mcp > d_pip_mcp)

        def is_finger_curled(tip_idx, pip_idx, mcp_idx):
            tip = landmarks[tip_idx]
            pip = landmarks[pip_idx]
            mcp = landmarks[mcp_idx]
            d_tip_wrist = dist_sq(tip, wrist)
            d_pip_wrist = dist_sq(pip, wrist)
            return (d_tip_wrist < d_pip_wrist) or (dist_sq(tip, mcp) < dist_sq(pip, mcp) * 0.7)

        index_ext = is_finger_extended(8, 6, 5)
        middle_ext = is_finger_extended(12, 10, 9)
        ring_ext = is_finger_extended(16, 14, 13)
        pinky_ext = is_finger_extended(20, 18, 17)

        index_curled = is_finger_curled(8, 6, 5)
        middle_curled = is_finger_curled(12, 10, 9)
        ring_curled = is_finger_curled(16, 14, 13)
        pinky_curled = is_finger_curled(20, 18, 17)

        # Thumb up: tip is higher than IP and MCP, pointing upward
        thumb_tip = landmarks[4]
        thumb_ip = landmarks[3]
        thumb_mcp = landmarks[2]
        thumb_is_up = (thumb_tip.y < thumb_ip.y) and (thumb_tip.y < thumb_mcp.y) and (thumb_tip.y < wrist.y)

        # 1. Pointing Up: Index extended, Middle/Ring/Pinky curled
        if index_ext and middle_curled and ring_curled and pinky_curled:
            return self.GESTURE_POINTING_UP, 0.90

        # 2. Victory: Index & Middle extended, Ring & Pinky curled
        if index_ext and middle_ext and ring_curled and pinky_curled:
            # Bonus verification: distance between tips > 0.03
            d_tips = dist_sq(landmarks[8], landmarks[12])
            if d_tips > 0.0009: # separated index and middle
                return self.GESTURE_VICTORY, 0.92

        # 3. Thumbs Up: Thumb pointing up, all other 4 fingers curled
        if thumb_is_up and index_curled and middle_curled and ring_curled and pinky_curled:
            return self.GESTURE_THUMBS_UP, 0.90

        # 4. Open Palm: All 4 main fingers extended and thumb extended outward
        ext_count = sum([index_ext, middle_ext, ring_ext, pinky_ext])
        if ext_count >= 4 and thumb_tip.y < wrist.y:
            return self.GESTURE_OPEN_PALM, 0.88

        # 5. Closed Fist: All 4 fingers curled tightly into palm
        if index_curled and middle_curled and ring_curled and pinky_curled and not thumb_is_up:
            # Tip of thumb must be close to index/middle finger
            d_thumb_index = dist_sq(thumb_tip, landmarks[6])
            if d_thumb_index < 0.05:
                return self.GESTURE_CLOSED_FIST, 0.88

        return self.GESTURE_NONE, 0.0

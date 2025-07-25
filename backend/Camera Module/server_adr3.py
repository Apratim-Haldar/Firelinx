# server.py

import cv2
import numpy as np
from flask import Flask, Response, render_template, jsonify
import threading
import time
import sys
import queue
from ultralytics import YOLO
import pygame
import torch
import math
from datetime import datetime
import pprint
import json  # <-- NEW: For converting dict to JSON string
import paho.mqtt.client as mqtt  # <-- NEW: The MQTT client library

# --- NEW: HIVEMQTT Configuration ---
MQTT_CONFIG = {
    'broker_hostname': '259353f6c5704a35aeb3dff107a0ab04.s1.eu.hivemq.cloud',
    'broker_port': 8884,
    'username': 'Staferb',
    'password': 'EspWebDash@32',
    'topic': 'staferb/web_alerts'
}

# --- Flask App Initialization ---
app = Flask(__name__)

# --- Main Fire Detection Class ---
class FireDetectorServer:
    def __init__(self, esp32_ip):
        # Stream and Connection Configuration
        self.stream_url = f"http://{esp32_ip}/stream"
        self.capture = None
        self.is_connected = False
        
        # Frame Processing Management
        self.frame_queue = queue.Queue(maxsize=2)
        self.display_frame = None
        self.display_lock = threading.Lock()
        self.processing_thread = None
        self.last_frame_time = time.time()

        # --- NEW: MQTT Client Setup ---
        self.mqtt_client = None
        self._setup_mqtt()
        # ---

        # Model Initialization
        print("[+] Loading dual detection models...")
        self.device = 'cuda' if torch.cuda.is_available() else 'cpu'
        
        try:
            self.fire_model = YOLO("best.pt").to(self.device)
            self.screen_model = YOLO("yolov8n.pt").to(self.device)
            self.screen_classes = [62, 63, 67]
            print(f"[+] Models loaded successfully on {self.device.upper()}")
        except Exception as e:
            print(f"[!] CRITICAL ERROR: Model loading failed: {e}")
            sys.exit(1)

        # Sound Alert Initialization
        print("[+] Initializing Pygame Mixer for sound alerts...")
        pygame.mixer.init()
        self.alert_sound_path = "siren-alert-96052.mp3"
        self.is_sound_loaded = False
        try:
            pygame.mixer.music.load(self.alert_sound_path)
            self.is_sound_loaded = True
            print(f"[+] Sound file '{self.alert_sound_path}' loaded successfully.")
        except pygame.error as e:
            print(f"[!] ERROR: Could not load sound file: {e}")
            print("[!] Alerts will be silent.")

        # State Tracking
        self.alert_flag = False
        self.fire_confirmed_globally = False
        self.fire_tracking = {
            "start_time": None, "position": None, "confirmed": False,
            "last_seen_time": 0, "confidence": 0.0
        }
        self.STABILITY_SECONDS = 3
        self.TRACKING_TIMEOUT = 2
        self.fire_intensity = 0.0
        self.intensity_calculated = False

        # Detection Parameters
        self.screen_boxes = []
        self.SCREEN_CONF_THRESH = 0.45
        self.IOU_THRESHOLD = 0.25
        self.enable_advanced_screen_detection = True

        # FPS and Status Tracking
        self.processing_fps = 0.0
        self.tracking_status = "Initializing..."
        self.frame_counter = 0
        self.last_processed_time = time.time()
        print(f"[+] Initialized FireDetectorServer for ESP32 at: {self.stream_url}")

    # --- NEW: MQTT Setup and Callback Functions ---
    def _on_mqtt_connect(self, client, userdata, flags, rc):
        if rc == 0:
            print("[+] Successfully connected to HiveMQ Cloud!")
        else:
            print(f"[!] Failed to connect to HiveMQ, return code {rc}\n")

    def _setup_mqtt(self):
        print("[*] Setting up MQTT client...")
        try:
            # Using transport='websockets' is crucial for wss:// URLs
            self.mqtt_client = mqtt.Client(transport='websockets')
            self.mqtt_client.on_connect = self._on_mqtt_connect
            
            # Set username, password, and enable TLS for secure connection
            self.mqtt_client.username_pw_set(MQTT_CONFIG['username'], MQTT_CONFIG['password'])
            self.mqtt_client.tls_set(tls_version=mqtt.ssl.PROTOCOL_TLS)

            # Connect to the broker
            self.mqtt_client.connect(
                MQTT_CONFIG['broker_hostname'],
                MQTT_CONFIG['broker_port'],
                60
            )
            # Start the network loop in a background thread
            self.mqtt_client.loop_start()
        except Exception as e:
            print(f"[!] CRITICAL ERROR: MQTT setup failed: {e}")
            self.mqtt_client = None

    def start(self):
        threading.Thread(target=self._stream_loop, daemon=True).start()
        self.processing_thread = threading.Thread(target=self._processing_loop, daemon=True)
        self.processing_thread.start()

    def calculate_fire_intensity(self, fire_roi, frame_area):
        if fire_roi.size == 0: return 0.0
        fire_area = fire_roi.shape[0] * fire_roi.shape[1]
        area_factor = min(1.0, (fire_area / frame_area) / 0.25)
        hsv_roi = cv2.cvtColor(fire_roi, cv2.COLOR_BGR2HSV)
        mask_red = cv2.inRange(hsv_roi, np.array([0, 150, 150]), np.array([15, 255, 255]))
        mask_yellow = cv2.inRange(hsv_roi, np.array([20, 100, 200]), np.array([40, 255, 255]))
        red_pixels = cv2.countNonZero(mask_red)
        yellow_pixels = cv2.countNonZero(mask_yellow)
        total_pixels = fire_area or 1
        color_factor = min(1.5, ((red_pixels * 1.0) + (yellow_pixels * 1.5)) / total_pixels)
        gray_roi = cv2.cvtColor(fire_roi, cv2.COLOR_BGR2GRAY)
        brightness_factor = np.mean(gray_roi) / 255.0
        intensity_score = ((area_factor * 0.4) + (color_factor * 0.4) + (brightness_factor * 0.2)) * 100
        return min(100.0, intensity_score)

    def _create_structured_fire_alert(self, intensity_float):
        if intensity_float < 25.0: level = 1
        elif intensity_float < 50.0: level = 2
        elif intensity_float < 75.0: level = 3
        else: level = 4
        intensity_char = str(level)
        now = datetime.now()
        alert_data = {
            'fireType': 'A', 'fireIntensity': intensity_char, 'verified': True,
            'user': "Auto Mode", 'userID': "AUTO", 'stnID': "A/1",
            'latitude': "N/A", 'longitude': "N/A",
            'date': now.strftime("%Y-%m-%d"), 'time': now.strftime("%H:%M:%S")
        }
        return alert_data

    def _stream_loop(self):
        print("[*] Starting video stream capture thread...")
        while True:
            try:
                self.capture = cv2.VideoCapture(self.stream_url)
                if not self.capture.isOpened(): raise ConnectionError("Stream URL not accessible.")
                self.capture.set(cv2.CAP_PROP_BUFFERSIZE, 1); self.capture.set(cv2.CAP_PROP_FPS, 15)
                self.is_connected = True; print("[+] Stream connected successfully.")
                while self.is_connected:
                    ret, frame = self.capture.read()
                    if not ret: self.is_connected = False; break
                    if self.frame_queue.full(): self.frame_queue.get_nowait()
                    self.frame_queue.put(frame)
            except Exception as e:
                print(f"[!] Stream Error: {e}. Retrying in 5 seconds...")
                self.is_connected = False
                if self.capture: self.capture.release()
                time.sleep(5)

    def _processing_loop(self):
        fps_start_time = time.time(); fps_counter = 0
        while True:
            try: frame = self.frame_queue.get(timeout=1)
            except queue.Empty:
                with self.display_lock: self.display_frame = self._create_placeholder_frame("Waiting for stream...")
                self.tracking_status = "No Stream"; continue
            current_time = time.time()
            if current_time - self.last_processed_time < 0.05: time.sleep(0.01); continue
            self.last_processed_time = current_time
            processed_frame = self.process_frame(frame)
            with self.display_lock: self.display_frame = processed_frame
            fps_counter += 1
            elapsed = time.time() - fps_start_time
            if elapsed >= 1.0: self.processing_fps = fps_counter / elapsed; fps_counter = 0; fps_start_time = time.time()

    def process_frame(self, frame):
        orig_frame = frame.copy(); self.frame_counter += 1
        scale_factor = 0.6; small_frame = cv2.resize(frame, (0, 0), fx=scale_factor, fy=scale_factor)
        inv_scale = 1 / scale_factor
        run_advanced_detection = (self.frame_counter % 4 == 0) and self.enable_advanced_screen_detection
        frame_tensor = torch.from_numpy(cv2.cvtColor(small_frame, cv2.COLOR_BGR2RGB)).to(self.device).float() / 255.0
        frame_tensor = frame_tensor.permute(2, 0, 1).unsqueeze(0)
        with torch.no_grad():
            if self.frame_counter % 3 == 0: screen_results = self.screen_model(frame_tensor, classes=self.screen_classes, conf=self.SCREEN_CONF_THRESH, verbose=False, imgsz=256)[0]
            else: screen_results = None
            fire_results = self.fire_model.predict(source=frame_tensor, verbose=False, conf=0.65, iou=0.45, device=self.device, imgsz=256)[0]
        if screen_results is not None:
            self.screen_boxes = []
            for box in screen_results.boxes:
                if float(box.conf[0]) > self.SCREEN_CONF_THRESH:
                    x1, y1, x2, y2 = map(int, box.xyxy[0].tolist()); x1, y1, x2, y2 = int(x1 * inv_scale), int(y1 * inv_scale), int(x2 * inv_scale), int(y2 * inv_scale)
                    self.screen_boxes.append((x1, y1, x2, y2))
                    cv2.rectangle(frame, (x1, y1), (x2, y2), (255, 50, 255), 2); cv2.putText(frame, "Screen", (x1, y1 - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 50, 255), 2)
        if run_advanced_detection:
            edge_screens = self.detect_screens_by_edges(orig_frame)
            for screen_box in edge_screens:
                self.screen_boxes.append(screen_box); x1, y1, x2, y2 = screen_box
                cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 255), 2); cv2.putText(frame, "Screen (Edge)", (x1, y1 - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)
        potential_fire_found = False
        if fire_results.boxes is not None and len(fire_results.boxes) > 0:
            for box in fire_results.boxes:
                cls_id = int(box.cls[0]); conf = float(box.conf[0])
                if cls_id != 0 or conf < 0.6: continue
                x1, y1, x2, y2 = map(int, box.xyxy[0].tolist()); x1, y1, x2, y2 = int(x1 * inv_scale), int(y1 * inv_scale), int(x2 * inv_scale), int(y2 * inv_scale)
                fire_box = (x1, y1, x2, y2); roi = orig_frame[y1:y2, x1:x2]
                if roi.size == 0: continue
                if self.frame_counter % 4 == 0 or self.validate_fire_region(roi):
                    if self.check_screen_overlap(fire_box):
                        cv2.rectangle(frame, (x1, y1), (x2, y2), (100, 100, 255), 2); cv2.putText(frame, "SCREEN FIRE", (x1, y1 - 30), cv2.FONT_HERSHEY_SIMPLEX, 0.9, (100, 100, 255), 2)
                        self.tracking_status = "Fire on screen - Ignored"; continue
                    potential_fire_found = True; self.tracking_status = "Tracking Fire Candidate..."
                    current_time = time.time()
                    if self.fire_tracking["position"] and self.same_area(fire_box, self.fire_tracking["position"]):
                        if self.fire_tracking["start_time"] and (current_time - self.fire_tracking["start_time"]) > self.STABILITY_SECONDS: self.fire_tracking["confirmed"] = True
                    else: self.fire_tracking["start_time"] = current_time; self.fire_tracking["position"] = fire_box; self.fire_tracking["confirmed"] = False
                    self.fire_tracking["last_seen_time"] = current_time; self.fire_tracking["confidence"] = conf
                    cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 0, 255), 3); cv2.putText(frame, f"FIRE: {conf:.2f}", (x1, y1 - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.9, (0, 0, 255), 2)
        if not potential_fire_found:
            self.tracking_status = "All Clear"; current_time = time.time()
            if self.fire_tracking["start_time"] and (current_time - self.fire_tracking["last_seen_time"]) > self.TRACKING_TIMEOUT:
                self.tracking_status = "Tracker Reset"; self.fire_tracking = {"start_time": None, "position": None, "confirmed": False, "last_seen_time": 0, "confidence": 0.0}
                self.intensity_calculated = False; self.fire_intensity = 0.0
        self.fire_confirmed_globally = self.fire_tracking["confirmed"]
        if self.fire_confirmed_globally:
            self.tracking_status = "STABLE FIRE DETECTED!"
            if not self.alert_flag and self.is_sound_loaded:
                print("🔥 STABLE FIRE DETECTED! Triggering alert.")
                pygame.mixer.music.play(-1); self.alert_flag = True
                if not self.intensity_calculated:
                    x1, y1, x2, y2 = self.fire_tracking["position"]; fire_roi = orig_frame[y1:y2, x1:x2]
                    frame_area = orig_frame.shape[0] * orig_frame.shape[1]
                    self.fire_intensity = self.calculate_fire_intensity(fire_roi, frame_area); self.intensity_calculated = True
                    alert_payload = self._create_structured_fire_alert(self.fire_intensity)
                    print("\n--- STRUCTURED FIRE ALERT GENERATED ---")
                    pprint.pprint(alert_payload, sort_dicts=False)
                    print("---------------------------------------\n")
                    
                    # --- NEW: PUBLISH MESSAGE TO MQTT ---
                    if self.mqtt_client and self.mqtt_client.is_connected():
                        # Convert the dictionary to a JSON string for publishing
                        message_str = json.dumps(alert_payload)
                        result = self.mqtt_client.publish(MQTT_CONFIG['topic'], message_str)
                        if result.rc == mqtt.MQTT_ERR_SUCCESS:
                             print(f"✅ Successfully sent alert message to MQTT topic: {MQTT_CONFIG['topic']}")
                        else:
                             print(f"❌ Failed to send MQTT message, error code: {result.rc}")
                    else:
                        print("[!] MQTT client not connected. Cannot send alert.")
                    # --- END OF MQTT PUBLISH ---

        elif self.alert_flag:
            print("[+] Condition cleared. Stopping alert.")
            pygame.mixer.music.stop(); self.alert_flag = False
            self.intensity_calculated = False; self.fire_intensity = 0.0
        return self._overlay(frame)

    def _overlay(self, frame):
        h, w, _ = frame.shape
        if not hasattr(self, 'status_bar') or self.status_bar.shape[1] != w: self.status_bar = np.zeros((50, w, 3), dtype=np.uint8)
        status_bar = self.status_bar
        if self.fire_confirmed_globally: status_text = "FIRE DETECTED!"; status_color = (0, 0, 255); status_bar[:, :] = (0, 0, 100)
        else:
            status_text = self.tracking_status
            if "Tracking" in status_text: status_color = (0, 255, 255); status_bar[:, :] = (0, 100, 100)
            elif "screen" in status_text.lower(): status_color = (180, 105, 255); status_bar[:, :] = (100, 50, 100)
            else: status_color = (0, 255, 0); status_bar[:, :] = (0, 100, 0)
        frame[0:50, 0:w] = status_bar
        cv2.putText(frame, status_text, (10, 35), cv2.FONT_HERSHEY_SIMPLEX, 1.0, status_color, 2, cv2.LINE_AA)
        if self.fire_confirmed_globally and self.intensity_calculated:
            intensity_text = f"Intensity: {self.fire_intensity:.1f}"
            (text_width, _), _ = cv2.getTextSize(intensity_text, cv2.FONT_HERSHEY_SIMPLEX, 0.8, 2)
            cv2.putText(frame, intensity_text, (w - text_width - 170, 35), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 255, 255), 2)
        fps_text = f"FPS: {self.processing_fps:.1f}"; cv2.putText(frame, fps_text, (w - 150, 35), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 255, 255), 2)
        return frame
    def detect_screens_by_edges(self, frame):
        scale_factor = 0.5; small_frame = cv2.resize(frame, (0, 0), fx=scale_factor, fy=scale_factor)
        screen_boxes = []; gray = cv2.cvtColor(small_frame, cv2.COLOR_BGR2GRAY); blurred = cv2.GaussianBlur(gray, (3, 3), 0)
        thresh = cv2.adaptiveThreshold(blurred, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C, cv2.THRESH_BINARY_INV, 11, 2)
        contours, _ = cv2.findContours(thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        frame_area = small_frame.shape[0] * small_frame.shape[1]
        for contour in contours:
            area = cv2.contourArea(contour)
            if area < 0.01 * frame_area: continue
            rect = cv2.minAreaRect(contour); box = cv2.boxPoints(rect); box = np.intp(box)
            width, height = rect[1]; aspect_ratio = max(width, height) / min(width, height) if min(width, height) > 0 else 0
            if not (1.2 <= aspect_ratio <= 2.5): continue
            x, y, w, h = cv2.boundingRect(contour); x, y, w, h = int(x/scale_factor), int(y/scale_factor), int(w/scale_factor), int(h/scale_factor)
            if w < 20 or h < 20: continue
            if self.is_screen_region(frame[y:y+h, x:x+w]): screen_boxes.append((x, y, x+w, y+h))
        return screen_boxes
    def is_screen_region(self, region):
        if region.size == 0 or region.shape[0] < 10 or region.shape[1] < 10: return False
        if np.std(region) < 25: return False
        gray = cv2.cvtColor(region, cv2.COLOR_BGR2GRAY); edges = cv2.Canny(gray, 50, 150)
        if np.sum(edges > 0) / float(region.size) < 0.05: return False
        return True
    def validate_fire_region(self, crop):
        hsv = cv2.cvtColor(crop, cv2.COLOR_BGR2HSV)
        fire_mask = cv2.inRange(hsv, (5, 50, 50), (25, 255, 255)); fire_mask |= cv2.inRange(hsv, (0, 50, 50), (10, 255, 255))
        if np.sum(fire_mask > 0) < (crop.shape[0] * crop.shape[1] * 0.05): return False
        return True
    def check_screen_overlap(self, fire_box):
        fx1, fy1, fx2, fy2 = fire_box; fire_area = (fx2 - fx1) * (fy2 - fy1)
        if fire_area == 0: return False
        for (sx1, sy1, sx2, sy2) in self.screen_boxes:
            inter_x1 = max(fx1, sx1); inter_y1 = max(fy1, sy1); inter_x2 = min(fx2, sx2); inter_y2 = min(fy2, sy2)
            if inter_x2 <= inter_x1 or inter_y2 <= inter_y1: continue
            inter_area = (inter_x2 - inter_x1) * (inter_y2 - inter_y1)
            if inter_area / fire_area > self.IOU_THRESHOLD: return True
        return False
    def same_area(self, pos1, pos2, tolerance=50):
        x1, y1, _, _ = pos1; a1, b1, _, _ = pos2; return abs(x1 - a1) < tolerance and abs(y1 - b1) < tolerance
    def _create_placeholder_frame(self, text):
        if not hasattr(self, 'placeholder_frame'):
            self.placeholder_frame = np.zeros((480, 640, 3), dtype=np.uint8)
            cv2.putText(self.placeholder_frame, text, (50, 240), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2, cv2.LINE_AA)
        return self.placeholder_frame

detector = None
@app.route('/')
def index(): return render_template('index.html')
def generate_video_stream():
    last_frame = None
    while True:
        if detector and detector.display_frame is not None:
            with detector.display_lock: current_frame = detector.display_frame
            if current_frame is not last_frame:
                success, buffer = cv2.imencode('.jpg', current_frame, [int(cv2.IMWRITE_JPEG_QUALITY), 75])
                if success:
                    frame_bytes = buffer.tobytes(); last_frame = current_frame
                    yield (b'--frame\r\n' b'Content-Type: image/jpeg\r\n\r\n' + frame_bytes + b'\r\n')
            else: time.sleep(0.01)
        else: time.sleep(0.05)
@app.route('/video_feed')
def video_feed(): return Response(generate_video_stream(), mimetype='multipart/x-mixed-replace; boundary=frame')
@app.route('/status')
def status():
    if not detector: return jsonify({'is_connected': False, 'fire_detected': False, 'status': 'Detector not initialized'})
    return jsonify({
        'is_connected': bool(detector.is_connected), 'processing_fps': float(detector.processing_fps),
        'fire_detected': bool(detector.fire_confirmed_globally),
        'fire_confidence': float(detector.fire_tracking.get("confidence", 0.0)),
        'tracking_status': str(detector.tracking_status), 'alert_active': bool(detector.alert_flag),
        'screen_detected': len(detector.screen_boxes) > 0,
        'fire_intensity': float(detector.fire_intensity) if detector.intensity_calculated else 0.0
    })

if __name__ == '__main__':
    if len(sys.argv) < 2:
        print("Usage: python server.py <ESP32_IP_ADDRESS>"); sys.exit(1)
    esp32_ip = sys.argv[1]
    detector = FireDetectorServer(esp32_ip)
    detector.start()
    print("[+] Starting Flask web server at http://0.0.0.0:5000")
    app.run(host="0.0.0.0", port=5000)
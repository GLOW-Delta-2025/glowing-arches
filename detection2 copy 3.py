import cv2
from ultralytics import YOLO
import time
import serial
import math
 
# --- Configuration ---
MODEL_NAME = "yolov8n.pt"
CONFIDENCE_THRESHOLD = 0.5
CAMERA_FOCAL_LENGTH_PIXELS = 1000  # Calibrate!
KNOWN_OBJECT_HEIGHT = 1.75
MAX_DISTANCE = 5
MIN_DISTANCE = 0
SERIAL_PORT = 'COM3'
BAUD_RATE = 9600
 
SEND_INTERVAL = 100  # ms
millis_last_sent = time.time_ns()
 
# Motion detection parameter for push (shove) gesture
SHOVE_THRESHOLD = 10  # pixels difference in bounding box height
 
# Gains for incremental control (you may need to tweak these)
TRACKING_GAIN = 10      # Gain for normal tracking adjustment
PUSH_GAIN = 20          # Additional gain when push motion is detected
 
# Full DMX range constants
DMX_MIN = 1
DMX_MAX = 255
 
def estimate_distance(bbox_height_pixels):
    """Estimate distance based on bounding box height."""
    if bbox_height_pixels <= 0:
        return -1
    return (KNOWN_OBJECT_HEIGHT * CAMERA_FOCAL_LENGTH_PIXELS) / bbox_height_pixels
 
def send_dmx(ser, pan, tilt, color):
    """Send DMX pan, tilt, and color values via serial."""
    global millis_last_sent
    current_time = time.time_ns()
 
    # Ensure a minimum interval between DMX commands
    if (current_time - millis_last_sent) / 1_000_000 < SEND_INTERVAL:
        return
 
    millis_last_sent = current_time
 
    try:
        pan = max(DMX_MIN, min(DMX_MAX, int(pan)))
        tilt = max(DMX_MIN, min(DMX_MAX, int(tilt)))
        red, green, blue = color
 
        # Build DMX command
        command = f"0:{pan},{tilt},{red},{green},{blue},0,0,20;\n"
        ser.write(command.encode())
        print(f"Sent DMX: Pan={pan}, Tilt={tilt}, Color={color}")
 
        response = read_serial_response(ser)
        if response:
            print(f"Arduino Response: {response}")
 
    except serial.SerialException as e:
        print(f"Serial communication error: {e}")
    except Exception as e:
        print(f"Unexpected error in send_dmx: {e}")
 
def read_serial_response(ser):
    """Read response from serial."""
    try:
        if ser.in_waiting > 0:
            response = ser.readline().decode('utf-8', errors='ignore').strip()
            return response
    except serial.SerialTimeoutException:
        print("Serial read timeout.")
    except serial.SerialException as e:
        print(f"Serial error: {e}")
    except UnicodeDecodeError as e:
        print(f"Unicode decode error: {e}")
    return None
 
def main():
    model = YOLO(MODEL_NAME)
    cap = cv2.VideoCapture(1)
 
    if not cap.isOpened():
        print("Cannot open camera.")
        return
 
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)
    cap.set(cv2.CAP_PROP_FPS, 30)
 
    try:
        ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1)
        time.sleep(2)
        print(f"Serial connection established on {SERIAL_PORT}")
    except serial.SerialException as e:
        print(f"Failed to open serial port {SERIAL_PORT}: {e}")
        return
 
    # Variable to track previous bounding box height for push detection
    prev_bbox_height = None
    # Initialize the current DMX positions in the middle of the full range
    current_pan = (DMX_MIN + DMX_MAX) // 2
    current_tilt = (DMX_MIN + DMX_MAX) // 2
 
    try:
        while True:
            ret, frame = cap.read()
            if not ret:
                print("Failed to grab frame.")
                break
 
            results = model(frame)
            detections = results[0].boxes.xyxy.cpu().numpy()
            confidences = results[0].boxes.conf.cpu().numpy()
            class_ids = results[0].boxes.cls.cpu().numpy()
 
            person_detections = [
                (i, det) for i, (det, conf, cls_id) in enumerate(zip(detections, confidences, class_ids))
                if conf > CONFIDENCE_THRESHOLD and int(cls_id) == 0  # Class 0 = person
            ]
 
            if person_detections:
                # Pick the most central person based on horizontal distance from frame center
                frame_center_x = frame.shape[1] // 2
                frame_center_y = frame.shape[0] // 2
                center_person = min(
                    person_detections,
                    key=lambda p: abs(((p[1][0] + p[1][2]) / 2) - frame_center_x)
                )
                _, bbox = center_person
 
                # Extract bounding box info
                x1, y1, x2, y2 = map(int, bbox)
                bbox_center_x = (x1 + x2) // 2
                bbox_center_y = (y1 + y2) // 2
                bbox_height = y2 - y1
 
                distance = estimate_distance(bbox_height)
                print(f"Distance: {distance:.2f} m")
 
                # Determine if a push motion occurred by checking for a sudden increase in bbox height
                push_motion = False
                if prev_bbox_height is not None:
                    if (bbox_height - prev_bbox_height) > SHOVE_THRESHOLD:
                        push_motion = True
                        print("Push motion detected: moving light away.")
                prev_bbox_height = bbox_height  # Update for next iteration
 
                # Calculate normalized error relative to frame center (-1 to 1)
                error_x = (bbox_center_x - frame_center_x) / frame_center_x
                error_y = (bbox_center_y - frame_center_y) / frame_center_y
 
                # Normal tracking: adjust current DMX positions based on the normalized error
                current_pan += int(error_x * TRACKING_GAIN)
                current_tilt += int(error_y * TRACKING_GAIN)
 
                # If a push is detected, move the light further away by applying an opposite offset
                if push_motion:
                    # Here, subtracting the scaled error moves the light away from the person’s position
                    current_pan -= int(error_x * PUSH_GAIN)
                    current_tilt -= int(error_y * PUSH_GAIN)
 
                # Clamp the values within the full DMX range
                current_pan = max(DMX_MIN, min(DMX_MAX, current_pan))
                current_tilt = max(DMX_MIN, min(DMX_MAX, current_tilt))
 
                # Color logic remains based on the estimated distance
                if distance < MIN_DISTANCE:
                    color = (255, 0, 0)  # Red
                elif distance > MAX_DISTANCE:
                    color = (0, 0, 255)  # Blue
                else:
                    color = (0, 255, 0)  # Green
 
                send_dmx(ser, current_pan, current_tilt, color)
 
                # Visualization on the frame
                cv2.rectangle(frame, (x1, y1), (x2, y2), color, 2)
                cv2.putText(frame, f"{distance:.2f}m", (x1, y1 - 10),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.9, color, 2)
 
            cv2.imshow("Tracking", frame)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
 
    finally:
        cap.release()
        ser.close()
        cv2.destroyAllWindows()
        print("Resources released. Exiting.")
 
if __name__ == "__main__":
    main()
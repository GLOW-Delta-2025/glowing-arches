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

# --- DMX Calibration ---
DMX_LEFT = 70
DMX_RIGHT = 105
DMX_TOP = 10
DMX_BOTTOM = 105

SEND_INTERVAL = 100  # ms
millis_last_sent = time.time_ns()

# Motion detection parameters
SHOVE_THRESHOLD = 10  # pixels


def estimate_distance(bbox_height_pixels):
    """Estimate distance based on bounding box height."""
    if bbox_height_pixels <= 0:
        return -1
    return (KNOWN_OBJECT_HEIGHT * CAMERA_FOCAL_LENGTH_PIXELS) / bbox_height_pixels


def send_dmx(ser, pan, tilt, color):
    """Send DMX pan, tilt, and color values via serial."""
    global millis_last_sent
    current_time = time.time_ns()

    if (current_time - millis_last_sent) / 1_000_000 < SEND_INTERVAL:
        return

    millis_last_sent = current_time

    try:
        pan = max(1, min(255, int(pan)))
        tilt = max(1, min(255, int(tilt)))
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


def calculate_distance(x1, y1, x2, y2):
    """Calculate Euclidean distance between points."""
    return math.hypot(x2 - x1, y2 - y1)


def map_value(value, left_min, left_max, right_min, right_max):
    """Map value from one range to another."""
    left_span = left_max - left_min
    right_span = right_max - right_min
    value_scaled = float(value - left_min) / float(left_span)
    return int(right_min + (value_scaled * right_span))


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

    tracked_person_id = None
    tracked_person_position = None
    color = (255, 255, 255)  # Default color (white)

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
                # Pick the most central person
                frame_center_x = frame.shape[1] // 2
                center_person = min(
                    person_detections,
                    key=lambda p: abs(((p[1][0] + p[1][2]) / 2) - frame_center_x)
                )
                _, bbox = center_person

                # Extract bbox info
                x1, y1, x2, y2 = map(int, bbox)
                bbox_center_x = (x1 + x2) // 2
                bbox_center_y = (y1 + y2) // 2
                bbox_height = y2 - y1

                distance = estimate_distance(bbox_height)
                print(f"Distance: {distance:.2f} m")

                # Map position to DMX values
                pan_value = map_value(bbox_center_x, 0, frame.shape[1], DMX_LEFT, DMX_RIGHT)
                tilt_value = map_value(bbox_center_y, 0, frame.shape[0], DMX_TOP, DMX_BOTTOM)

                # Color logic based on distance
                if distance < MIN_DISTANCE:
                    color = (255, 0, 0)  # Red
                elif distance > MAX_DISTANCE:
                    color = (0, 0, 255)  # Blue
                else:
                    color = (0, 255, 0)  # Green

                send_dmx(ser, pan_value, tilt_value, color)

                # Visualize
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

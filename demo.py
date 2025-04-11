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

# These values are used for DMX commands.

DMX_LEFT = 70   # We no longer change pan, so use center value

DMX_RIGHT = 105

DMX_TOP = 10    # The top tilt position (light fully up)

DMX_BOTTOM = 105  # The bottom tilt position (light fully down)
 
SEND_INTERVAL = 100  # ms

millis_last_sent = time.time_ns()
 
# Motion detection parameter for push (shove) gesture

# (Not used in this vertical-only movement example.)

SHOVE_THRESHOLD = 40  # (Unused here)
 
def estimate_distance(bbox_height_pixels):

    """Estimate distance based on bounding box height."""

    if bbox_height_pixels <= 0:

        return -1

    return (KNOWN_OBJECT_HEIGHT * CAMERA_FOCAL_LENGTH_PIXELS) / bbox_height_pixels
 
def send_dmx(ser, pan, tilt, color):

    """Send DMX pan, tilt, and color values via serial."""

    global millis_last_sent

    current_time = time.time_ns()
 
    # Enforce sending interval

    if (current_time - millis_last_sent) / 1_000_000 < SEND_INTERVAL:

        return

    millis_last_sent = current_time
 
    try:

        # Clamp pan and tilt to full DMX range (1 to 255)

        pan = max(1, min(255, int(pan)))

        tilt = max(1, min(255, int(tilt)))

        red, green, blue = color
 
        # Build DMX command (the last value here is kept as 255)

        command = f"0:{pan},{tilt},{red},{green},{blue},0,0,255;\n"

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
 
def map_value(value, left_min, left_max, right_min, right_max):

    """Map value from one range to another."""

    # Avoid division by zero

    if left_max - left_min == 0:

        return right_min

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
 
    # Set camera resolution and FPS

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
 
    # For tracking vertical position

    last_bbox_center_y = None

    lost_detection_time = None  # Time when person was lost

    # Calculate the center positions for pan and tilt.

    center_pan = (DMX_LEFT + DMX_RIGHT) // 2

    center_tilt = (DMX_TOP + DMX_BOTTOM) // 2
 
    # Default color (set to green, similar to when a detected person is at a normal distance)

    default_color = (0, 255, 0)
 
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
 
            # Consider only person detections (class 0) above confidence threshold.

            person_detections = [

                (i, det) for i, (det, conf, cls_id) in enumerate(zip(detections, confidences, class_ids))

                if conf > CONFIDENCE_THRESHOLD and int(cls_id) == 0

            ]
 
            frame_center_y = frame.shape[0] // 2
 
            if person_detections:

                # Reset lost state when a person is detected.

                lost_detection_time = None
 
                # Pick the most central person horizontally (though we only use vertical information now)

                frame_center_x = frame.shape[1] // 2

                center_person = min(

                    person_detections,

                    key=lambda p: abs(((p[1][0] + p[1][2]) / 2) - frame_center_x)

                )

                _, bbox = center_person
 
                # Extract bounding box coordinates and compute vertical center.

                x1, y1, x2, y2 = map(int, bbox)

                bbox_center_y = (y1 + y2) // 2

                bbox_height = y2 - y1
 
                # Estimate distance (for possible color logic)

                distance = estimate_distance(bbox_height)

                print(f"Distance: {distance:.2f} m")
 
                # Use the mapping of the vertical coordinate to the DMX tilt range.

                # Here, we map 0 (top of frame) to DMX_TOP and frame bottom to DMX_BOTTOM.

                tilt_value = map_value(bbox_center_y, 0, frame.shape[0], DMX_TOP, DMX_BOTTOM)
 
                # Update the last detected vertical position

                last_bbox_center_y = bbox_center_y
 
                # Color logic (retain your original logic based on distance)

                if distance < MIN_DISTANCE:

                    color = (255, 0, 0)  # Red

                elif distance > MAX_DISTANCE:

                    color = (0, 0, 255)  # Blue

                else:

                    color = default_color
 
                send_dmx(ser, center_pan, tilt_value, color)
 
                # Draw bounding box and distance for visualization

                cv2.rectangle(frame, (x1, y1), (x2, y2), color, 2)

                cv2.putText(frame, f"{distance:.2f}m", (x1, y1 - 10),

                            cv2.FONT_HERSHEY_SIMPLEX, 0.9, color, 2)

            else:

                # No person detected.

                if last_bbox_center_y is not None:

                    # Begin lost-detection sequence if not already started.

                    if lost_detection_time is None:

                        lost_detection_time = time.time()

                        # Decide the extreme tilt target based on where the person was last seen.

                        if last_bbox_center_y < frame_center_y:

                            # Last seen in the top half; move the light fully downward.

                            target_tilt = DMX_BOTTOM

                        else:

                            # Last seen in the bottom half; move the light fully upward.

                            target_tilt = DMX_TOP
 
                        print("Person lost: moving light to extreme tilt position.")

                        send_dmx(ser, center_pan, target_tilt, default_color)

                    else:

                        # Check if 5 seconds have elapsed since detection was lost.

                        if time.time() - lost_detection_time > 5:

                            print("Resetting light to center position.")

                            send_dmx(ser, center_pan, center_tilt, default_color)

                            # Reset stored detection info to resume normal tracking

                            lost_detection_time = None

                            last_bbox_center_y = None
 
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

 
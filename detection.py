import cv2
from ultralytics import YOLO
import time
import serial

# --- Configuration ---
MODEL_NAME = "yolov8s.pt"
CONFIDENCE_THRESHOLD = 0.5
CAMERA_FOCAL_LENGTH_PIXELS = 1000  # Calibrate!
KNOWN_OBJECT_HEIGHT = 1.75
MAX_DISTANCE = 10
MIN_DISTANCE = 0
SERIAL_PORT = '/dev/tty.usbserial-2110'  # CHANGE THIS!
BAUD_RATE = 9600

# --- DMX 1 Calibration ---
# --- PAN ---
DMX_1_LEFT = 70
DMX_1_RIGHT = 105
# --- TILT ---
DMX_1_TOP = 0
DMX_1_BOTTOM = 105

# --- DMX 2 Calibration ---
# --- PAN ---
DMX_2_LEFT = 70
DMX_2_RIGHT = 105
# --- TILT ---
DMX_2_TOP = 0
DMX_2_BOTTOM = 105

# --- Tracking ---
tracked_person_id = None  # ID of the currently tracked person (None = no one tracked)


def estimate_distance(bbox_height_pixels):
    """Estimates distance based on bounding box height."""
    if bbox_height_pixels <= 0:
        return -1
    distance = (KNOWN_OBJECT_HEIGHT * CAMERA_FOCAL_LENGTH_PIXELS) / bbox_height_pixels
    return distance


def send_dmx(ser, pan1, tilt1, pan2=0, tilt2=0):
    """Sends DMX pan and tilt values for two individuals and reads the response."""
    try:
        # Clamp pan and tilt values for both individuals between 0 and 255
        pan1 = max(0, min(255, int(pan1)))
        tilt1 = max(0, min(255, int(tilt1)))
        pan2 = max(0, min(255, int(pan2)))
        tilt2 = max(0, min(255, int(tilt2)))

        # Format the message as '0:pan,tilt;1:pan,tilt'
        message = f"0:{pan1},{tilt1};1:{pan2},{tilt2}\n"

        # Send the DMX message over serial
        ser.write(message.encode())
        print(f"Sent DMX: {message.strip()}")

        # Read and print Arduino's response
        response = read_serial_response(ser)
        if response:
            print(f"Arduino: {response}")
    except serial.SerialException as e:
        print(f"Serial communication error: {e}")
    except Exception as e:
        print(f"An unexpected error occurred: {e}")

def read_serial_response(ser):
    """Reads a line from the serial port, handling timeouts and decoding."""
    try:
        if ser.in_waiting > 0:
            response = ser.readline().decode('utf-8', errors='ignore').strip()
            return response
        else:
             return None
    except serial.SerialTimeoutException:
        print("Serial read timeout.")
        return None
    except serial.SerialException as e:
        print(f"Serial error during read: {e}")
        return None
    except UnicodeDecodeError as e:
        print(f"Unicode decode error: {e}")
        return None


def main():
    global tracked_person_id  # Use the global variable

    model = YOLO(MODEL_NAME)
    cap = cv2.VideoCapture(0)
    if not cap.isOpened():
        print("Cannot open camera")
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

    while True:
        ret, frame = cap.read()
        if not ret:
            print("Can't receive frame (stream end?). Exiting ...")
            break

        frame_height, frame_width, _ = frame.shape

        start_time = time.time()
        results = model(frame, stream=False, conf=CONFIDENCE_THRESHOLD, classes=0)
        end_time = time.time()
        inference_time = end_time - start_time
        print(f"Inference time: {inference_time:.4f} seconds")

        annotated_frame = frame.copy()

        # --- Tracking Logic ---
        if tracked_person_id is None:  # No one currently tracked
            # Find the *first* person detected (within distance range)
            for r in results:
                boxes = r.boxes
                for box_idx, box in enumerate(boxes): #added enumerate for unique id
                    x1, y1, x2, y2 = [int(coord) for coord in box.xyxy[0]]
                    w = x2 - x1
                    h = y2 - y1
                    confidence = float(box.conf[0])
                    cls = int(box.cls[0])
                    # class_name = model.names[cls] if model.names else f"Class {cls}" # No longer needed here
                    distance = estimate_distance(h)

                    if MIN_DISTANCE <= distance <= MAX_DISTANCE:
                        tracked_person_id = box_idx  # Assign an ID (index) to the first valid detection
                        break  # Exit inner loop (boxes) after finding the first person
                if tracked_person_id is not None:
                    break  # Exit outer loop (results) after finding the first person

        # --- Control DMX based on tracked person (if any) ---
        if tracked_person_id is not None:
            tracked_box = None  # Initialize

            # Find the box with the tracked ID
            for r in results:
                boxes = r.boxes
                if tracked_person_id < len(boxes): # Check if valid
                    tracked_box = boxes[tracked_person_id]
                    break # Exit loop after box is found


            if tracked_box is not None:
                x1, y1, x2, y2 = [int(coord) for coord in tracked_box.xyxy[0]]
                w = x2 - x1
                h = y2 - y1
                confidence = float(tracked_box.conf[0])
                cls = int(tracked_box.cls[0])
                class_name = model.names[cls] if model.names else f"Class {cls}"

                distance = estimate_distance(h)

                # Check distance again (person might have moved out of range)
                if MIN_DISTANCE <= distance <= MAX_DISTANCE:
                    person_x = x1 + w // 2
                    person_y = y1 + h // 2
                    pan_value = int(DMX_1_RIGHT + (DMX_1_LEFT - DMX_1_RIGHT) * (person_x / frame_width))
                    tilt_value = int(DMX_1_BOTTOM + (DMX_1_TOP - DMX_1_BOTTOM) * (person_y / frame_height))
                    send_dmx(ser, pan_value, tilt_value)

                    label = f"{class_name}: {confidence:.2f}"
                    if distance != -1:
                        label += f" Dist: {distance:.2f}m"
                    cv2.rectangle(annotated_frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
                    cv2.putText(annotated_frame, label, (x1, y1 - 10), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
                else:
                    # Person moved out of range, stop tracking
                    tracked_person_id = None
            else: #if tracked_box is none, there are no boxes.
                tracked_person_id = None

        fps = 1 / inference_time if inference_time > 0 else 0
        cv2.putText(annotated_frame, f"FPS: {fps:.2f}", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)

        cv2.imshow("YOLOv8 Object Detection", annotated_frame)

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    cap.release()
    cv2.destroyAllWindows()
    if ser:
        ser.close()
        print("Serial connection closed.")

if __name__ == "__main__":
    main()
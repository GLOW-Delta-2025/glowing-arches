import cv2
from ultralytics import YOLO
import time
import serial
import math  # Import the math module for distance calculation
 
# --- Configuration ---
MODEL_NAME = "yolov8n.pt"
CONFIDENCE_THRESHOLD = 0.5
CAMERA_FOCAL_LENGTH_PIXELS = 1000  # Calibrate!
KNOWN_OBJECT_HEIGHT = 1.75
MAX_DISTANCE = 5
MIN_DISTANCE = 0
SERIAL_PORT = 'COM3'
BAUD_RATE = 9600
 
# --- DMX Pan Calibration ---
DMX_LEFT = 70
DMX_RIGHT = 105
 
# --- DMX Tilt Calibration ---
DMX_TOP = 10
DMX_BOTTOM = 105
 
SEND_INTERVAL = 100 #in ms
millis_last_sent = time.time_ns()
 
 
def estimate_distance(bbox_height_pixels):
    """Estimates distance based on bounding box height."""
    if bbox_height_pixels <= 0:
        return -1
    distance = (KNOWN_OBJECT_HEIGHT * CAMERA_FOCAL_LENGTH_PIXELS) / bbox_height_pixels
    return distance
 
def send_dmx(ser, pan, tilt):
    current_time = time.time_ns()
    if (current_time - millis_last_sent)/1000000 > SEND_INTERVAL:
        setMillisLastSent(current_time)
    else:
        return
    """Sends DMX pan and tilt values and reads the response."""
    # time.sleep(0.1)
    try:
        pan = max(1,  min(255, int(pan)))
        tilt = max(1, min(255, int(tilt)))
 
        spotlight_id = 0
        red = 255
        green = 255
        blue = 0
        white = 0
        mixed = 0 # this is random and it explored everything. do not touch
        dimming = 20 # 1 is no light 255 is full brightness
 
        # Build the DMX command string
        command = f"{spotlight_id}:{pan},{tilt},{red},{green},{blue},{white},{mixed},{dimming};\n"
 
        # Send to Arduino
        ser.write(command.encode())
        print(f"Sent DMX: Pan={pan}, Tilt={tilt}")
 
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
def getMillisLastSent():
    global millis_last_sent
    return millis_last_sent
def setMillisLastSent(millis):
    global millis_last_sent
    millis_last_sent = millis
 
def calculate_distance(x1, y1, x2, y2):
    """Calculates the Euclidean distance between two points."""
    return math.sqrt((x2 - x1) ** 2 + (y2 - y1) ** 2)
 
 
def main():
    model = YOLO(MODEL_NAME)
    cap = cv2.VideoCapture(1)
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
 
    tracked_person_id = None
    tracked_person_position = None
 
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
        valid_boxes = []
        valid_confidences = []
        valid_classes = []
 
        if results:
            for i, box in enumerate(results[0].boxes.xyxy.tolist()):
                x1, y1, x2, y2 = map(int, box)
                h = y2 - y1
                distance = estimate_distance(h)
                if MIN_DISTANCE <= distance <= MAX_DISTANCE:
                    valid_boxes.append(box)
                    valid_confidences.append(results[0].boxes.conf[i].item())
                    valid_classes.append(results[0].boxes.cls[i].item())
 
        if valid_boxes:
            if tracked_person_id is None:
                # If no person is tracked yet, track the first one
                tracked_person_id = 0
                x1, y1, x2, y2 = map(int, valid_boxes[0])
                tracked_person_position = (x1 + (x2 - x1) // 2, y1 + (y2 - y1) // 2)
            else:
                # Find the closest detected person to the tracked person's last position
                min_distance = float('inf')
                closest_person_index = -1
                for i, box in enumerate(valid_boxes):
                    x1, y1, x2, y2 = map(int, box)
                    center = (x1 + (x2 - x1) // 2, y1 + (y2 - y1) // 2)
                    distance = calculate_distance(tracked_person_position[0], tracked_person_position[1], center[0], center[1])
                    if distance < min_distance:
                        min_distance = distance
                        closest_person_index = i
 
                if closest_person_index != -1:
                    # Update tracked person
                    tracked_person_id = 0  # Keep it 0 for simplicity
                    x1, y1, x2, y2 = map(int, valid_boxes[closest_person_index])
                    tracked_person_position = (x1 + (x2 - x1) // 2, y1 + (y2 - y1) // 2)
 
                    # Draw bounding box and send DMX
                    w = x2 - x1
                    h = y2 - y1
                    confidence = valid_confidences[closest_person_index]
                    class_name = model.names[int(valid_classes[closest_person_index])] if model.names else f"Class {int(valid_classes[closest_person_index])}"
                    distance = estimate_distance(h)
 
                    pan_value = int(DMX_RIGHT + (DMX_LEFT - DMX_RIGHT) * (tracked_person_position[0] * -1 / frame_width))
                    tilt_value = int(DMX_BOTTOM + (DMX_TOP - DMX_BOTTOM) * (tracked_person_position[1] * -1 / frame_height))
                    send_dmx(ser, pan_value, tilt_value)
                    person_x = x1 + w // 2
                    person_y = y1 + h // 2
                    if distance < 1.0:
                        # Flip the pan/tilt to look away
                        flipped_x = frame_width - person_x
                        flipped_y = frame_height - person_y
                        pan_value = int(DMX_RIGHT + (DMX_LEFT - DMX_RIGHT) * (flipped_x / frame_width))
                        tilt_value = int(DMX_BOTTOM + (DMX_TOP - DMX_BOTTOM) * (flipped_y / frame_height))
                    else:
                        # Normal tracking
                        pan_value = int(DMX_RIGHT + (DMX_LEFT - DMX_RIGHT) * (person_x / frame_width))
                        tilt_value = int(DMX_BOTTOM + (DMX_TOP - DMX_BOTTOM) * (person_y / frame_height))
                    print("pan_value", pan_value)
                    current_time = time.time_ns()
                    if (current_time - millis_last_sent)/1000000 > SEND_INTERVAL:
                        send_dmx(ser, pan_value, tilt_value)
                        setMillisLastSent(current_time)
 
                    label = f"ID {tracked_person_id} {class_name}: {confidence:.2f}"
                    if distance != -1:
                        label += f" Dist: {distance:.2f}m"
                    cv2.rectangle(annotated_frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
                    cv2.putText(annotated_frame, label, (x1, y1 - 10), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
        elif tracked_person_position is not None:
            # If the tracked person is lost, keep the last position for a while (optional)
            # For simplicity, we'll just keep the last position
            pan_value = int(DMX_RIGHT + (DMX_LEFT - DMX_RIGHT) * (tracked_person_position[0] / frame_width))
            flipped_pan = int(DMX_LEFT + (DMX_RIGHT - DMX_LEFT) * (1 - (tracked_person_position[0] / frame_width)))
            tilt_value = int(DMX_BOTTOM + (DMX_TOP - DMX_BOTTOM) * (tracked_person_position[1] / frame_height))
            send_dmx(ser, pan_value, tilt_value)
 
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
 
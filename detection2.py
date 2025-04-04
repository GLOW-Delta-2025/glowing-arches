import cv2
from ultralytics import YOLO
import time
import serial

# --- Configuration ---
MODEL_NAME = "yolov8n.pt"
CONFIDENCE_THRESHOLD = 0.5
CAMERA_FOCAL_LENGTH_PIXELS = 1000  # Calibrate!
KNOWN_OBJECT_HEIGHT = 1.75
MAX_DISTANCE = 12
MIN_DISTANCE = 0
SERIAL_PORT = '/dev/ttyUSB0'
BAUD_RATE = 9600

# --- DMX Pan Calibration ---
DMX_LEFT = 70
DMX_RIGHT = 105

# --- DMX Tilt Calibration ---
DMX_TOP = 0
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
    # time.sleep(0.1)
    """Sends DMX pan and tilt values and reads the response."""
    try:
        spotlight_id = 0
        red = 0
        green = 255
        blue = 0
        white = 0
        mixed = 0 # this is random and it explored everything. do not touch
        dimming = 20 # 1 is no light 255 is full brightness

        # Build the DMX command string
        command = f"{spotlight_id}:{pan},{tilt},{red},{green},{blue},{white},{mixed},{dimming};\n"
        print(command)

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

def main():
    model = YOLO(MODEL_NAME)
    cap = cv2.VideoCapture(4)
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
        for r in results:
            boxes = r.boxes
            for box_idx, box in enumerate(boxes):
                x1, y1, x2, y2 = [int(coord) for coord in box.xyxy[0]]
                w = x2 - x1
                h = y2 - y1
                confidence = float(box.conf[0])
                cls = int(box.cls[0])
                class_name = model.names[cls]
                # class_name = model.names[cls] if model.names else f"Class {cls}"
                distance = estimate_distance(h)

                if MIN_DISTANCE <= distance <= MAX_DISTANCE:
                    person_x = x1 + w // 2
                    person_y = y1 + h // 2
                    pan_value = int(DMX_RIGHT + (DMX_LEFT - DMX_RIGHT) * (person_x / frame_width))
                    tilt_value = int(DMX_BOTTOM + (DMX_TOP - DMX_BOTTOM) * (person_y / frame_height))
                    print("pan_value", pan_value)
                    current_time = time.time_ns()
                    if (current_time - millis_last_sent)/1000000 > SEND_INTERVAL:
                        send_dmx(ser, pan_value, tilt_value)
                        setMillisLastSent(current_time)

                    label = f"ID {box_idx} {class_name}: {confidence:.2f}"
                    if distance != -1:
                        label += f" Dist: {distance:.2f}m"
                    cv2.rectangle(annotated_frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
                    cv2.putText(annotated_frame, label, (x1, y1 - 10), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)

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
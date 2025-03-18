import cv2
from ultralytics import YOLO
import time
import serial  # Import the serial library

# --- Configuration ---
MODEL_NAME = "yolov8s.pt"
CONFIDENCE_THRESHOLD = 0.5
CAMERA_FOCAL_LENGTH_PIXELS = 1000  # Calibrate!
KNOWN_OBJECT_HEIGHT = 1.75
MAX_DISTANCE = 10
MIN_DISTANCE = 0
SERIAL_PORT = 'COM3'  #  <--- CHANGE THIS to your serial port
BAUD_RATE = 9600      #  <--- Make sure this matches your DMX device

def estimate_distance(bbox_height_pixels):
    """Estimates distance based on bounding box height."""
    if bbox_height_pixels <= 0:
        return -1
    distance = (KNOWN_OBJECT_HEIGHT * CAMERA_FOCAL_LENGTH_PIXELS) / bbox_height_pixels
    return distance

def send_dmx(ser, pan, tilt):
    """Sends DMX pan and tilt values to the serial port."""
    try:
        # Ensure values are within 0-255 range
        pan = max(0, min(255, int(pan)))
        tilt = max(0, min(255, int(tilt)))
        ser.write(f"{pan},{tilt}\n".encode())
        print(f"Sent DMX: Pan={pan}, Tilt={tilt}") #log
    except serial.SerialException as e:
        print(f"Serial communication error: {e}")
    except Exception as e:
        print(f"An unexpected error occurred: {e}")


def main():
    model = YOLO(MODEL_NAME)
    cap = cv2.VideoCapture(0)
    if not cap.isOpened():
        print("Cannot open camera")
        return

    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)
    cap.set(cv2.CAP_PROP_FPS, 30)

     # --- Serial Connection Setup ---
    try:
        ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1)
        time.sleep(2)  # Wait for connection to establish
        print(f"Serial connection established on {SERIAL_PORT}")
    except serial.SerialException as e:
        print(f"Failed to open serial port {SERIAL_PORT}: {e}")
        return  # Exit if serial connection fails
    except Exception as e:
        print(f"An unexpected error occurred: {e}")
        return


    while True:
        ret, frame = cap.read()
        if not ret:
            print("Can't receive frame (stream end?). Exiting ...")
            break

        frame_height, frame_width, _ = frame.shape #get heigt and width

        start_time = time.time()
        results = model(frame, stream=False, conf=CONFIDENCE_THRESHOLD, classes=0)
        end_time = time.time()
        inference_time = end_time - start_time
        print(f"Inference time: {inference_time:.4f} seconds")

        annotated_frame = frame.copy()
        for r in results:
            boxes = r.boxes
            for box in boxes:
                x1, y1, x2, y2 = [int(coord) for coord in box.xyxy[0]]
                w = x2 - x1
                h = y2 - y1
                confidence = float(box.conf[0])
                cls = int(box.cls[0])
                class_name = model.names[cls] if model.names else f"Class {cls}"

                distance = estimate_distance(h)

                if MIN_DISTANCE <= distance <= MAX_DISTANCE:
                    # --- Calculate center of the bounding box ---
                    person_x = x1 + w // 2
                    person_y = y1 + h // 2

                    # --- Normalize to DMX range (0-255) ---
                    pan_value = int((person_x / frame_width) * 255)
                    tilt_value = int((person_y / frame_height) * 255)

                    # --- Send DMX values ---
                    send_dmx(ser, pan_value, tilt_value)

                    label = f"{class_name}: {confidence:.2f}"
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
        ser.close() #close serial connection
        print("Serial connection closed.")


if __name__ == "__main__":
    main()
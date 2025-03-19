import cv2
from ultralytics import YOLO
import time
import serial
from collections import OrderedDict
import numpy as np

# --- Configuration ---
MODEL_NAME = "yolov8s.pt"
CONFIDENCE_THRESHOLD = 0.5
CAMERA_FOCAL_LENGTH_PIXELS = 1000  # Set appropriately for your camera
KNOWN_OBJECT_HEIGHT = 1.75  # Average height of a person (meters)
MAX_DISTANCE = 10  # Maximum allowable distance (meters)
MIN_DISTANCE = 0  # Minimum allowable distance (meters)
SERIAL_PORT = '/dev/tty.usbserial-2110'  # CHANGE THIS!
BAUD_RATE = 9600

# --- DMX Calibration ---
DMX_LEFT = 70
DMX_RIGHT = 105
DMX_TOP = 0
DMX_BOTTOM = 105

# --- Tracking States ---
tracked_objects = OrderedDict()  # Active object trackers
next_object_id = 0  # Next ID to assign
tracked_person_id_1 = None  # Person ID #1 to control DMX
tracked_person_id_2 = None  # Person ID #2 to control DMX
lost_objects = {}  # Tracks lost object IDs and how long they've been missing


def estimate_distance(bbox_height_pixels):
    """Estimates distance based on bounding box height."""
    if bbox_height_pixels <= 0:
        return -1
    return (KNOWN_OBJECT_HEIGHT * CAMERA_FOCAL_LENGTH_PIXELS) / bbox_height_pixels


def calculate_centroid(x1, y1, x2, y2):
    """Calculates the centroid of a bounding box."""
    return int((x1 + x2) / 2), int((y1 + y2) / 2)


def match_objects(detected_boxes, centroids):
    """
    Match currently tracked objects with newly detected objects (boxes).
    Uses simple centroid matching with minimum Euclidean distance.
    """
    global tracked_objects, next_object_id

    updated_objects = OrderedDict()
    used_centroids = set()

    # Match detected centroids to tracked objects by proximity
    for obj_id, tracked_centroid in tracked_objects.items():
        min_distance = float('inf')
        matched_index = None

        for i, centroid in enumerate(centroids):
            if i in used_centroids:
                continue

            distance = np.linalg.norm(np.array(tracked_centroid) - np.array(centroid))
            if distance < min_distance:
                min_distance = distance
                matched_index = i

        if matched_index is not None and min_distance < 50:  # Matching threshold
            updated_objects[obj_id] = centroids[matched_index]
            used_centroids.add(matched_index)
        else:
            lost_objects[obj_id] = True

    # Assign new IDs to unmatched detections
    for i, centroid in enumerate(centroids):
        if i not in used_centroids:
            updated_objects[next_object_id] = centroid
            next_object_id += 1

    tracked_objects = updated_objects


def send_dmx_combined(ser, dmx_message):
    """Sends consolidated DMX message via serial."""
    try:
        ser.write(f"{dmx_message}\n".encode())
        print(f"Sent DMX Message: {dmx_message}")
    except serial.SerialException as e:
        print(f"Serial communication error: {e}")
    except Exception as e:
        print(f"Unexpected error occurred while sending DMX: {e}")


def convert_to_dmx(value, value_min, value_max, dmx_min, dmx_max):
    """Converts values (like coordinates) to DMX range."""
    value = max(value_min, min(value, value_max))  # Clamp to value range
    return int(dmx_min + (dmx_max - dmx_min) * ((value - value_min) / (value_max - value_min)))


def main():
    global tracked_person_id_1, tracked_person_id_2  # Allow modifying tracked persons globally

    model = YOLO(MODEL_NAME)
    cap = cv2.VideoCapture(0)

    if not cap.isOpened():
        print("Camera not accessible")
        return

    try:
        ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1)
        time.sleep(2)  # Let the serial connection stabilize
        print(f"Serial connection established on {SERIAL_PORT}")
    except serial.SerialException as e:
        print(f"Failed to open serial port {SERIAL_PORT}: {e}")
        ser = None

    while True:
        ret, frame = cap.read()
        if not ret:
            print("Can't fetch frame (stream ended?). Exiting...")
            break

        frame_height, frame_width, _ = frame.shape
        results = model(frame, stream=False, conf=CONFIDENCE_THRESHOLD, classes=0)

        detected_boxes = []
        centroids = []

        # Process detected bounding boxes
        for r in results:
            boxes = r.boxes
            for box in boxes:
                x1, y1, x2, y2 = [int(coord) for coord in box.xyxy[0]]
                w = x2 - x1
                h = y2 - y1
                bbox_confidence = float(box.conf[0])

                # Estimate distance and filter by range
                distance = estimate_distance(h)
                if MIN_DISTANCE <= distance <= MAX_DISTANCE:
                    detected_boxes.append((x1, y1, x2, y2))
                    centroids.append(calculate_centroid(x1, y1, x2, y2))

        # Update object tracking
        match_objects(detected_boxes, centroids)

        # Assign DMX channels based on tracked IDs
        tracked_ids = list(tracked_objects.keys())
        if len(tracked_ids) > 0:
            tracked_person_id_1 = tracked_ids[0]
        if len(tracked_ids) > 1:
            tracked_person_id_2 = tracked_ids[1]

        dmx_message = ""  # Consolidated DMX message

        # Handle DMX movement for Person #1
        if tracked_person_id_1 in tracked_objects:
            person_x_1, person_y_1 = tracked_objects[tracked_person_id_1]
            pan_value_1 = convert_to_dmx(person_x_1, 0, frame_width, DMX_LEFT, DMX_RIGHT)
            tilt_value_1 = convert_to_dmx(person_y_1, 0, frame_height, DMX_TOP, DMX_BOTTOM)
            dmx_message += f"0:{pan_value_1},{tilt_value_1};"

        # Handle DMX movement for Person #2
        if tracked_person_id_2 in tracked_objects:
            person_x_2, person_y_2 = tracked_objects[tracked_person_id_2]
            pan_value_2 = convert_to_dmx(person_x_2, 0, frame_width, DMX_LEFT, DMX_RIGHT)
            tilt_value_2 = convert_to_dmx(person_y_2, 0, frame_height, DMX_TOP, DMX_BOTTOM)
            dmx_message += f"1:{pan_value_2},{tilt_value_2};"
        else:
            # Default DMX signal for Person #2 when not detected but Person #1 is
            if tracked_person_id_1 in tracked_objects:
                dmx_message += "1:0,0;"

        # Send DMX message to serial port
        if ser and dmx_message:
            send_dmx_combined(ser, dmx_message.strip(";"))  # Remove trailing semicolon

        # Annotate the frame
        for obj_id, centroid in tracked_objects.items():
            index = tracked_ids.index(obj_id)
            box = detected_boxes[index]
            x1, y1, x2, y2 = box

            if obj_id == tracked_person_id_1:
                color = (0, 255, 0)  # Green for Person #1
                label_text = f"Person #1: ID {obj_id}"
            elif obj_id == tracked_person_id_2:
                color = (0, 0, 255)  # Red for Person #2
                label_text = f"Person #2: ID {obj_id}"
            else:
                color = (255, 0, 0)  # Blue for others
                label_text = f"ID: {obj_id}"

            cv2.rectangle(frame, (x1, y1), (x2, y2), color, 2)
            cv2.putText(frame, label_text, (x1, y1 - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.6, color, 2)

        # Display the frame
        cv2.imshow("YOLOv8 Multi-Person Tracker", frame)

        # Break on 'q' key press
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    cap.release()
    cv2.destroyAllWindows()
    if ser:
        ser.close()
        print("Serial connection closed.")


if __name__ == "__main__":
    main()
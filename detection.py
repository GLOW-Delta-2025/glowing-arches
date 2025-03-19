import cv2
from ultralytics import YOLO
import time
import serial
from collections import OrderedDict

# --- Configuration ---
MODEL_NAME = "yolov8s.pt"
CONFIDENCE_THRESHOLD = 0.5
CAMERA_FOCAL_LENGTH_PIXELS = 1000  # Set appropriately for your camera
KNOWN_OBJECT_HEIGHT = 1.75  # Average height of a person
MAX_DISTANCE = 10  # Maximum allowable distance (meters)
MIN_DISTANCE = 0  # Minimum allowable distance
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
tracked_person_id = None  # Person ID to control DMX (focus tracking)
lost_objects = {}  # Tracks lost object IDs and how long they've been missing


def estimate_distance(bbox_height_pixels):
    """Estimates distance based on bounding box height."""
    if bbox_height_pixels <= 0:
        return -1
    distance = (KNOWN_OBJECT_HEIGHT * CAMERA_FOCAL_LENGTH_PIXELS) / bbox_height_pixels
    return distance


def calculate_centroid(x1, y1, x2, y2):
    """Calculates the centroid of a bounding box."""
    return int((x1 + x2) / 2), int((y1 + y2) / 2)


def match_objects(detected_boxes, frame_centroids):
    """
    Match currently tracked objects with newly detected objects (boxes).
    Uses simple centroid matching with minimum Euclidean distance.
    """
    global tracked_objects, next_object_id

    updated_objects = OrderedDict()  # Store updated object positions
    new_detected_ids = set()  # New detections

    for i, centroid in enumerate(frame_centroids):
        min_distance = float('inf')
        matched_id = None
        # Compare detected objects with tracked objects
        for obj_id, tracked_centroid in tracked_objects.items():
            distance = ((centroid[0] - tracked_centroid[0]) ** 2 +
                        (centroid[1] - tracked_centroid[1]) ** 2) ** 0.5
            if distance < min_distance:
                min_distance = distance
                matched_id = obj_id

        # Threshold for matching: assign new ID if no close match
        if min_distance > 50:  # Pixel distance threshold
            new_detected_ids.add(i)
        else:
            updated_objects[matched_id] = centroid

    # Assign new IDs to unmatched detections
    for i in new_detected_ids:
        updated_objects[next_object_id] = frame_centroids[i]
        next_object_id += 1

    # Update tracked objects and return updated dict
    tracked_objects = updated_objects


def send_dmx(ser, pan, tilt):
    """Sends DMX pan and tilt values via serial."""
    try:
        pan = max(0, min(255, int(pan)))
        tilt = max(0, min(255, int(tilt)))
        ser.write(f"{pan},{tilt}\n".encode())
        print(f"Sent DMX: Pan={pan}, Tilt={tilt}")
    except serial.SerialException as e:
        print(f"Serial communication error: {e}")
    except Exception as e:
        print(f"Unexpected error occurred while sending DMX: {e}")


def main():
    global tracked_person_id  # Allow modifying tracked person globally

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
        return

    while True:
        ret, frame = cap.read()
        if not ret:
            print("Can't fetch frame (stream ended?). Exiting...")
            break

        frame_height, frame_width, _ = frame.shape
        results = model(frame, stream=False, conf=CONFIDENCE_THRESHOLD, classes=0)

        # List of detected bounding boxes and centroids
        detected_boxes = []
        centroids = []

        # Process detected bounding boxes
        for r in results:
            boxes = r.boxes
            for box in boxes:
                x1, y1, x2, y2 = [int(coord) for coord in box.xyxy[0]]
                w = x2 - x1
                h = y2 - y1
                confidence = float(box.conf[0])

                # Estimate distance
                distance = estimate_distance(h)
                if MIN_DISTANCE <= distance <= MAX_DISTANCE:
                    detected_boxes.append((x1, y1, x2, y2))
                    centroids.append(calculate_centroid(x1, y1, x2, y2))

        # Update tracking
        match_objects(detected_boxes, centroids)

        # Annotate detected people and tracked IDs
        for obj_id, centroid in tracked_objects.items():
            if obj_id in tracked_objects:
                # Get corresponding bounding box for the ID
                index = list(tracked_objects.keys()).index(obj_id)
                box = detected_boxes[index]

                # Unpack bounding box coordinates
                x1, y1, x2, y2 = box

                # Green Box for Tracked Person
                if obj_id == tracked_person_id:
                    color = (0, 255, 0)  # Green
                    label_text = f"Tracked: ID {obj_id}"
                else:
                    color = (255, 0, 0)  # Blue for other IDs
                    label_text = f"ID: {obj_id}"

                # Draw bounding boxes and text
                cv2.rectangle(frame, (x1, y1), (x2, y2), color, 2)
                cv2.putText(frame, label_text, (x1, y1 - 10),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, color, 2)

        # Assign tracked person if none is set
        if tracked_person_id is None and tracked_objects:
            tracked_person_id = list(tracked_objects.keys())[0]  # Lock onto the first detected person

        # Send DMX commands for tracked person
        if tracked_person_id in tracked_objects:
            tracked_centroid = tracked_objects[tracked_person_id]
            person_x, person_y = tracked_centroid

            # Convert object centroid position to DMX pan/tilt values
            pan_value = int(DMX_RIGHT + (DMX_LEFT - DMX_RIGHT) * (person_x / frame_width))
            tilt_value = int(DMX_BOTTOM + (DMX_TOP - DMX_BOTTOM) * (person_y / frame_height))

            send_dmx(ser, pan_value, tilt_value)
        else:
            tracked_person_id = None  # Reset tracking if tracked person is lost

        # Display video stream
        cv2.imshow("YOLOv8 Multi-Object Tracker", frame)
        if cv2.waitKey(1) & 0xFF == ord('q'):  # Press 'q' to quit
            break

    cap.release()
    cv2.destroyAllWindows()
    if ser:
        ser.close()
        print("Serial connection closed.")


if __name__ == "__main__":
    main()
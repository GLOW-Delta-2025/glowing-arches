import random
import cv2

CONFIDENCE_THRESHOLD = 0.5

# Define possible colors for detections
COLORS = ["RED", "GREEN", "BLUE"]

def assign_random_color():
    """Assign a random color from the defined COLORS."""
    return random.choice(COLORS)

def default_detection(yolo_model, frame, classes=None):
    """
    Default detection logic using YOLO. Processes the frame and performs object detection.

    Args:
        yolo_model: The initialized YOLO model.
        frame: The current video frame.
        classes: List of class IDs to detect, or None to detect all classes.

    Returns:
        A tuple (annotated_frame, dmx_coordinates, results):
        - annotated_frame: The frame annotated with detection results.
        - dmx_coordinates: List of coordinates for detected objects.
        - results: The raw YOLO detection results.
    """
    # Perform detection on the frame
    results = yolo_model(frame, stream=False, classes=classes, conf=CONFIDENCE_THRESHOLD)

    # Clone the frame for annotations
    annotated_frame = frame.copy()

    # Coordinates list for sending to DMX (supports up to 2 lights)
    dmx_coordinates = []

    # Loop through each detection, draw boxes, and collect coordinates
    for r in results:
        for box in r.boxes:
            # Get bounding box coordinates
            x1, y1, x2, y2 = map(int, box.xyxy[0])  # Convert to integer values for OpenCV
            confidence = box.conf[0]  # Confidence score

            # Only process if confidence is above threshold
            if confidence > CONFIDENCE_THRESHOLD:
                # Draw rectangle around detected object
                cv2.rectangle(annotated_frame, (x1, y1), (x2, y2), (0, 255, 0), 2)

                # Calculate bottom center (camera coordinates)
                bottom_center_x = (x1 + x2) // 2
                bottom_center_y = y2

                # Assign a random color to the detected object
                color = assign_random_color()

                # Append to DMX coordinates (only collect up to 2 lights)
                if len(dmx_coordinates) < 2:
                    dmx_coordinates.append({
                        "x": bottom_center_x,
                        "y": bottom_center_y,
                        "color": color
                    })

                # Display the color next to the bounding box
                color_text_pos = (x1, y1 - 10)
                cv2.putText(annotated_frame, color, color_text_pos, cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 1)

    return annotated_frame, dmx_coordinates, results
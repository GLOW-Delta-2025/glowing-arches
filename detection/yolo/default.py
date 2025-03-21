import cv2

CONFIDENCE_THRESHOLD = 0.5


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

                # Append to DMX coordinates (only collect up to 2 lights)
                if len(dmx_coordinates) < 2:
                    dmx_coordinates.append((bottom_center_x, bottom_center_y))

    return annotated_frame, dmx_coordinates, results
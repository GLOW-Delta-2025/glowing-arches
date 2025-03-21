import cv2
import serial
import time
from dmx import send_dmx
from ultralytics import YOLO

CONFIDENCE_THRESHOLD = 0.5
SERIAL_PORT = '/dev/tty.usbserial-110'  # Update this to your serial port
BAUD_RATE = 9600

def main():
    model_name = "yolov8n.pt"

    # Initialize YOLO model
    yolo = YOLO(model_name)

    # Open webcam
    cap = cv2.VideoCapture(0)
    if not cap.isOpened():
        print("Error: Could not open webcam.")
        return

    # Set camera properties (resolution + FPS)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)
    cap.set(cv2.CAP_PROP_FPS, 30)

    # Open a persistent serial connection
    try:
        serial_connection = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1)
        print("Serial connection established.")
    except serial.SerialException as e:
        print(f"Error establishing serial connection: {e}")
        return

    # Variables for calculating FPS
    prev_time = 0  # To store the time of the previous frame

    # Camera loop
    try:
        while cap.isOpened():
            ret, frame = cap.read()
            if not ret:
                print("Error: Could not read frame.")
                break

            # Calculate FPS
            current_time = time.time()
            fps = 1 / (current_time - prev_time)  # FPS calculation
            prev_time = current_time

            # Perform detection on the frame
            results = yolo(frame, stream=False, classes=0, conf=CONFIDENCE_THRESHOLD)

            # Clone the frame for annotations
            annotated_frame = frame.copy()

            # Coordinates list for sending to DMX (supports up to 2 lights)
            dmx_coordinates = []

            # Loop through each detection, draw boxes, and put labels
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

            # Send DMX data for calculated coordinates
            if dmx_coordinates:
                send_dmx(dmx_coordinates, serial_connection)

            # Overlay the FPS in the top-left corner of the video feed
            fps_text = f"FPS: {fps:.2f}"
            cv2.putText(annotated_frame, fps_text, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 255), 2)

            # Show the video feed with annotations
            cv2.imshow("YOLO Detection", annotated_frame)

            # Break the loop if 'q' is pressed
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

    finally:
        # Ensure resources are released
        cap.release()
        cv2.destroyAllWindows()

        # Close the serial connection
        if serial_connection.is_open:
            serial_connection.close()
            print("Serial connection closed.")

if __name__ == "__main__":
    main()
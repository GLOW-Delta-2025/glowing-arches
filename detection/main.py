import cv2
import serial
import time

from yolo.default import default_detection
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

            # Perform detection using external logic
            annotated_frame, dmx_coordinates, results = default_detection(yolo, frame, classes=0)

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
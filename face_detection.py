import cv2
import mediapipe as mp
import time

def process_frames(cap, face_detection, output_window=True):
    """Processes the camera stream, detects faces, and displays structured results.

    Args:
        cap: The OpenCV VideoCapture object.
        face_detection: The MediaPipe FaceDetection object.
        output_window: Boolean, whether to show a separate output window.
    """
    frame_count = 0
    fps = 0
    start_time = time.time()

    while cap.isOpened():
        success, frame = cap.read()
        if not success:
            print("Ignoring empty camera frame.")
            break

        frame_count += 1

        # Convert the BGR image to RGB.
        frame_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        results = face_detection.process(frame_rgb)

        # Draw the face detection annotations on the image (for the output window).
        annotated_frame = frame.copy()

        # --- Structured Output Section ---
        if results.detections:
            print("FaceDetectionResult:")
            print("  Detections:")
            for i, detection in enumerate(results.detections):
                print(f"    Detection #{i}:")

                # Bounding Box
                bboxC = detection.location_data.relative_bounding_box
                ih, iw, _ = annotated_frame.shape
                bbox = int(bboxC.xmin * iw), int(bboxC.ymin * ih), \
                       int(bboxC.width * iw), int(bboxC.height * ih)
                print("      BoundingBox:")
                print(f"        origin_x: {bbox[0]}")
                print(f"        origin_y: {bbox[1]}")
                print(f"        width: {bbox[2]}")
                print(f"        height: {bbox[3]}")

                # Categories (usually just one category: "face")
                print("      Categories:")
                print(f"        Category #0:")  #  MediaPipe Face Detection only has one category.
                print(f"          index: 0")  # Always 0 for face detection
                print(f"          score: {detection.score[0]}")


                # Keypoints (eyes, nose, mouth, ears)
                print("      NormalizedKeypoints:")
                for j, keypoint in enumerate(detection.location_data.relative_keypoints):
                    print(f"        NormalizedKeypoint #{j}:")
                    print(f"          x: {keypoint.x}")
                    print(f"          y: {keypoint.y}")

                # --- Drawing for the output window (can be inside the loop) ---
                cv2.rectangle(annotated_frame, bbox, (0, 255, 0), 2)
                cv2.putText(annotated_frame, f'{int(detection.score[0] * 100)}%',
                            (bbox[0], bbox[1] - 20), cv2.FONT_HERSHEY_PLAIN, 2, (0, 255, 0), 2)
        else:
            print("No faces detected.")  # Indicate when no faces are found

        # --- End of Structured Output Section ---


        # Calculate FPS
        current_time = time.time()
        elapsed_time = current_time - start_time
        if elapsed_time >= 1.0:  # Calculate FPS every second
            fps = frame_count / elapsed_time
            start_time = current_time
            frame_count = 0

        # Display FPS on the frame (for the output window)
        cv2.putText(annotated_frame, f"FPS: {fps:.2f}", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)

        if output_window:
           cv2.imshow('MediaPipe Face Detection', annotated_frame)

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    cap.release()
    cv2.destroyAllWindows()
def main():
    """Main function to set up camera and MediaPipe."""

    # Initialize MediaPipe Face Detection.
    mp_face_detection = mp.solutions.face_detection
    face_detection = mp_face_detection.FaceDetection(min_detection_confidence=0.5)

    # For webcam input:
    cap = cv2.VideoCapture(0)  # 0 is usually the default webcam.

    if not cap.isOpened():
        print("Cannot open camera")
        exit()

    # Set resolution and FPS (optional, but recommended)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)
    cap.set(cv2.CAP_PROP_FPS, 30)

    # Check actual camera settings
    actual_width = cap.get(cv2.CAP_PROP_FRAME_WIDTH)
    actual_height = cap.get(cv2.CAP_PROP_FRAME_HEIGHT)
    actual_fps = cap.get(cv2.CAP_PROP_FPS)
    print(f"Actual Resolution: {actual_width}x{actual_height}")
    print(f"Actual FPS: {actual_fps}")

    process_frames(cap, face_detection)

if __name__ == "__main__":
    main()
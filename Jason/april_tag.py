import cv2
import numpy as np
from pupil_apriltags import Detector
import time
import os

# Parameters
FULL_DETECTION_INTERVAL = 10  # Run full detection every 10 frames
ROI_MARGIN = 50               # Extra pixels around the detected tag for ROI
LK_PARAMS = dict(winSize=(15, 15), maxLevel=2,
                 criteria=(cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, 10, 0.03))
DECISION_MARGIN_THRESHOLD = 50  # Reject weak detections
MAX_MOVEMENT_THRESHOLD = 50     # Max allowed movement between frames to filter outliers

# Set DLL directory for AprilTag DLLs
os.add_dll_directory(r"D:\Anaconda\envs\gym_env39\lib\site-packages\pupil_apriltags.libs")

# Function to create detector with given quad_decimate
def create_detector(quad_decimate):
    return Detector(families='tag16h5', 
                    nthreads=4, 
                    quad_decimate=quad_decimate,  
                    quad_sigma=0.8,  
                    refine_edges=True)

# Start with a decimation factor of 1.0 (for better accuracy at distance)
quad_dec = 0.5
at_detector = create_detector(quad_dec)

# Video capture setup
cap = cv2.VideoCapture(0)
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1920)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 1080)
cap.set(cv2.CAP_PROP_FPS, 60)
# cap.set(cv2.CAP_PROP_ZOOM, 2.0)

# Tracking variables
tags_dict = {}   # Store tag positions: {tag_id: (x, y)}
frame_count = 0  # Track frame number
prev_gray = None # Previous grayscale frame for Optical Flow

while True:
    ret, frame = cap.read()
    if not ret:
        break

    start_time = time.perf_counter()
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

    # Full detection every FULL_DETECTION_INTERVAL frames or if we have no tracked tags.
    if frame_count % FULL_DETECTION_INTERVAL == 0 or not tags_dict:
        detected_tags = at_detector.detect(gray, estimate_tag_pose=True,
                                           camera_params=(600, 600, 320, 240), tag_size=0.082)
        # If no tags detected, try increasing quad_decimate (faster detection on downsampled image)
        if len(detected_tags) == 0 and quad_dec == 0.5:
            quad_dec = 2.0
            at_detector = create_detector(quad_dec)
        elif len(detected_tags) > 0 and quad_dec != 0.5:
            # If we detect tags with the higher resolution detector, revert back.
            quad_dec = 0.5
            at_detector = create_detector(quad_dec)

        # Clear tracked tags and update from full detection.
        tags_dict.clear()
        for tag in detected_tags:
            # Optionally, you can check decision margin here if needed.
            if tag.decision_margin >= DECISION_MARGIN_THRESHOLD:
                tags_dict[tag.tag_id] = tag.center.astype(int)
    else:
        # Use Optical Flow to track previous tag positions
        if prev_gray is not None and tags_dict:
            prev_pts = np.array(list(tags_dict.values()), dtype=np.float32).reshape(-1, 1, 2)
            new_pts, status, _ = cv2.calcOpticalFlowPyrLK(prev_gray, gray, prev_pts, None, **LK_PARAMS)
            # Update tag positions if tracking is successful and movement is reasonable
            for i, (new, st) in enumerate(zip(new_pts, status)):
                if st == 1:
                    tag_id = list(tags_dict.keys())[i]
                    new_pos = tuple(new.ravel().astype(int))
                    old_pos = tags_dict[tag_id]
                    if np.linalg.norm(np.array(new_pos) - np.array(old_pos)) < MAX_MOVEMENT_THRESHOLD:
                        tags_dict[tag_id] = new_pos
    end_time = time.perf_counter()
    prev_gray = gray.copy()  # Save current frame for next Optical Flow calculation

    # Draw detected & tracked tags on frame
    for tag_id, center in tags_dict.items():
        x, y = center
        cv2.circle(frame, (x, y), 5, (0, 0, 255), -1)
        cv2.putText(frame, f'ID: {tag_id}', (x - 10, y - 10),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 0, 0), 2)
        # Optional: define ROI based on tag center (not used for detection here)
        ROI = [max(0, x - ROI_MARGIN), max(0, y - ROI_MARGIN),
               min(frame.shape[1], x + ROI_MARGIN),
               min(frame.shape[0], y + ROI_MARGIN)]
        # (ROI can be used to crop image for further processing if desired)

    # Display frame (create window once outside loop in a production system)
    cv2.namedWindow("AprilTag Detector", cv2.WINDOW_NORMAL)
    cv2.imshow("AprilTag Detector", frame)
    cv2.resizeWindow("AprilTag Detector", 600, 400)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

    frame_count += 1
    
    print(f"Execution Time: {(end_time - start_time) * 1e3:.2f} ms")

cap.release()
cv2.destroyAllWindows()

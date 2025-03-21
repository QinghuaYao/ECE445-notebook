import cv2
import numpy as np
from pupil_apriltags import Detector
import os
import time 

a = 0.33
b = 0.33
ROI = [None, None, None, None]
found = False

def main():
    cap = cv2.VideoCapture(0)
    os.add_dll_directory(r"E:\Users\Djisa\Anaconda\envs\gym_env39\lib\site-packages\pupil_apriltags.libs")
    at_detector = Detector(families='tag25h9', nthreads=4, quad_decimate=2.0)
    times = []
    # cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1920) # Set width
    # cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 1080) # Set height
    # cap.set(cv2.CAP_PROP_FPS, 60)
    
    while True:
        ret, frame = cap.read()
        if not ret:
            break
        start_time = time.perf_counter()
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        tags = at_detector.detect(gray, estimate_tag_pose=True, camera_params=(600, 600, 320, 240), tag_size=0.082)
        
        end_time = time.perf_counter()
        print(tags)
        cv2.namedWindow("AprilTag Detector",cv2.WINDOW_NORMAL)
        cv2.imshow("AprilTag Detector", frame)
        cv2.resizeWindow("AprilTag Detector", 600, 400)
        for tag in tags:
            # if tag.decision_margin < 50:  # Confidence threshold to avoid false positives
            #     continue
            
            for i in range(4):
                pt1 = tuple(tag.corners[i].astype(int))
                pt2 = tuple(tag.corners[(i+1) % 4].astype(int))
                cv2.line(frame, pt1, pt2, (0, 255, 0), 2)
            
            center = tuple(tag.center.astype(int))
            cv2.circle(frame, center, 5, (0, 0, 255), -1)
            cv2.putText(frame, f'ID: {tag.tag_id}', (center[0]-10, center[1]-10),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 0, 0), 2)
            
            if tag.pose_R is not None and tag.pose_t is not None:
                translation = tag.pose_t.flatten()
                cv2.putText(frame, f'XYZ: {translation[0]:.2f}, {translation[1]:.2f}, {translation[2]:.2f}',
                            (center[0] - 30, center[1] + 20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 2)
        cv2.namedWindow("AprilTag Detector",cv2.WINDOW_NORMAL)
        cv2.imshow("AprilTag Detector", frame)
        cv2.resizeWindow("AprilTag Detector", 600, 400)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
        times.append((end_time - start_time) * 1e6)
        avg_time = np.mean(times[-100:])
        print(f"Average Calculation Time: {avg_time:.2f} us")
        print(f"Execution Time: {(end_time - start_time) * 1e3:.2f} us")
    
    cap.release()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()
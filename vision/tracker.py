from enum import Enum
from ultralytics import YOLO
from camera_interface import Camera
import cv2
from deep_sort_realtime.deepsort_tracker import DeepSort
import numpy as np
import math

from transformers import pipeline
from PIL import Image

import contextlib
import sys
import os

import config


@contextlib.contextmanager
def suppress_output():
    with open(os.devnull, "w") as devnull:
        old_stdout = sys.stdout
        old_stderr = sys.stderr
        sys.stdout = devnull
        sys.stderr = devnull
        try:
            yield
        finally:
            sys.stdout = old_stdout
            sys.stderr = old_stderr



import numpy as np

def screen_to_camera_coords(u, v, z, K):
    """
    Convert screen pixel coordinates and depth to camera-space 3D coordinates.

    Args:
        u, v: pixel coordinates
        z: depth value at (u, v) in same units as K (e.g., meters)
        K: camera intrinsic matrix (3x3)

    Returns:
        3D point (x, y, z) in camera coordinate system
    """
    fx, fy = K[0, 0], K[1, 1]
    cx, cy = K[0, 2], K[1, 2]

    x = (u - cx) * z / fx
    y = (v - cy) * z / fy

    return np.array([x, y, z])



# Define states using Enum
class TrackerState(Enum):
    INIT = 1
    TRACK = 2

class TrackedObj:
    def __init__(self, id: int, x: int, y: int, z: int):
        self.id = id
        self.x = x
        self.y = y
        self.z = z

        self.camx, self.camy, z = screen_to_camera_coords(x, y, z, K)
    
    def update(self, x, y):
        self.x = x
        self.y = y

        self.camx, self.camy, z = screen_to_camera_coords(x, y, self.z, K)


class Tracker:
    def __init__(self, camera: Camera):
        self.camera = camera
        self.width, self.height = camera.get_resolution()

        self.model = YOLO("yolov10n.pt")  # Load model
        # self.model = torch.hub.load('deepcam-cn/yolov5-face', 'yolov5s-face', pretrained=True)
        self.class_names = self.model.names
        
        self.state = TrackerState.INIT  # Initialize with INIT state

        self.tracked_obj = TrackedObj(-1, -1, -1, -1)

        self.tracker = DeepSort()

        self.depth_model = pipeline(task="depth-estimation", model="depth-anything/Depth-Anything-V2-Small-hf")
            

        

    def update(self):
        frame = self.camera.read_frame()

        rgb_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        
        with suppress_output():
            results = self.model(rgb_frame)[0]
        
        
        '''if self.tracked_obj.id != -1:
            roi_frame = frame[int(self.tracked_obj.y1):int(self.tracked_obj.y2), int(self.tracked_obj.x1):int(self.tracked_obj.x2)]
            pil_image = Image.fromarray(roi_frame)
            
            depth = self.depth_model(pil_image)["depth"]

            depth_normalized = cv2.normalize(np.array(depth), None, 0, 255, cv2.NORM_MINMAX)
            depth_normalized = np.uint8(depth_normalized)
            
            # Display the depth map as a grayscale image
            cv2.imshow("Depth Map", depth_normalized)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                return False
        '''

        boxes = results.boxes.xyxy.cpu().numpy()
        scores = results.boxes.conf.cpu().numpy()

        #indices = np.where((scores > 0.7))[0]
        #boxes = boxes[indices] 
        #scores = scores[indices]

        detections = []
        for box, score in zip(boxes, scores):
            x1, y1, x2, y2 = box
            left = x1
            top = y1
            width = x2 - x1
            height = y2 - y1
            detections.append(([left, top, width, height], score, 0))

            # Draw the bounding box
            #cv2.rectangle(frame, (int(x1), int(y1)), (int(x2), int(y2)), (0, 255, 0), 2)  # Green box


        self.tracker.update_tracks(detections, frame=frame)

        if self.state == TrackerState.INIT:
            if len(results.boxes) != 0:
                min_dist = 100000
                min_loc = (-1, -1)
                min_id = -1

                for track in self.tracker.tracker.tracks:
                    if track.is_confirmed():
                        x1, y1, x2, y2 = track.to_tlbr()
                        track_id = track.track_id

                        # Calculate center of the track bounding box
                        center_x = int(x1 + abs(x2 - x1) / 2)
                        center_y = int(y1 + abs(y2 - y1) / 2)

                        # Calculate the center of the frame
                        frame_center_x = self.width / 2
                        frame_center_y = self.height / 2

                        # Compute Euclidean distance from the center of the frame to the center of the bounding box
                        dist = math.sqrt((center_x - frame_center_x) ** 2 + (center_y - frame_center_y) ** 2)

                        # Update minimum distance and ID if this track is closer
                        if dist < min_dist:
                            min_dist = dist
                            min_loc = (center_x, center_y)
                            min_id = track_id
                
                if min_id != -1:
                    self.tracked_obj = TrackedObj(min_id, min_loc[0], min_loc[1], z=config.VisionConfig.depth)

                    self.state = TrackerState.TRACK
                    print("Switching state: INIT -> TRACK")
                    print("With id: ", self.tracked_obj.id)


        if self.state == TrackerState.TRACK:
            for track in self.tracker.tracker.tracks:
                    if track.is_confirmed() and track.time_since_update <= 1:
                        track_id = track.track_id

                        if track_id == self.tracked_obj.id:
                            x1, y1, x2, y2 = track.to_tlbr()
                            
                            # Calculate center of the track bounding box
                            center_x = int(x1 + abs(x2 - x1) / 2)
                            center_y = int(y1 + abs(y2 - y1) / 2)

                            self.tracked_obj.update(center_x, center_y)

                            cv2.circle(frame, (center_x, center_y), radius=5, color=(0, 0, 255), thickness=-1)  # red filled circle

            # Display the frame in a window
            cv2.imshow('Live Feed', frame)

            if cv2.waitKey(1) & 0xFF == ord('q'):
                return False

        return True

    def get_target_rel_pose(self, dt):
        rx, ry, rz = 0, 0, 0
        return [self.tracked_obj.camx, self.tracked_obj.camy, self.tracked_obj.z, rx, ry, rz]

from enum import Enum
from ultralytics import YOLO
from vision.camera_interface import Camera
import cv2
from deep_sort_realtime.deepsort_tracker import DeepSort
import numpy as np
import math

from PIL import Image

import contextlib
import sys
import os

from config import Config

from vision.ball_detect import Detector

#from ultralytics.utils import LOGGER
#LOGGER.setLevel("ERROR")  # or "WARNING" or "CRITICAL"


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
    RETRACK = 3

class TrackedObj:
    def __init__(self, id: int, x: int, y: int, width: int, type, K, config: Config):
        self.ids = [id]
        self.x = x
        self.y = y
        self.width = width
        self.K = np.array(K)

        if type == "tennis_ball":
            self.real_diameter = config.vision.tennis_diameter

        self.z = Detector.estimate_depth_from_bbox(self.width, self.real_diameter, self.K)

        self.camx, self.camy, z = screen_to_camera_coords(self.x, self.y, self.z, self.K)
    
    def update(self, x, y, width):
        self.x = x
        self.y = y
        self.width = width

        self.z = Detector.estimate_depth_from_bbox(self.width, self.real_diameter, self.K)
        self.camx, self.camy, z = screen_to_camera_coords(x, y, self.z, self.K)

        print("----------------------------------")

        print("X is: ", self.camx)

        print("Y is: ", self.camy)

        print("Z is: ", self.z)

        print("----------------------------------")

        print("=-=-=-=-=-=-==-=-=-=-=-=-=-=-=-=-=-=-")
    
    def id_update(self, id):

        if len(self.ids) >= 15:
            self.ids = [id]
        else:
            self.ids.append(id)


class Tracker:
    def __init__(self, camera: Camera, config: Config, tracked_type):
        self.camera = camera
        self.width, self.height = camera.get_resolution()

        #self.model = YOLO("yolov10.pt")  # Load model
        self.model = Detector()


        self.state = TrackerState.INIT  # Initialize with INIT state

        self.tracked_obj = None
        self.tracked_Type = tracked_type

        self.tracker = DeepSort(max_age=1)

        self.config = config

        #self.depth_model = pipeline(task="depth-estimation", model="depth-anything/Depth-Anything-V2-Small-hf")


        self.warmup_frames = 0


        

    def update(self):

        self.warmup_frames += 1

        frame = self.camera.read_frame()

        #rgb_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        
        
        #results = self.model(rgb_frame, conf=0.2, verbose=False)[0]        
        
        results = self.model.detect(frame)


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

        '''boxes = results.boxes.xyxy.cpu().numpy()
        scores = results.boxes.conf.cpu().numpy()
        '''

        #indices = np.where((scores > 0.7))[0]
        #boxes = boxes[indices] 
        #scores = scores[indices]

        detections = []
        for (box, score) in results:
            left, top, width, height = box
            '''left = x1
            top = y1
            width = x2 - x1
            height = y2 - y1'''
            detections.append(([left, top, width, height], score, 0))

            #Draw the bounding box
            x1, y1 = int(left), int(top)
            x2, y2 = int(left + width), int(top + height)
            cv2.rectangle(frame, (int(x1), int(y1)), (int(x2), int(y2)), (0, 255, 0), 2)  # Green box


        self.tracker.update_tracks(detections, frame=frame)

        if self.state == TrackerState.INIT:
            if self.warmup_frames >= 4:

                if len(results) != 0:
                    min_dist = 100000
                    min_loc = (-1, -1)
                    min_id = -1
                    min_width = -1


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
                                min_width = abs(x2 - x1)
                
                    if min_id != -1:

                        self.tracked_obj = TrackedObj(min_id, min_loc[0], min_loc[1], min_width, self.tracked_Type , self.config.vision.K, self.config)

                        self.state = TrackerState.TRACK
                        print("Switching state: INIT -> TRACK")
                        print("With id: ", self.tracked_obj.ids[-1])

            # Display the frame in a window
            cv2.imshow('Live Feed', frame)

            if cv2.waitKey(1) & 0xFF == ord('q'):
                print("falsing")
                return False

            return True
        




        if self.state == TrackerState.TRACK:

            found_track = False

            for track in self.tracker.tracker.tracks:
                    if track.is_confirmed() and track.time_since_update <= 1:
                        track_id = track.track_id

                        x1, y1, x2, y2 = track.to_tlbr()
                        
                        # Calculate center of the track bounding box
                        center_x = int(x1 + abs(x2 - x1) / 2)
                        center_y = int(y1 + abs(y2 - y1) / 2)
                        
                        if track_id == self.tracked_obj.ids[-1]:
                            found_track = True

                            self.tracked_obj.update(center_x, center_y, abs(x2 - x1))

                            cv2.circle(frame, (center_x, center_y), radius=5, color=(0, 0, 255), thickness=-1)  # red filled circle
                        
                        # Draw track ID text
                        text = f"ID: {track_id}"
                        font = cv2.FONT_HERSHEY_SIMPLEX
                        font_scale = 0.6
                        thickness = 2
                        text_size, _ = cv2.getTextSize(text, font, font_scale, thickness)
                        text_x = center_x - text_size[0] // 2
                        text_y = center_y + text_size[1] // 2
                        cv2.putText(frame, text, (text_x, text_y), font, font_scale, (0, 255, 0), thickness, cv2.LINE_AA)


            if not found_track:
                self.state = TrackerState.RETRACK
                print("Switching state: TRACK -> RETRACK")
                print("Lost id: ", self.tracked_obj.ids[-1])

                self.warmup_frames = 0
                
            
            # Display the frame in a window
            cv2.imshow('Live Feed', frame)

            if cv2.waitKey(1) & 0xFF == ord('q'):
                print("falsing")
                return False
                
            return True





        if self.state == TrackerState.RETRACK:
            #print("new frame")
            
            if len(results) != 0:  
                #if self.warmup_frames >= 4:
            
                min_dist = 100000
                min_loc = (-1, -1)
                min_id = -1
                min_width = -1

                for track in self.tracker.tracker.tracks:
                    if track.is_confirmed():
                        x1, y1, x2, y2 = track.to_tlbr()
                        track_id = track.track_id

                        if track_id not in self.tracked_obj.ids:

                            # Calculate center of the track bounding box
                            center_x = int(x1 + abs(x2 - x1) / 2)
                            center_y = int(y1 + abs(y2 - y1) / 2)

                            # Calculate the center of the frame
                            tracked_x = self.tracked_obj.x
                            tracked_y = self.tracked_obj.y

                            # Compute Euclidean distance from the center of the frame to the center of the bounding box
                            dist = math.sqrt((center_x - tracked_x) ** 2 + (center_y - tracked_y) ** 2)

                            # Update minimum distance and ID if this track is closer
                            if dist < min_dist:
                                min_dist = dist
                                min_loc = (center_x, center_y)
                                min_id = track_id
                                min_width = abs(x2 - x1)
                
                if min_id != -1:
                    self.tracked_obj.update(min_loc[0], min_loc[1], min_width)
                    self.tracked_obj.id_update(min_id)
                    cv2.circle(frame, (min_loc[0], min_loc[1]), radius=5, color=(0, 255, 0), thickness=-1)  # red filled circle


                    self.state = TrackerState.TRACK
                    print("Switching state: RETRACK -> TRACK")
                    print("With id: ", self.tracked_obj.ids[-1])

            
            else:

                '''obj_x = self.tracked_obj.x
                obj_y = self.tracked_obj.y

                frame_center_x = self.width / 2
                frame_center_y = self.height / 2
                # Step size (0 < alpha <= 1) — how much to move towards target
                alpha = 0.1  # 0.1 means move 10% toward the frame center

                # Compute direction vector
                dx = frame_center_x - obj_x
                dy = frame_center_y - obj_y

                # Move point towards frame center
                x_new = obj_x + alpha * dx
                y_new = obj_y + alpha * dy

                # Optionally cast to int if working with pixels
                x_new = int(x_new)
                y_new = int(y_new)

                self.tracked_obj.update(x_new, y_new, self.tracked_obj.width)'''

                desired_pos = np.array(self.config.robot.default_desired_rel_pos) - np.array(self.config.robot.cam_rel_offset)

                self.tracked_obj.update(desired_pos[0], desired_pos[1], desired_pos[2])


            # Display the frame in a window
            cv2.imshow('Live Feed', frame)

            if cv2.waitKey(1) & 0xFF == ord('q'):
                return False
            
            return True
            

    def get_target_rel_pose(self, dt):
        rx, ry, rz = 0, 0, 0
        return [self.tracked_obj.camy, self.tracked_obj.camx, self.tracked_obj.z, rx, ry, rz]

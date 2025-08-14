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




def Point_In(rect, x, y):
    if rect is None:
        return False
    x1, y1, x2, y2 = rect
    return (x1 <= x <= x2) and (y1 <= y <= y2)


def Compute_Button_Rects(frame):
    """Return (manual_rect, auto_rect) as (x1,y1,x2,y2), sized by frame."""
    h, w = frame.shape[:2]
    btn_w = int(0.28 * w)
    btn_h = int(0.12 * h)
    gap   = int(0.04 * w)
    y1    = int(0.08 * h)

    cx = w // 2
    # manual on the left of center, automatic on the right
    man_x1 = cx - gap//2 - btn_w
    man_y1 = y1
    man_x2 = man_x1 + btn_w
    man_y2 = man_y1 + btn_h

    auto_x1 = cx + gap//2
    auto_y1 = y1
    auto_x2 = auto_x1 + btn_w
    auto_y2 = auto_y1 + btn_h
    return (man_x1, man_y1, man_x2, man_y2), (auto_x1, auto_y1, auto_x2, auto_y2)



# Define states using Enum
class TrackerState(Enum):
    INIT_CHOOSE = 1
    INIT_MANUAL = 2
    INIT_AUTOMATIC = 3
    TRACK = 4
    RETRACK = 5

class TrackedObj:
    def __init__(self, K, config, type, id=None, x=None, y=None, width=None, height=None):
        self.K = np.array(K)
        self.config = config

        if type == "tennis_ball":
                self.real_diameter = self.config.vision.tennis_diameter


        if id == None:
            self.ids = []

            self.x = None
            self.y = None
            self.width = None
            self.height = None
            
            desired_pos = np.array(self.config.robot.default_desired_rel_pos) - np.array(self.config.robot.cam_rel_offset)
            self.z = desired_pos[2]
            self.camx = desired_pos[0]
            self.camy = desired_pos[1]
            

        else:
            self.ids = [id]
            self.x = x
            self.y = y
            self.width = width
            self.height = height

            self.z = Detector.estimate_depth_from_bbox(max(self.width, self.height), self.real_diameter, self.K)

            self.camx, self.camy, z = screen_to_camera_coords(self.x, self.y, self.z, self.K)

        
        
            
        '''print("----------------------------------")

        print("X is: ", self.camx)

        print("Y is: ", self.camy)

        print("Z is: ", self.z)

        print("----------------------------------")

        print("=-=-=-=-=-=-==-=-=-=-=-=-=-=-=-=-=-=-")'''


    
    def update(self, x, y, width, height):
        self.x = x
        self.y = y
        self.width = width
        self.height = height

        self.z = Detector.estimate_depth_from_bbox(max(self.width, self.height), self.real_diameter, self.K)
        self.camx, self.camy, z = screen_to_camera_coords(x, y, self.z, self.K)

        '''print("----------------------------------")

        print("X is: ", self.camx)

        print("Y is: ", self.camy)

        print("Z is: ", self.z)

        print("----------------------------------")

        print("=-=-=-=-=-=-==-=-=-=-=-=-=-=-=-=-=-=-")'''
    
    
    
    def id_update(self, id):

        if len(self.ids) >= 15:
            self.ids = [id]
        else:
            self.ids.append(id)

            

    def default(self):
        desired_pos = np.array(self.config.robot.default_desired_rel_pos) - np.array(self.config.robot.cam_rel_offset)

        self.camx = desired_pos[0]
        self.camy = desired_pos[1]
        self.z = desired_pos[2]

        
        
        '''print("----------------------------------")

        print("X is: ", self.camx)

        print("Y is: ", self.camy)

        print("Z is: ", self.z)

        print("----------------------------------")

        print("=-=-=-=-=-=-==-=-=-=-=-=-=-=-=-=-=-=-")'''




class Tracker:

    def __init__(self, camera: Camera, config: Config, tracked_type):
        self.camera = camera
        self.width, self.height = camera.get_resolution()

        #self.model = YOLO("yolov10.pt")  # Load model
        self.model = Detector()


        self.state = TrackerState.INIT_CHOOSE  # Initialize with INIT state

        self.tracked_obj = TrackedObj(config.vision.K, config, tracked_type)


        self.tracked_Type = tracked_type

        self.tracker = DeepSort(max_age=1)

        self.config = config

        #self.depth_model = pipeline(task="depth-estimation", model="depth-anything/Depth-Anything-V2-Small-hf")


        self.warmup_frames = 0

        self.window_name = "Live Feed"

        cv2.namedWindow(self.window_name, cv2.WINDOW_NORMAL)  # ensure the window exists
        cv2.setMouseCallback(self.window_name, self._on_mouse)  # set callback once
        self._manual_rect = None
        self._auto_rect = None

        self.reset_rect = (10, 10, 150, 50)

        self.track_rects = []


    def _on_mouse(self, event, x, y, flags, param=None):
        # Only react in INIT_CHOOSE and only if click is inside a button
        if event == cv2.EVENT_LBUTTONDOWN and self.state == TrackerState.INIT_CHOOSE:
            if Point_In(self._manual_rect, x, y):
                print("Switching state: INIT_CHOOSE -> INIT_MANUAL")
                self.state = TrackerState.INIT_MANUAL
                self._manual_rect = None
                self._auto_rect   = None
            elif Point_In(self._auto_rect, x, y):
                print("Switching state: INIT_CHOOSE -> INIT_AUTOMATIC")
                self.state = TrackerState.INIT_AUTOMATIC
                self._manual_rect = None
                self._auto_rect   = None 

        elif event == cv2.EVENT_LBUTTONDOWN and self.state == TrackerState.INIT_MANUAL:
              found_rect = False
              for rect, id in self.track_rects:
                  if not found_rect:
                      if Point_In(rect, x, y):
                        x1, y1, x2, y2 = rect

                        # Calculate center of the track bounding box
                        center_x = int(x1 + abs(x2 - x1) / 2)
                        center_y = int(y1 + abs(y2 - y1) / 2)

                        self.tracked_obj = TrackedObj(self.config.vision.K, self.config, self.tracked_Type, id, center_x, center_y, abs(x2 - x1), abs(y2 -y1))

                        print("Switching state: INIT_MANUAL -> TRACK")
                        print("With id: ", self.tracked_obj.ids[-1])
                        self.state = TrackerState.TRACK

                        found_rect = True

        
        elif event == cv2.EVENT_LBUTTONDOWN and (self.state == TrackerState.TRACK or self.state == TrackerState.RETRACK):
              if Point_In(self.reset_rect, x, y):

                self.tracked_obj.default()
                
                if self.state == TrackerState.TRACK:
                    print("Switching state: TRACK -> INIT_CHOOSE")
                else:
                    print("Switching state: RETRACK -> INIT_CHOOSE")

                self.state = TrackerState.INIT_CHOOSE
                          

    
    
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
            #cv2.rectangle(frame, (int(x1), int(y1)), (int(x2), int(y2)), (0, 255, 0), 2)  # Green box


        self.tracker.update_tracks(detections, frame=frame)

         # ---- INIT_CHOOSE logic ----
        if self.state == TrackerState.INIT_CHOOSE:
            # compute & store rects for this frame size
            self._manual_rect, self._auto_rect = Compute_Button_Rects(frame)

            # draw Manual box
            mx1, my1, mx2, my2 = self._manual_rect
            cv2.rectangle(frame, (mx1, my1), (mx2, my2), (255, 0, 0), 2)
            cv2.putText(frame, "Manual", (mx1 + 20, my1 +  int(0.65*(my2-my1))),
                        cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 0, 0), 2)

            # draw Automatic box
            ax1, ay1, ax2, ay2 = self._auto_rect
            cv2.rectangle(frame, (ax1, ay1), (ax2, ay2), (0, 0, 255), 2)
            cv2.putText(frame, "Automatic", (ax1 + 20, ay1 + int(0.65*(ay2-ay1))),
                        cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)


            cv2.imshow(self.window_name, frame)
            cv2.waitKey(1)
            return True

            

        
        if self.state == TrackerState.INIT_MANUAL:
            if self.warmup_frames >= 4:
                self.track_rects = []

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

                        self.track_rects.append(((x1, y1, x2, y2), track_id))


                        cv2.rectangle(frame, (int(x1), int(y1)), (int(x2), int(y2)), (0, 255, 0), 2)  # Green box
           

            cv2.putText(
                frame,
                "Choose object to track",
                (20, 40),  # Position (x, y)
                cv2.FONT_HERSHEY_SIMPLEX,  # Font
                1.0,  # Font scale
                (0, 255, 255),  # Color (Yellow)
                2,  # Thickness
                cv2.LINE_AA
            )
            
            cv2.imshow(self.window_name, frame)
            cv2.waitKey(1)
            return True

        
        
        
        if self.state == TrackerState.INIT_AUTOMATIC:
            if self.warmup_frames >= 4:

                if len(results) != 0:
                    min_dist = 100000
                    min_loc = (-1, -1)
                    min_id = -1
                    min_width = -1
                    min_height = -1


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
                                min_height = abs(y2 - y1)
                
                    if min_id != -1:

                        self.tracked_obj = TrackedObj(self.config.vision.K, self.config, self.tracked_Type, min_id, min_loc[0], min_loc[1], min_width, min_height)

                        self.state = TrackerState.TRACK
                        print("Switching state: INIT_AUTOMATIC -> TRACK")
                        print("With id: ", self.tracked_obj.ids[-1])

            # Display the frame in a window
            cv2.imshow('Live Feed', frame)

            if cv2.waitKey(1) & 0xFF == ord('q'):
                print("falsing")
                return False

            return True
        

        if self.state == TrackerState.TRACK or self.state == TrackerState.RETRACK:
            # Draw the red "return" button at the top of the frame
            button_x1, button_y1 = 10, 10
            button_x2, button_y2 = 150, 50

            
            cv2.rectangle(frame, (button_x1, button_y1), (button_x2, button_y2), (0, 0, 255), 3)  # Red outline
            cv2.putText(frame, "return", (button_x1 + 20, button_y1 + 28),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 255), 2, cv2.LINE_AA)


        
        
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

                            self.tracked_obj.update(center_x, center_y, abs(x2 - x1), abs(y2 - y1))


                            cv2.rectangle(frame, (int(x1), int(y1)), (int(x2), int(y2)), (0, 255, 0), 2)  # Green box
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
                min_height = -1

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
                                min_height = abs(y2 - y1)
                
                if min_id != -1:
                    self.tracked_obj.update(min_loc[0], min_loc[1], min_width, min_height)
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

                

                self.tracked_obj.default()


            # Display the frame in a window
            cv2.imshow('Live Feed', frame)

            if cv2.waitKey(1) & 0xFF == ord('q'):
                return False
            
            return True
            

    def get_target_rel_pose(self, dt):
        rx, ry, rz = 0, 0, 0
        return [self.tracked_obj.camy, self.tracked_obj.camx, self.tracked_obj.z, rx, ry, rz]

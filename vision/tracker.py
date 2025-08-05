from enum import Enum
from ultralytics import YOLO
from vision.camera_interface import Camera
import cv2
from deep_sort_realtime.deepsort_tracker import DeepSort
import numpy as np
import math



# Define states using Enum
class TrackerState(Enum):
    INIT = 1
    TRACK = 2

class Object:
    def __init__(self, id: int, x: int, y: int):
        self.id = id
        self.x = x
        self.y = y

class Tracker:
    def __init__(self, camera: Camera):
        self.camera = camera
        self.width, self.height = camera.get_resolution()
        self.curr_pos = (int(self.width / 2), int(self.height / 2))

        self.model = YOLO("yolov10n.pt")  # Load model
        # self.model = torch.hub.load('deepcam-cn/yolov5-face', 'yolov5s-face', pretrained=True)
        self.class_names = self.model.names
        
        self.state = TrackerState.INIT  # Initialize with INIT state

        self.tracked_obj = Object(-1, -1, -1)

        self.tracker = DeepSort()

        

    def update(self):
        frame = self.camera.read_frame()

        rgb_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        results = self.model(rgb_frame)[0]

        boxes = results.boxes.xyxy.cpu().numpy()
        scores = results.boxes.conf.cpu().numpy()

        indices = np.where((scores > 0.7))[0]
        boxes = boxes[indices]
        scores = scores[indices]

        detections = []
        for box, score in zip(boxes, scores):
            x1, y1, x2, y2 = box
            left = x1
            top = y1
            width = x2 - x1
            height = y2 - y1
            detections.append(([left, top, width, height], score, 0))

        self.tracker.update_tracks(detections, frame=frame)

        if self.state == TrackerState.INIT:
            if len(results.boxes) != 0:
                min_dist = 100000
                min_loc = (-1, -1)
                min_id = -1

                for track in self.tracker.tracker.tracks:
                    if track.is_confirmed() and track.time_since_update <= 1:
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
                    self.tracked_obj = Object(min_id, min_loc[0], min_loc[1])

                    self.state = TrackerState.TRACK
                    print("Switching state: INIT -> TRACK")
                    print("With id: ", self.tracked_obj.id)


        if self.state == TrackerState.TRACK:
            for track in self.tracker.tracker.tracks:
                    if track.is_confirmed() and track.time_since_update <= 1:
                        x1, y1, x2, y2 = track.to_tlbr()
                        track_id = track.track_id

                        if track_id == self.tracked_obj.id:
                            self.tracked_obj.x = int(x1 + abs(x2 - x1) / 2)
                            self.tracked_obj.y = int(y1 + abs(y2 - y1) / 2)

   
            cv2.circle(frame, (self.tracked_obj.x, self.tracked_obj.y), radius=5, color=(0, 0, 255), thickness=-1)  # red filled circle
            
            # Add the track_id text slightly above the circle
            font = cv2.FONT_HERSHEY_SIMPLEX
            text = str(self.tracked_obj.id)
            text_size = cv2.getTextSize(text, font, 0.6, 1)[0]
            text_x = self.tracked_obj.x - text_size[0] // 2  # Center text horizontally with the circle
            text_y = self.tracked_obj.y - 10  # Position the text above the circle

            # Draw the track_id text above the red circle
            cv2.putText(frame, text, (text_x, text_y), font, 0.6, (0, 0, 255), 1, cv2.LINE_AA)


            # Display the frame in a window
            cv2.imshow('Live Feed', frame)

            if cv2.waitKey(1) & 0xFF == ord('q'):
                return False

            # After the first tracking update, change state to TRACK
            self.state = TrackerState.TRACK

        elif self.state == TrackerState.TRACK:
            # Track state logic (you can add additional behavior here if needed)
            # You could continue processing, display more information, or perform actions based on the tracked person.
            print("Tracking target:", self.curr_pos)

        return True

    def get_target_rel_pose(self, dt):
        x, y = self.curr_pos
        z = 0.5
        rx, ry, rz = 0, 0, 0
        return [x, y, z, rx, ry, rz]

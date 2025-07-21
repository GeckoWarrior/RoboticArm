from ultralytics import YOLO
from camera_interface import Camera
#import torch
import cv2


class Tracker:
    def __init__(self, camera: Camera):
        self.camera = camera
        self.width, self.height = camera.get_resolution()
        self.curr_pos = (int(self.width / 2), int(self.height / 2))

        self.model = YOLO("yolov10n.pt")

        # Load model
        #self.model = torch.hub.load('deepcam-cn/yolov5-face', 'yolov5s-face', pretrained=True)

        self.class_names = self.model.names

    def update(self):
         # Read a frame from the camera
        frame = self.camera.read_frame()

        rgb_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        results = self.model(rgb_frame)[0]

        # Draw only 'cup' boxes
        for box in results.boxes:
            cls_id = int(box.cls)
            if self.class_names[cls_id] == "person":
                x1, y1, x2, y2 = map(int, box.xyxy[0])

                pos_relto_camera = (int(x1 + abs(x2 - x1)/2), int(y1 + abs(y2 - y1)/2))

                self.curr_pos = (pos_relto_camera[0] - self.width, pos_relto_camera[1] - self.height)
                break


        cv2.circle(frame, self.curr_pos, radius=5, color=(0, 0, 255), thickness=-1)  # red filled circle
        # Display the frame in a window

        #resized = cv2.resize(frame, (640, 480))
        cv2.imshow('Live Feed', frame)

        if cv2.waitKey(1) & 0xFF == ord('q'):
            return False
        
        return True



    def get_curr_position(self):
        return self.curr_pos
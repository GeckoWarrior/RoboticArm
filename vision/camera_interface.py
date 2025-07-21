import cv2


class Camera:
    def __init__(self, input_camera=0, width=1280, height=720):
        self.frame = None
        self.cap = cv2.VideoCapture(input_camera)

        self.width = self.cap.get(cv2.CAP_PROP_FRAME_WIDTH)
        self.height = self.cap.get(cv2.CAP_PROP_FRAME_HEIGHT)

        '''self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)'''


    def update(self):
        ret, frame = self.cap.read()

        if not ret:
            print("Error: Failed to read frame.")
            return
        
        self.frame = frame


    def read_frame(self):
        return self.frame
    
    def get_resolution(self):
        return (self.width, self.height)
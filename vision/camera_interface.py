import cv2


class Camera:
    def __init__(self, camera_mode="laptop"):
        self.camera_mode = camera_mode
        self.frame = None

    
    def setup(self):
        camera_list = []
        for index in range(10):
            cap = cv2.VideoCapture(index)
            if cap.read()[0]:
                camera_list.append(index)
                cap.release()
            else:
                cap.release()

        if len(camera_list) != 2 and self.camera_mode != "laptop":
            raise RuntimeError("Error, incorrect number of cameras connected")

       
        if self.camera_mode == "laptop":
            self.cap = cv2.VideoCapture(0)
        else:
            self.cap = cv2.VideoCapture(camera_list[1])

        self.width = self.cap.get(cv2.CAP_PROP_FRAME_WIDTH)
        self.height = self.cap.get(cv2.CAP_PROP_FRAME_HEIGHT)
        


        




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
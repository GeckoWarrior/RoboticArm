import cv2
import numpy as np




class Detector:
    def __init__(self):
        pass
    


    def detect(self, frame):

        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        # HSV range for tennis ball yellow-green
        lower = np.array([25, 50, 50])
        upper = np.array([45, 255, 255])

        mask = cv2.inRange(hsv, lower, upper)

        # Blur mask to reduce noise
        mask = cv2.GaussianBlur(mask, (7,7), 0)

        # Find contours
        contours, _ = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

        bboxes = []  # list of bounding boxes for detected balls

        for cnt in contours:
            area = cv2.contourArea(cnt)
            if area > 500:  # tune area threshold
                (x_c, y_c), radius = cv2.minEnclosingCircle(cnt)
                if radius > 10:
                    # Get the smallest rectangle around the contour
                    x, y, w, h = cv2.boundingRect(cnt)
                    bboxes.append(([x, y, w, h], 1))


        print(bboxes)  # Each box is [x, y, w, h]
       
        return bboxes
    
    def estimate_depth_from_bbox(width, real_diameter_m, K):
        """
        Estimate object depth from bounding box size.

        Args:
            bbox: (left, top, width, height)
            real_diameter_m: real-world diameter of the ball in meters
            K: camera intrinsic matrix (3x3)

        Returns:
            Estimated depth (Z) in meters
        """
        f = K[0, 0]      # focal length in pixels (fx from intrinsics)
        Z = (f * real_diameter_m) / width #width is the diameter
        return Z


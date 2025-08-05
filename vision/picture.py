import cv2
import time

def capture_image_from_camera():
    # Open the camera (0 = default camera)

    camera_list = []
    for index in range(10):
        cap = cv2.VideoCapture(index)
        if cap.read()[0]:
            camera_list.append(index)
            cap.release()
        else:
            cap.release()

    cap = cv2.VideoCapture(camera_list[1])
    
    if not cap.isOpened():
        print("Error: Could not open camera.")
        return
    
    print("Press SPACE to capture an image, or ESC to exit.")
    
    while True:
        ret, frame = cap.read()
        if not ret:
            print("Failed to grab frame.")
            break
        
        # Show the live camera feed
        cv2.imshow("Camera", frame)
        
        # Wait for a key press
        key = cv2.waitKey(1)
        
        if key == 27:  # ESC to exit
            print("Exiting without saving.")
            break
        elif key == 32:  # SPACE to capture
            save_path = f"calibration_imgs/{time.time()}.jpg"
            cv2.imwrite(save_path, frame)
            print(f"Image saved to {save_path}")
            
    
    cap.release()
    cv2.destroyAllWindows()

# Run the function
capture_image_from_camera()

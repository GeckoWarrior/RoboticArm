import cv2

def capture_image_from_camera(camera_index=0, save_path="calibrate_imgs/01.jpg"):
    # Open the camera (0 = default camera)
    cap = cv2.VideoCapture(camera_index)
    
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
            cv2.imwrite(save_path, frame)
            print(f"Image saved to {save_path}")
            break
    
    cap.release()
    cv2.destroyAllWindows()

# Run the function
capture_image_from_camera()

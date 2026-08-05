import cv2
import numpy as np
import cv2.aruco as aruco
import math

def get_marker_color(marker_id):
    """Assigns a unique BGR color based on the marker ID."""
    # OpenCV uses BGR (Blue, Green, Red) instead of RGB
    colors = {
        0: (0, 255, 0),    # ID 0: Green (Robot 1)
        1: (255, 0, 0),    # ID 1: Blue  (Robot 2)
        2: (0, 0, 255),    # ID 2: Red   (Robot 3)
        3: (0, 255, 255)   # ID 3: Yellow (Robot 4)
    }
    # Default to white if ID is not in our specific list
    return colors.get(marker_id, (255, 255, 255))

def main():
    # 1. Setup the ArUco Dictionary and Detector (Modern OpenCV 4.x syntax)
    aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_4X4_50)
    parameters = aruco.DetectorParameters()
    detector = aruco.ArucoDetector(aruco_dict, parameters)

    # 2. Start Video Capture (1 is usually the built-in webcam)
    cap = cv2.VideoCapture(1, cv2.CAP_DSHOW)

    # Tell the camera to use MJPEG compression first
    cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))
    
    # Optional: Set camera resolution to 1080p for better detection
    #cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1920)
    #cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 1080)

    # Set the name of our window
    window_name = "Centralized Brain Vision"

    # 3. Create a resizable window
    cv2.namedWindow(window_name, cv2.WINDOW_NORMAL)
    
    # 4. Force the window into true fullscreen mode
    cv2.setWindowProperty(window_name, cv2.WND_PROP_FULLSCREEN, cv2.WINDOW_FULLSCREEN)

    print("Starting camera... Press 'q' to quit.")

    while True:
        ret, frame = cap.read()
        if not ret:
            print("Failed to grab frame")
            break

        # Convert to grayscale (ArUco detection requires grayscale)
        gray_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        # Detect the markers
        corners, ids, rejected = detector.detectMarkers(gray_frame)

        # If markers are found, process and draw them
        if ids is not None:
            # Flatten the array to make it version-proof
            flat_ids = ids.flatten()
            
            for i in range(len(flat_ids)):
                marker_id = flat_ids[i]
                marker_corners = corners[i][0] # Get the 4 corners of this marker
                
                # Fetch the assigned color for this ID
                color = get_marker_color(marker_id)

                # Draw the bounding box polygon
                pts = np.int32(marker_corners).reshape(-1, 1, 2)
                cv2.polylines(frame, [pts], isClosed=True, color=color, thickness=3)

                # Calculate the exact center of the marker
                center_x = int(np.mean(marker_corners[:, 0]))
                center_y = int(np.mean(marker_corners[:, 1]))

                # --- CALCULATE HEADING (THETA) ---
                # Corner 0 is Top-Left, Corner 1 is Top-Right
                top_center_x = (marker_corners[0][0] + marker_corners[1][0]) / 2.0
                top_center_y = (marker_corners[0][1] + marker_corners[1][1]) / 2.0

                # Calculate differences (Inverting Y because image Y goes down, but math Y goes up)
                dx = top_center_x - center_x
                dy = center_y - top_center_y 

                # math.atan2 returns angle in radians, convert to degrees
                theta_rad = math.atan2(dy, dx)
                theta_deg = (math.degrees(theta_rad) + 360) % 360 # Keeps angle between 0 and 360

                # --- VISUALIZE HEADING ---
                # Draw a line from the center pointing towards the front of the robot
                cv2.line(frame, (center_x, center_y), (int(top_center_x), int(top_center_y)), (0, 0, 255), 3)

                # Draw the ID text in the center
                cv2.putText(frame, f"ID: {marker_id}", (center_x - 20, center_y), 
                            cv2.FONT_HERSHEY_SIMPLEX, 0.7, color, 2, cv2.LINE_AA)

                # Output to terminal: The complete dataset for the C++ Brain
                print(f"Robot {marker_id} -> X: {center_x}, Y: {center_y}, Heading: {int(theta_deg)}°")

        # Display the live feed in our true fullscreen window
        cv2.imshow(window_name, frame)

        # Break loop on 'q' press
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    cap.release()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()
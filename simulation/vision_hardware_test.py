from numpy.lib import _function_base_impl
from numpy.lib import _function_base_impl
from numpy.lib import _function_base_impl
from numpy.lib import _function_base_impl
import cv2
import numpy as np
import cv2.aruco as aruco

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

    print("Starting camera... Press 'q' to quit.")

    while True:
        ret, frame = cap.read()
        if not ret:
            print("Failed to grab frame")
            break

        # Convert to grayscale (ArUco detection requires grayscale)
        gray_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        # 3. Detect the markers
        corners, ids, rejected = detector.detectMarkers(gray_frame)

        # 4. If markers are found, process and draw them
        # 4. If markers are found, process and draw them
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

                # Calculate the center of the marker to place the text
                center_x = int(np.mean(marker_corners[:, 0]))
                center_y = int(np.mean(marker_corners[:, 1]))

                # Draw the ID text in the center
                cv2.putText(frame, f"ID: {marker_id}", (center_x - 20, center_y), 
                            cv2.FONT_HERSHEY_SIMPLEX, 0.7, color, 2, cv2.LINE_AA)

                # Output to terminal to verify the data (What the C++ Brain will eventually get)
                print(f"Detected Robot {marker_id} at Pixel X:{center_x} Y:{center_y}")

        # Display the live feed
        cv2.imshow("Warehouse Vision Tracker", frame)

        # Break loop on 'q' press
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    cap.release()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()
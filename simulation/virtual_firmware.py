import cv2
import numpy as np
import cv2.aruco as aruco
import math
import socket # <-- Network library

def get_marker_info(marker_id):
    """Assigns a group color and custom label based on the marker ID."""
    if 0 <= marker_id <= 2:
        return (0, 255, 0), f"Robot {marker_id + 1}"
    elif 3 <= marker_id <= 5:
        return (255, 0, 0), f"Pallet {marker_id - 2}"
    elif 6 <= marker_id <= 8:
        return (0, 0, 255), f"Dest {marker_id - 5}"
    else:
        return (255, 255, 255), f"Unknown ({marker_id})"

def main():
    # Setup ArUco
    aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_4X4_50)
    parameters = aruco.DetectorParameters()
    detector = aruco.ArucoDetector(aruco_dict, parameters)

    # Setup Camera
    cap = cv2.VideoCapture(1, cv2.CAP_DSHOW)
    cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))

    # Setup Window
    window_name = "Centralized Brain Vision"
    cv2.namedWindow(window_name, cv2.WINDOW_NORMAL)
    cv2.setWindowProperty(window_name, cv2.WND_PROP_FULLSCREEN, cv2.WINDOW_FULLSCREEN)
    is_fullscreen = True

    # --- SETUP UDP NETWORK ---
    UDP_IP = "127.0.0.1"
    UDP_PORT = 5005
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    print(f"Network Active: Broadcasting to {UDP_IP}:{UDP_PORT}")
    # -------------------------

    print("Starting camera... Press 'q' to quit, 'f' to toggle fullscreen.")

    while True:
        ret, frame = cap.read()
        if not ret:
            print("Failed to grab frame")
            break

        gray_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        corners, ids, rejected = detector.detectMarkers(gray_frame)

        if ids is not None:
            flat_ids = ids.flatten()
            
            # 1. Create empty list for this specific frame
            frame_data = [] 
            
            for i in range(len(flat_ids)):
                marker_id = flat_ids[i]
                marker_corners = corners[i][0]
                color, label_text = get_marker_info(marker_id)

                pts = np.int32(marker_corners).reshape(-1, 1, 2)
                cv2.polylines(frame, [pts], isClosed=True, color=color, thickness=3)

                center_x = int(np.mean(marker_corners[:, 0]))
                center_y = int(np.mean(marker_corners[:, 1]))

                top_center_x = (marker_corners[0][0] + marker_corners[1][0]) / 2.0
                top_center_y = (marker_corners[0][1] + marker_corners[1][1]) / 2.0
                dx = top_center_x - center_x
                dy = center_y - top_center_y 
                theta_rad = math.atan2(dy, dx)
                theta_deg = (math.degrees(theta_rad) + 360) % 360 

                cv2.line(frame, (center_x, center_y), (int(top_center_x), int(top_center_y)), (0, 0, 255), 3)
                cv2.putText(frame, label_text, (center_x - 30, center_y), 
                            cv2.FONT_HERSHEY_SIMPLEX, 0.7, color, 2, cv2.LINE_AA)

                # Output to Python terminal
                print(f"{label_text} -> X: {center_x}, Y: {center_y}, Heading: {int(theta_deg)}°")

                # 2. Add this marker's raw math to the network list
                frame_data.append(f"{marker_id},{center_x},{center_y},{int(theta_deg)}")

            # 3. If we have data, broadcast it over UDP!
            if len(frame_data) > 0:
                packet = ";".join(frame_data)
                sock.sendto(packet.encode('utf-8'), (UDP_IP, UDP_PORT))

        cv2.imshow(window_name, frame)

        key = cv2.waitKey(1) & 0xFF
        if key == ord('q'):
            break
        elif key == ord('f'):
            is_fullscreen = not is_fullscreen
            if is_fullscreen:
                cv2.setWindowProperty(window_name, cv2.WND_PROP_FULLSCREEN, cv2.WINDOW_FULLSCREEN)
            else:
                cv2.setWindowProperty(window_name, cv2.WND_PROP_FULLSCREEN, cv2.WINDOW_NORMAL)

    cap.release()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()
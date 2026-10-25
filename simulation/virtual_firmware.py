import cv2
import numpy as np
import cv2.aruco as aruco
import math
import socket
import time

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

def draw_dashed_line(img, pt1, pt2, color, thickness=3, dash_len=14, gap_len=8):
    """Draws an anti-aliased dashed line between pt1 and pt2."""
    dist = math.hypot(pt2[0] - pt1[0], pt2[1] - pt1[1])
    if dist < 1.0:
        return
    dx = (pt2[0] - pt1[0]) / dist
    dy = (pt2[1] - pt1[1]) / dist

    curr = 0.0
    while curr < dist:
        sx = int(pt1[0] + dx * curr)
        sy = int(pt1[1] + dy * curr)
        end_dist = min(curr + dash_len, dist)
        ex = int(pt1[0] + dx * end_dist)
        ey = int(pt1[1] + dy * end_dist)
        cv2.line(img, (sx, sy), (ex, ey), color, thickness, cv2.LINE_AA)
        curr += dash_len + gap_len

def draw_ar_paths(frame, paths_dict, show_paths=True):
    """
    Renders semi-transparent AR trajectory paths from the C++ Planner:
      - Yellow dashed line when approaching pallet (DRIVE->PALLET)
      - Green dashed line when delivering to destination (DRIVE->DEST)
    """
    if not show_paths or not paths_dict:
        return frame

    now = time.time()
    overlay = frame.copy()
    has_drawn = False

    for rid, pdata in list(paths_dict.items()):
        # Expire stale paths (older than 2.0s)
        if now - pdata.get('timestamp', 0) > 2.0:
            paths_dict.pop(rid, None)
            continue

        state = pdata.get('state', '')
        pts = pdata.get('points', [])
        if len(pts) < 2:
            continue

        # Color mapping based on state:
        # Yellow for approach to pallet (BGR: 0, 230, 255)
        # Emerald Green for delivery to destination (BGR: 50, 255, 50)
        if "PALLET" in state:
            color = (0, 230, 255)          # Bright Yellow
            badge_text = f"R{rid + 1} -> Pallet"
        elif "DEST" in state:
            color = (50, 255, 50)          # Emerald Green
            badge_text = f"R{rid + 1} -> Dest"
        elif "PICK" in state:
            color = (255, 200, 0)          # Cyan
            badge_text = f"R{rid + 1} [PICKING]"
        elif "DROP" in state:
            color = (200, 50, 255)         # Magenta
            badge_text = f"R{rid + 1} [DROPPING]"
        else:
            color = (200, 200, 200)        # Default Gray
            badge_text = f"R{rid + 1} {state}"

        has_drawn = True

        # 1. Draw dashed line connecting waypoints: Robot -> WP1 -> WP2 ... -> Target
        for i in range(len(pts) - 1):
            draw_dashed_line(overlay, pts[i], pts[i + 1], color, thickness=3, dash_len=14, gap_len=8)

        # 2. Draw waypoint circles
        for i, pt in enumerate(pts):
            if i == 0:
                continue  # Start point is the robot center
            elif i == len(pts) - 1:
                # Final destination target halo
                cv2.circle(overlay, pt, 9, color, -1, cv2.LINE_AA)
                cv2.circle(overlay, pt, 14, (255, 255, 255), 2, cv2.LINE_AA)
            else:
                # Intermediate turning waypoint
                cv2.circle(overlay, pt, 5, color, -1, cv2.LINE_AA)
                cv2.circle(overlay, pt, 7, (255, 255, 255), 1, cv2.LINE_AA)

        # 3. Label badge near the start of the path
        start_pt = pts[0]
        cv2.putText(overlay, badge_text, (start_pt[0] + 15, start_pt[1] - 15),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.55, color, 2, cv2.LINE_AA)

    if has_drawn:
        # Alpha-blend overlay (60% path + 40% camera frame) for semi-transparent holographic effect
        cv2.addWeighted(overlay, 0.60, frame, 0.40, 0, frame)

    return frame

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
    show_paths = True

    # --- SETUP UDP NETWORKS ---
    # TX: Broadcast camera vision telemetry to C++ Planner
    UDP_TX_IP = "127.0.0.1"
    UDP_TX_PORT = 5005
    tx_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

    # RX: Receive planned AR paths from C++ Planner
    UDP_RX_PORT = 5006
    rx_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    rx_sock.bind(("0.0.0.0", UDP_RX_PORT))
    rx_sock.setblocking(False)

    print("==================================================")
    print("  CENTRALIZED BRAIN VISION + AR VISUALIZER")
    print("==================================================")
    print(f"  [TX] Broadcasting vision to UDP {UDP_TX_IP}:{UDP_TX_PORT}")
    print(f"  [RX] Listening for AR paths on UDP :{UDP_RX_PORT}")
    print("  Controls:")
    print("    'q' : Quit")
    print("    'f' : Toggle Fullscreen")
    print("    'p' : Toggle AR Path Lines (ON/OFF)")
    print("==================================================")

    active_paths = {}

    while True:
        ret, frame = cap.read()
        if not ret:
            print("Failed to grab frame")
            break

        # 1. Drain UDP buffer to fetch latest planned paths from C++ Planner
        while True:
            try:
                data, _ = rx_sock.recvfrom(2048)
                msg = data.decode('utf-8', errors='ignore')
                parts = msg.split(';')
                if parts[0] == "CLEAR" and len(parts) >= 2:
                    rid = int(parts[1])
                    active_paths.pop(rid, None)
                elif parts[0] == "PATH" and len(parts) >= 4:
                    rid = int(parts[1])
                    state_str = parts[2]
                    pts = []
                    for coord_str in parts[3:]:
                        if ',' in coord_str:
                            x_s, y_s = coord_str.split(',')
                            pts.append((int(x_s), int(y_s)))
                    if pts:
                        active_paths[rid] = {
                            'state': state_str,
                            'points': pts,
                            'timestamp': time.time()
                        }
            except BlockingIOError:
                break
            except Exception:
                break

        # 2. Detect ArUco Markers
        gray_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        corners, ids, rejected = detector.detectMarkers(gray_frame)

        if ids is not None:
            flat_ids = ids.flatten()
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

                frame_data.append(f"{marker_id},{center_x},{center_y},{int(theta_deg)}")

            # Broadcast vision telemetry to C++ Planner
            if len(frame_data) > 0:
                packet = ";".join(frame_data)
                tx_sock.sendto(packet.encode('utf-8'), (UDP_TX_IP, UDP_TX_PORT))

        # 3. Render AR planned trajectories over the frame
        frame = draw_ar_paths(frame, active_paths, show_paths=show_paths)

        # 4. HUD Banner
        hud_status = "ON" if show_paths else "OFF"
        hud_color = (0, 255, 0) if show_paths else (0, 0, 255)
        cv2.putText(frame, f"AR Paths: [{hud_status}] (P to toggle)", (25, 40),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.65, hud_color, 2, cv2.LINE_AA)

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
        elif key == ord('p'):
            show_paths = not show_paths

    cap.release()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()

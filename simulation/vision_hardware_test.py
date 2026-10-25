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

def draw_ar_paths(frame, paths_dict, live_markers, show_paths=True):
    """
    Renders rock-solid semi-transparent AR trajectory paths from the C++ Planner:
      - Yellow dashed line when approaching pallet (DRIVE->PALLET)
      - Green dashed line when delivering to destination (DRIVE->DEST)
    Endpoints are directly pinned to detected live marker centers so paths NEVER drift.
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
        target_id = pdata.get('target_id', -1)
        raw_pts = pdata.get('points', [])
        if len(raw_pts) < 2:
            continue

        # Pin start point to the live robot center if detected
        pts = list(raw_pts)
        if rid in live_markers:
            pts[0] = live_markers[rid]

        # Pin end point to live target center if detected
        if target_id in live_markers:
            pts[-1] = live_markers[target_id]
        elif "PALLET" in state or "PICK" in state:
            # Fallback: check pallets 3, 4, 5
            for pid in [3, 4, 5]:
                if pid in live_markers and math.hypot(live_markers[pid][0] - pts[-1][0], live_markers[pid][1] - pts[-1][1]) < 150:
                    pts[-1] = live_markers[pid]
                    break
        elif "DEST" in state:
            # Fallback: check destinations 6, 7, 8
            for did in [6, 7, 8]:
                if did in live_markers and math.hypot(live_markers[did][0] - pts[-1][0], live_markers[did][1] - pts[-1][1]) < 150:
                    pts[-1] = live_markers[did]
                    break

        if "PALLET" in state:
            color = (0, 230, 255)          # Bright Yellow
            badge_text = f"R{rid + 1} -> Pallet {target_id - 2 if target_id >= 3 else ''}"
        elif "PICK" in state:
            color = (0, 230, 255)          # Bright Yellow
            badge_text = f"R{rid + 1} [AT PALLET - Press 'L' when lifted]"
        elif "DEST" in state:
            color = (50, 255, 50)          # Emerald Green
            badge_text = f"R{rid + 1} -> Dest {target_id - 5 if target_id >= 6 else ''}"
        elif "DROP" in state:
            color = (200, 50, 255)         # Magenta
            badge_text = f"R{rid + 1} [DROPPING]"
        else:
            color = (200, 200, 200)        # Default Gray
            badge_text = f"R{rid + 1} {state}"

        has_drawn = True

        for i in range(len(pts) - 1):
            draw_dashed_line(overlay, pts[i], pts[i + 1], color, thickness=3, dash_len=14, gap_len=8)

        for i, pt in enumerate(pts):
            if i == 0:
                continue
            elif i == len(pts) - 1:
                cv2.circle(overlay, pt, 9, color, -1, cv2.LINE_AA)
                cv2.circle(overlay, pt, 14, (255, 255, 255), 2, cv2.LINE_AA)
            else:
                cv2.circle(overlay, pt, 5, color, -1, cv2.LINE_AA)
                cv2.circle(overlay, pt, 7, (255, 255, 255), 1, cv2.LINE_AA)

        start_pt = pts[0]
        cv2.putText(overlay, badge_text, (start_pt[0] + 15, start_pt[1] - 15),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.55, color, 2, cv2.LINE_AA)

    if has_drawn:
        cv2.addWeighted(overlay, 0.60, frame, 0.40, 0, frame)

    return frame

def main():
    aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_4X4_50)
    parameters = aruco.DetectorParameters()
    detector = aruco.ArucoDetector(aruco_dict, parameters)

    cap = cv2.VideoCapture(1, cv2.CAP_DSHOW)
    cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))

    window_name = "Centralized Brain Vision"
    cv2.namedWindow(window_name, cv2.WINDOW_NORMAL)
    cv2.setWindowProperty(window_name, cv2.WND_PROP_FULLSCREEN, cv2.WINDOW_FULLSCREEN)
    is_fullscreen = True
    show_paths = True

    UDP_TX_IP = "127.0.0.1"
    UDP_TX_PORT = 5005
    tx_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

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
    print("    'l' : Signal Pallet LIFTED (Shows Destination Path!)")
    print("    '1', '2', '3' : Signal LIFTED for Robot 1, 2, or 3")
    print("    'c' : Recalibrate 2cm Marker Scale")
    print("==================================================")

    MARKER_PHYSICAL_SIZE_M = 0.02
    calibration_samples = []
    CALIBRATION_FRAMES_NEEDED = 25
    smoothed_ppm = 1200.0
    ppm_calibrated = False

    active_paths = {}

    while True:
        ret, frame = cap.read()
        if not ret:
            print("Failed to grab frame")
            break

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
                    target_id = -1
                    coord_idx = 3
                    if ',' not in parts[3]:
                        try:
                            target_id = int(parts[3])
                            coord_idx = 4
                        except ValueError:
                            target_id = -1

                    pts = []
                    for coord_str in parts[coord_idx:]:
                        if ',' in coord_str:
                            x_s, y_s = coord_str.split(',')
                            pts.append((int(x_s), int(y_s)))
                    if pts:
                        active_paths[rid] = {
                            'state': state_str,
                            'target_id': target_id,
                            'points': pts,
                            'timestamp': time.time()
                        }
            except BlockingIOError:
                break
            except Exception:
                break

        gray_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        corners, ids, rejected = detector.detectMarkers(gray_frame)

        live_markers = {}

        if ids is not None:
            flat_ids = ids.flatten()
            frame_data = []
            measured_ppms = []

            for i in range(len(flat_ids)):
                marker_id = flat_ids[i]
                marker_corners = corners[i][0]
                color, label_text = get_marker_info(marker_id)

                pts = np.int32(marker_corners).reshape(-1, 1, 2)
                cv2.polylines(frame, [pts], isClosed=True, color=color, thickness=3)

                center_x = int(np.mean(marker_corners[:, 0]))
                center_y = int(np.mean(marker_corners[:, 1]))
                live_markers[marker_id] = (center_x, center_y)

                top_center_x = (marker_corners[0][0] + marker_corners[1][0]) / 2.0
                top_center_y = (marker_corners[0][1] + marker_corners[1][1]) / 2.0
                dx = top_center_x - center_x
                dy = center_y - top_center_y
                theta_rad = math.atan2(dy, dx)
                theta_deg = (math.degrees(theta_rad) + 360) % 360

                cv2.line(frame, (center_x, center_y), (int(top_center_x), int(top_center_y)), (0, 0, 255), 3)
                cv2.putText(frame, label_text, (center_x - 30, center_y),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.7, color, 2, cv2.LINE_AA)

                if not ppm_calibrated:
                    side_t = math.hypot(marker_corners[1][0] - marker_corners[0][0], marker_corners[1][1] - marker_corners[0][1])
                    side_r = math.hypot(marker_corners[2][0] - marker_corners[1][0], marker_corners[2][1] - marker_corners[1][1])
                    side_b = math.hypot(marker_corners[3][0] - marker_corners[2][0], marker_corners[3][1] - marker_corners[2][1])
                    side_l = math.hypot(marker_corners[0][0] - marker_corners[3][0], marker_corners[0][1] - marker_corners[3][1])
                    marker_px = (side_t + side_r + side_b + side_l) / 4.0
                    if marker_px > 5.0:
                        measured_ppms.append(marker_px / MARKER_PHYSICAL_SIZE_M)

                frame_data.append(f"{marker_id},{center_x},{center_y},{int(theta_deg)}")

            if not ppm_calibrated and measured_ppms:
                calibration_samples.append(float(np.median(measured_ppms)))
                if len(calibration_samples) >= CALIBRATION_FRAMES_NEEDED:
                    smoothed_ppm = float(round(np.median(calibration_samples)))
                    ppm_calibrated = True
                    print(f"[CALIBRATION LOCKED] Scale: {int(smoothed_ppm)} px/m (2cm marker = {smoothed_ppm*MARKER_PHYSICAL_SIZE_M:.1f} px)")
                    tx_sock.sendto(f"PPM:{int(smoothed_ppm)}".encode('utf-8'), (UDP_TX_IP, UDP_TX_PORT))

            if len(frame_data) > 0:
                packet = f"PPM:{int(smoothed_ppm)};" + ";".join(frame_data)
                tx_sock.sendto(packet.encode('utf-8'), (UDP_TX_IP, UDP_TX_PORT))

        frame = draw_ar_paths(frame, active_paths, live_markers, show_paths=show_paths)

        hud_status = "ON" if show_paths else "OFF"
        hud_color = (0, 255, 0) if show_paths else (0, 0, 255)
        cv2.putText(frame, f"AR Paths: [{hud_status}] (P: toggle | L: Lifted | C: Calibrate)", (25, 40),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.60, hud_color, 2, cv2.LINE_AA)

        calib_str = "LOCKED" if ppm_calibrated else f"Calibrating ({len(calibration_samples)}/{CALIBRATION_FRAMES_NEEDED})"
        marker_px_disp = smoothed_ppm * MARKER_PHYSICAL_SIZE_M
        cv2.putText(frame, f"Scale: {int(smoothed_ppm)} px/m [{calib_str}] (2cm = {marker_px_disp:.1f}px)",
                    (25, 70), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 255, 255), 2, cv2.LINE_AA)

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
        elif key in [ord('l'), ord('L')]:
            tx_sock.sendto(b"LIFTED;ALL", (UDP_TX_IP, UDP_TX_PORT))
            print("[LIFT SIGNAL] Broadcasted 'LIFTED;ALL' to Planner! (Destination paths will now show)")
        elif key in [ord('1'), ord('2'), ord('3')]:
            rid = key - ord('1')
            tx_sock.sendto(f"LIFTED;{rid}".encode('utf-8'), (UDP_TX_IP, UDP_TX_PORT))
            print(f"[LIFT SIGNAL] Broadcasted 'LIFTED;{rid}' for Robot {rid+1} to Planner!")
        elif key in [ord('c'), ord('C')]:
            calibration_samples = []
            ppm_calibrated = False
            print("[CALIBRATION] Reset! Collecting fresh samples from 2cm markers...")

    cap.release()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()

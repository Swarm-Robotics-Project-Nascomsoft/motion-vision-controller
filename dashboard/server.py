"""
Dashboard Server — Manager Control Panel for Centralized Warehouse Planner
===========================================================================

Standard Flask web server (zero extra pip packages required):
  - Live MJPEG camera feed with AR trajectory paths rendered on top
  - Real-time Server-Sent Events (SSE) + REST polling for status
  - Full system control: manual task assignment, deactivation, e-stop,
    pause/resume, reset, and pallet lifted signals
  - Interactive component coordinate inspector and activity log

Network Ports:
  - Planner Vision & Command RX : UDP 127.0.0.1:5005
  - AR Viz Trajectory RX        : UDP 127.0.0.1:5006 (from planner)
  - Planner Status RX           : UDP 127.0.0.1:5007 (from planner)
  - Dashboard Web Server        : HTTP 127.0.0.1:8080

Usage:
  python dashboard/server.py
  Open http://localhost:8080 in your browser.
"""

import cv2
import cv2.aruco as aruco
import numpy as np
import math
import socket
import time
import json
import threading
from datetime import datetime
from collections import deque
from flask import Flask, render_template, Response, jsonify, request

# ============================================================================
#  CONFIGURATION
# ============================================================================

CAMERA_INDEX = 1                    # Camera device index (DirectShow)
PLANNER_IP   = "127.0.0.1"
PLANNER_PORT = 5005                 # Send commands & vision to planner
STATUS_PORT  = 5007                 # Receive status from planner
VIZ_PORT     = 5006                 # Receive AR viz paths from planner
WEB_PORT     = 8080                 # Dashboard web server port

MARKER_PHYSICAL_SIZE_M = 0.02      # 2cm x 2cm physical markers
CALIBRATION_FRAMES_NEEDED = 25

# ============================================================================
#  SHARED STATE
# ============================================================================

class SharedState:
    def __init__(self):
        self.frame_jpeg = None
        self.frame_lock = threading.Lock()

        self.live_markers = {}          # {marker_id: (cx, cy, theta_deg)}
        self.marker_lock = threading.Lock()

        self.planner_status = {}
        self.status_lock = threading.Lock()

        self.active_paths = {}          # {rid: {state, target_id, points, timestamp}}
        self.paths_lock = threading.Lock()

        self.activity_log = deque(maxlen=200)
        self.log_lock = threading.Lock()

        self.ppm = 1200.0
        self.ppm_calibrated = False

        self.camera_connected = False
        self.planner_connected = False
        self.last_planner_status_time = 0

    def add_log(self, level, message):
        with self.log_lock:
            entry = {
                'time': datetime.now().strftime('%H:%M:%S'),
                'level': level,
                'message': message
            }
            self.activity_log.appendleft(entry)
            return entry

    def get_logs(self, count=50):
        with self.log_lock:
            return list(self.activity_log)[:count]


state = SharedState()

# UDP socket for sending commands to planner
cmd_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

def send_to_planner(message):
    """Send a command string to the C++ planner via UDP port 5005."""
    try:
        cmd_sock.sendto(message.encode('utf-8'), (PLANNER_IP, PLANNER_PORT))
        return True
    except Exception as e:
        state.add_log('ERROR', f'Failed to send to planner: {e}')
        return False


# ============================================================================
#  AR PATH DRAWING UTILITIES
# ============================================================================

def get_marker_info(marker_id):
    if 0 <= marker_id <= 2:
        return (0, 255, 0), f"Robot {marker_id + 1}"
    elif 3 <= marker_id <= 5:
        return (255, 0, 0), f"Pallet {marker_id - 2}"
    elif 6 <= marker_id <= 8:
        return (0, 0, 255), f"Dest {marker_id - 5}"
    else:
        return (255, 255, 255), f"Unknown ({marker_id})"

def draw_dashed_line(img, pt1, pt2, color, thickness=3, dash_len=14, gap_len=8):
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

def draw_ar_paths(frame, paths_dict, live_markers):
    if not paths_dict:
        return frame

    now = time.time()
    overlay = frame.copy()
    has_drawn = False

    for rid, pdata in list(paths_dict.items()):
        if now - pdata.get('timestamp', 0) > 2.0:
            paths_dict.pop(rid, None)
            continue

        state_str = pdata.get('state', '')
        target_id = pdata.get('target_id', -1)
        raw_pts = pdata.get('points', [])
        if len(raw_pts) < 2:
            continue

        pts = list(raw_pts)

        # Pin start point to the live robot center if detected
        if rid in live_markers:
            pts[0] = (live_markers[rid][0], live_markers[rid][1])

        # Pin end point to live target center if detected
        if target_id in live_markers:
            pts[-1] = (live_markers[target_id][0], live_markers[target_id][1])
        elif "PALLET" in state_str or "PICK" in state_str:
            for pid in [3, 4, 5]:
                if pid in live_markers and math.hypot(live_markers[pid][0] - pts[-1][0], live_markers[pid][1] - pts[-1][1]) < 150:
                    pts[-1] = (live_markers[pid][0], live_markers[pid][1])
                    break
        elif "DEST" in state_str:
            for did in [6, 7, 8]:
                if did in live_markers and math.hypot(live_markers[did][0] - pts[-1][0], live_markers[did][1] - pts[-1][1]) < 150:
                    pts[-1] = (live_markers[did][0], live_markers[did][1])
                    break

        if "PALLET" in state_str:
            color = (0, 230, 255)          # Bright Yellow
            badge_text = f"R{rid + 1} -> Pallet {target_id - 2 if target_id >= 3 else ''}"
        elif "PICK" in state_str:
            color = (0, 230, 255)          # Bright Yellow
            badge_text = f"R{rid + 1} [AT PALLET]"
        elif "DEST" in state_str:
            color = (50, 255, 50)          # Emerald Green
            badge_text = f"R{rid + 1} -> Dest {target_id - 5 if target_id >= 6 else ''}"
        elif "DROP" in state_str:
            color = (200, 50, 255)         # Magenta
            badge_text = f"R{rid + 1} [DROPPING]"
        else:
            color = (200, 200, 200)
            badge_text = f"R{rid + 1} {state_str}"

        has_drawn = True

        # Draw dashed line
        for i in range(len(pts) - 1):
            draw_dashed_line(overlay, pts[i], pts[i + 1], color, thickness=3, dash_len=14, gap_len=8)

        # Draw waypoint circles
        for i, pt in enumerate(pts):
            if i == 0:
                continue
            elif i == len(pts) - 1:
                cv2.circle(overlay, pt, 9, color, -1, cv2.LINE_AA)
                cv2.circle(overlay, pt, 14, (255, 255, 255), 2, cv2.LINE_AA)
            else:
                cv2.circle(overlay, pt, 5, color, -1, cv2.LINE_AA)
                cv2.circle(overlay, pt, 7, (255, 255, 255), 1, cv2.LINE_AA)

        # Label badge
        start_pt = pts[0]
        cv2.putText(overlay, badge_text, (start_pt[0] + 15, start_pt[1] - 15),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.55, color, 2, cv2.LINE_AA)

    if has_drawn:
        cv2.addWeighted(overlay, 0.60, frame, 0.40, 0, frame)

    return frame


# ============================================================================
#  CAMERA & VISION TELEMETRY THREAD
# ============================================================================

def camera_thread_func():
    """Captures camera frames, detects ArUco markers, renders AR paths,
    and sends vision telemetry to the planner."""

    aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_4X4_50)
    parameters = aruco.DetectorParameters()
    detector = aruco.ArucoDetector(aruco_dict, parameters)

    # Try camera index
    cap = cv2.VideoCapture(CAMERA_INDEX, cv2.CAP_DSHOW)
    cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))

    if not cap.isOpened():
        print(f"[CAM] Warning: Camera {CAMERA_INDEX} not available, trying camera 0...")
        cap = cv2.VideoCapture(0, cv2.CAP_DSHOW)

    if not cap.isOpened():
        state.add_log('WARN', 'Physical camera not opened. Running in telemetry-only mode.')
        state.camera_connected = False
        # Create dynamic placeholder frame
        while True:
            blank = np.zeros((480, 640, 3), dtype=np.uint8)
            cv2.putText(blank, "NO CAMERA ATTACHED", (170, 220),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 255), 2, cv2.LINE_AA)
            cv2.putText(blank, "Dashboard controls & telemetry active", (150, 260),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.55, (200, 200, 200), 1, cv2.LINE_AA)
            _, jpeg = cv2.imencode('.jpg', blank)
            with state.frame_lock:
                state.frame_jpeg = jpeg.tobytes()
            time.sleep(0.5)

    state.camera_connected = True
    state.add_log('INFO', f'Overhead camera opened successfully')

    vision_tx = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

    smoothed_ppm = state.ppm
    calibration_samples = []
    ppm_calibrated = state.ppm_calibrated

    while True:
        ret, frame = cap.read()
        if not ret:
            state.camera_connected = False
            time.sleep(0.1)
            continue

        state.camera_connected = True

        # Detect ArUco markers
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        corners, ids, _ = detector.detectMarkers(gray)

        markers_this_frame = {}
        frame_data = []
        measured_ppms = []

        if ids is not None:
            flat_ids = ids.flatten()

            for i in range(len(flat_ids)):
                marker_id = flat_ids[i]
                mc = corners[i][0]
                color, label = get_marker_info(marker_id)

                pts = np.int32(mc).reshape(-1, 1, 2)
                cv2.polylines(frame, [pts], isClosed=True, color=color, thickness=3)

                cx = int(np.mean(mc[:, 0]))
                cy = int(np.mean(mc[:, 1]))

                top_cx = (mc[0][0] + mc[1][0]) / 2.0
                top_cy = (mc[0][1] + mc[1][1]) / 2.0
                dx = top_cx - cx
                dy = cy - top_cy
                theta_deg = (math.degrees(math.atan2(dy, dx)) + 360) % 360

                markers_this_frame[int(marker_id)] = (cx, cy, theta_deg)

                cv2.line(frame, (cx, cy), (int(top_cx), int(top_cy)), (0, 0, 255), 3)
                cv2.putText(frame, label, (cx - 30, cy),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.7, color, 2, cv2.LINE_AA)

                # PPM auto-calibration
                if not ppm_calibrated:
                    sides = [
                        math.hypot(mc[1][0]-mc[0][0], mc[1][1]-mc[0][1]),
                        math.hypot(mc[2][0]-mc[1][0], mc[2][1]-mc[1][1]),
                        math.hypot(mc[3][0]-mc[2][0], mc[3][1]-mc[2][1]),
                        math.hypot(mc[0][0]-mc[3][0], mc[0][1]-mc[3][1])
                    ]
                    marker_px = sum(sides) / 4.0
                    if marker_px > 5.0:
                        measured_ppms.append(marker_px / MARKER_PHYSICAL_SIZE_M)

                frame_data.append(f"{marker_id},{cx},{cy},{int(theta_deg)}")

            if not ppm_calibrated and measured_ppms:
                calibration_samples.append(float(np.median(measured_ppms)))
                if len(calibration_samples) >= CALIBRATION_FRAMES_NEEDED:
                    smoothed_ppm = float(round(np.median(calibration_samples)))
                    ppm_calibrated = True
                    state.ppm = smoothed_ppm
                    state.ppm_calibrated = True
                    state.add_log('INFO', f'Scale LOCKED: {int(smoothed_ppm)} px/m (2cm = {smoothed_ppm*MARKER_PHYSICAL_SIZE_M:.1f}px)')
                    vision_tx.sendto(f"PPM:{int(smoothed_ppm)}".encode(), (PLANNER_IP, PLANNER_PORT))

            # Send vision telemetry packet to planner
            if frame_data:
                packet = f"PPM:{int(smoothed_ppm)};" + ";".join(frame_data)
                vision_tx.sendto(packet.encode(), (PLANNER_IP, PLANNER_PORT))

        with state.marker_lock:
            state.live_markers = markers_this_frame

        # Overlay AR paths from C++ planner
        with state.paths_lock:
            paths_copy = dict(state.active_paths)
        frame = draw_ar_paths(frame, paths_copy, markers_this_frame)

        # HUD banner
        cv2.putText(frame, "CENTRALIZED BRAIN DASHBOARD FEED", (20, 30),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 255, 0), 2, cv2.LINE_AA)
        calib_str = "LOCKED" if ppm_calibrated else f"CALIBRATING ({len(calibration_samples)}/{CALIBRATION_FRAMES_NEEDED})"
        cv2.putText(frame, f"Scale: {int(smoothed_ppm)} px/m [{calib_str}]", (20, 55),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.50, (0, 255, 255), 2, cv2.LINE_AA)

        # Encode JPEG for MJPEG stream
        _, jpeg = cv2.imencode('.jpg', frame, [cv2.IMWRITE_JPEG_QUALITY, 75])
        with state.frame_lock:
            state.frame_jpeg = jpeg.tobytes()

        time.sleep(0.033)

    cap.release()


# ============================================================================
#  AR VIZ PATH RECEIVER THREAD (UDP 5006)
# ============================================================================

def viz_listener_func():
    """Listens on UDP 5006 for AR path updates from planner."""
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind(("0.0.0.0", VIZ_PORT))
    sock.setblocking(False)

    while True:
        try:
            data, _ = sock.recvfrom(2048)
            msg = data.decode('utf-8', errors='ignore')
            parts = msg.split(';')

            with state.paths_lock:
                if parts[0] == "CLEAR" and len(parts) >= 2:
                    rid = int(parts[1])
                    state.active_paths.pop(rid, None)
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
                        state.active_paths[rid] = {
                            'state': state_str,
                            'target_id': target_id,
                            'points': pts,
                            'timestamp': time.time()
                        }
        except BlockingIOError:
            time.sleep(0.01)
        except Exception:
            time.sleep(0.01)


# ============================================================================
#  PLANNER STATUS LISTENER THREAD (UDP 5007)
# ============================================================================

def status_listener_func():
    """Listens on UDP 5007 for JSON status packets from C++ planner."""
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind(("0.0.0.0", STATUS_PORT))
    sock.settimeout(1.0)

    while True:
        try:
            data, _ = sock.recvfrom(8192)
            msg = data.decode('utf-8', errors='ignore')
            try:
                status = json.loads(msg)
                with state.status_lock:
                    state.planner_status = status
                    state.last_planner_status_time = time.time()
                    state.planner_connected = True
            except json.JSONDecodeError:
                pass
        except socket.timeout:
            if time.time() - state.last_planner_status_time > 4.0:
                state.planner_connected = False
        except Exception:
            pass


# ============================================================================
#  STATUS POLLER THREAD
# ============================================================================

def status_poller_func():
    """Sends 'STATUS' request to C++ planner every 400ms."""
    while True:
        send_to_planner("STATUS")
        time.sleep(0.4)


# ============================================================================
#  FLASK WEB APP
# ============================================================================

app = Flask(__name__,
            template_folder='templates',
            static_folder='static')
app.config['SECRET_KEY'] = 'warehouse-planner-dashboard-2026'

@app.route('/')
def index():
    return render_template('index.html')

@app.route('/video_feed')
def video_feed():
    def generate():
        while True:
            with state.frame_lock:
                frame = state.frame_jpeg
            if frame is not None:
                yield (b'--frame\r\n'
                       b'Content-Type: image/jpeg\r\n\r\n' + frame + b'\r\n')
            time.sleep(0.033)

    return Response(generate(), mimetype='multipart/x-mixed-replace; boundary=frame')

@app.route('/api/status')
def api_status():
    with state.status_lock:
        planner_data = dict(state.planner_status) if state.planner_status else {}

    with state.marker_lock:
        markers = {str(k): {'px': v[0], 'py': v[1], 'heading': round(v[2], 1)}
                   for k, v in state.live_markers.items()}

    return jsonify({
        'planner': planner_data,
        'markers': markers,
        'camera_connected': state.camera_connected,
        'planner_connected': state.planner_connected,
        'ppm': state.ppm,
        'ppm_calibrated': state.ppm_calibrated
    })

@app.route('/api/stream')
def api_stream():
    """Server-Sent Events (SSE) stream for real-time status and logs."""
    def event_stream():
        last_log_count = 0
        while True:
            with state.status_lock:
                planner_data = dict(state.planner_status) if state.planner_status else {}

            with state.marker_lock:
                markers = {str(k): {'px': v[0], 'py': v[1], 'heading': round(v[2], 1)}
                           for k, v in state.live_markers.items()}

            payload = {
                'planner': planner_data,
                'markers': markers,
                'camera_connected': state.camera_connected,
                'planner_connected': state.planner_connected,
                'ppm': state.ppm,
                'ppm_calibrated': state.ppm_calibrated,
                'timestamp': time.time()
            }

            yield f"data: {json.dumps(payload)}\n\n"
            time.sleep(0.4)

    return Response(event_stream(), mimetype='text/event-stream')

@app.route('/api/logs')
def api_logs():
    return jsonify(state.get_logs(100))

# ---- Action Endpoints ----

@app.route('/api/assign', methods=['POST'])
def api_assign():
    data = request.json or {}
    robot_id = data.get('robot_id')
    pallet_id = data.get('pallet_id')
    dest_id = data.get('dest_id')

    if robot_id is None or pallet_id is None or dest_id is None:
        return jsonify({'error': 'Missing robot_id, pallet_id, or dest_id'}), 400

    cmd = f"ASSIGN;{robot_id};{pallet_id};{dest_id}"
    success = send_to_planner(cmd)
    entry = state.add_log('CMD', f'Assigned Robot {robot_id+1} -> Pallet {pallet_id-2} -> Dest {dest_id-5}')
    return jsonify({'success': success, 'command': cmd, 'log': entry})

@app.route('/api/estop', methods=['POST'])
def api_estop():
    success = send_to_planner("ESTOP")
    entry = state.add_log('CRITICAL', 'EMERGENCY STOP ACTIVATED')
    return jsonify({'success': success, 'log': entry})

@app.route('/api/pause', methods=['POST'])
def api_pause():
    success = send_to_planner("PAUSE")
    entry = state.add_log('CMD', 'System PAUSED')
    return jsonify({'success': success, 'log': entry})

@app.route('/api/resume', methods=['POST'])
def api_resume():
    success = send_to_planner("RESUME")
    entry = state.add_log('CMD', 'System RESUMED')
    return jsonify({'success': success, 'log': entry})

@app.route('/api/reset', methods=['POST'])
def api_reset():
    success = send_to_planner("RESET")
    entry = state.add_log('CMD', 'System RESET')
    return jsonify({'success': success, 'log': entry})

@app.route('/api/lifted', methods=['POST'])
def api_lifted():
    data = request.json or {}
    target = data.get('target', 'ALL')

    if target == 'ALL':
        cmd = "LIFTED;ALL"
        entry = state.add_log('CMD', 'Pallet LIFTED signal sent for ALL robots')
    else:
        cmd = f"LIFTED;{target}"
        entry = state.add_log('CMD', f'Pallet LIFTED signal sent for Robot {int(target)+1}')

    success = send_to_planner(cmd)
    return jsonify({'success': success, 'log': entry})

@app.route('/api/deactivate', methods=['POST'])
def api_deactivate():
    data = request.json or {}
    entity_id = data.get('entity_id')
    if entity_id is None:
        return jsonify({'error': 'Missing entity_id'}), 400

    cmd = f"DEACTIVATE;{entity_id}"
    success = send_to_planner(cmd)

    if entity_id <= 2:
        name = f"Robot {entity_id + 1}"
    elif entity_id <= 5:
        name = f"Pallet {entity_id - 2}"
    else:
        name = f"Dest {entity_id - 5}"

    entry = state.add_log('CMD', f'Deactivated {name} (Marker ID {entity_id})')
    return jsonify({'success': success, 'log': entry})

@app.route('/api/activate', methods=['POST'])
def api_activate():
    data = request.json or {}
    entity_id = data.get('entity_id')
    if entity_id is None:
        return jsonify({'error': 'Missing entity_id'}), 400

    cmd = f"ACTIVATE;{entity_id}"
    success = send_to_planner(cmd)

    if entity_id <= 2:
        name = f"Robot {entity_id + 1}"
    elif entity_id <= 5:
        name = f"Pallet {entity_id - 2}"
    else:
        name = f"Dest {entity_id - 5}"

    entry = state.add_log('CMD', f'Activated {name} (Marker ID {entity_id})')
    return jsonify({'success': success, 'log': entry})


# ============================================================================
#  MAIN ENTRY POINT
# ============================================================================

def main():
    print("=" * 60)
    print("  WAREHOUSE MANAGER DASHBOARD SERVER")
    print("=" * 60)
    print(f"  Web Dashboard  : http://localhost:{WEB_PORT}")
    print(f"  Video Feed     : http://localhost:{WEB_PORT}/video_feed")
    print(f"  Planner Target : UDP {PLANNER_IP}:{PLANNER_PORT}")
    print(f"  AR Viz Listen  : UDP :{VIZ_PORT}")
    print(f"  Status Listen  : UDP :{STATUS_PORT}")
    print("=" * 60)
    print()

    state.add_log('INFO', 'Dashboard server starting...')

    # Background threads
    t_cam = threading.Thread(target=camera_thread_func, daemon=True)
    t_cam.start()

    t_viz = threading.Thread(target=viz_listener_func, daemon=True)
    t_viz.start()

    t_status = threading.Thread(target=status_listener_func, daemon=True)
    t_status.start()

    t_poller = threading.Thread(target=status_poller_func, daemon=True)
    t_poller.start()

    state.add_log('INFO', 'All background threads initialized')

    # Run standard Flask server (multi-threaded, no extra dependencies)
    app.run(host='0.0.0.0', port=WEB_PORT, debug=False, threaded=True)


if __name__ == '__main__':
    main()

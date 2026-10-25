"""
sim_vision_feeder.py — Synthetic Vision Feeder for Planner Integration Testing
================================================================================

Sends fake vision packets to the C++ planner on UDP 5005, simulating an overhead
camera that sees one robot, one pallet, and one destination.

Usage:
  1. Start the planner:    .\planner.exe
  2. Start this feeder:    python sim_vision_feeder.py
  3. Watch the planner console for state transitions, velocity commands, and AR paths.
"""

import socket
import struct
import time
import threading
import math

# ---- Config ----
PLANNER_IP   = "127.0.0.1"
VISION_PORT  = 5005          # Planner listens here
COMMAND_PORT = 5000          # Planner sends commands here
VIZ_PORT     = 5006          # Planner sends AR path waypoints here

# ---- Simulated entities (pixel coordinates, matching 400 px/m calibration) ----
# Robot 0:       bottom-left of arena,  heading right (0 deg)
# Pallet 3:      centre of arena
# Destination 6: top-right of arena

ROBOT_PX   = (100, 700, 0)     # x_px, y_px, theta_deg  (≈ 0.25m, 0.25m, 0°)
PALLET_PX  = (400, 400, 0)     # ≈ centre (1.0m, 1.0m)
DEST_PX    = (700, 100, 0)     # ≈ top-right (1.75m, 1.75m)


def command_listener():
    """Background thread: listen for velocity commands from the planner."""
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind(("0.0.0.0", COMMAND_PORT))
    sock.settimeout(1.0)
    print(f"[CMD LISTENER] Listening on UDP :{COMMAND_PORT}")

    while True:
        try:
            data, _ = sock.recvfrom(1024)
            msg = data.decode("utf-8", errors="replace")
            print(f"  << CMD: {msg}")
        except socket.timeout:
            pass
        except Exception as e:
            print(f"[CMD LISTENER] Error: {e}")
            break

def viz_listener():
    """Background thread: listen for AR path waypoints from the planner."""
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind(("0.0.0.0", VIZ_PORT))
    sock.settimeout(1.0)
    print(f"[VIZ LISTENER] Listening on UDP :{VIZ_PORT}")

    while True:
        try:
            data, _ = sock.recvfrom(2048)
            msg = data.decode("utf-8", errors="replace")
            print(f"  << VIZ AR PATH: {msg}")
        except socket.timeout:
            pass
        except Exception as e:
            print(f"[VIZ LISTENER] Error: {e}")
            break

def main():
    # Start command listener in background
    t1 = threading.Thread(target=command_listener, daemon=True)
    t1.start()

    t2 = threading.Thread(target=viz_listener, daemon=True)
    t2.start()

    # Vision sender socket
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

    print(f"[FEEDER] Sending vision packets to {PLANNER_IP}:{VISION_PORT}")
    print(f"[FEEDER] Simulating:  Robot 0 @ {ROBOT_PX}")
    print(f"                      Pallet 3 @ {PALLET_PX}")
    print(f"                      Dest 6   @ {DEST_PX}")
    print()

    cycle = 0
    while True:
        # Build packet:  "ID,X,Y,Theta;ID,X,Y,Theta;..."
        parts = []
        parts.append(f"0,{ROBOT_PX[0]},{ROBOT_PX[1]},{ROBOT_PX[2]}")
        parts.append(f"3,{PALLET_PX[0]},{PALLET_PX[1]},{PALLET_PX[2]}")
        parts.append(f"6,{DEST_PX[0]},{DEST_PX[1]},{DEST_PX[2]}")
        packet = ";".join(parts)

        sock.sendto(packet.encode("utf-8"), (PLANNER_IP, VISION_PORT))

        if cycle % 40 == 0:
            print(f"[FEEDER] Sent frame {cycle}:  {packet}")

        cycle += 1
        time.sleep(0.05)   # 20 Hz to match planner loop rate


if __name__ == "__main__":
    main()

import math
import socket
import struct
import time

# --- Robot Physical Parameters ---
WHEEL_BASE = 0.08  # Distance between wheels in meters (80mm)
UPDATE_RATE_HZ = 50.0  # Simulation loop frequency (50 Hz -> dt = 0.02s)
TIMEOUT_LIMIT_SEC = 2.0  # Safety heartbeat limit


class VirtualRobot:

    def __init__(
        self, robot_id=1, start_x=0.5, start_y=0.5, start_theta=0.0
    ):
        self.id = robot_id
        self.x = start_x  # meters
        self.y = start_y  # meters
        self.theta = start_theta  # radians

        self.v_left = 0.0  # m/s
        self.v_right = 0.0  # m/s
        self.last_command_time = time.time()

    def update_kinematics(self, dt: float):
        """Updates robot position based on current wheel velocities."""
        # 1. Heartbeat Fail-Safe Check
        if time.time() - self.last_command_time > TIMEOUT_LIMIT_SEC:
            self.v_left = 0.0
            self.v_right = 0.0

        # 2. Differential Drive Math
        v = (self.v_right + self.v_left) / 2.0
        omega = (self.v_right - self.v_left) / WHEEL_BASE

        # 3. Euler Integration
        self.x += v * math.cos(self.theta) * dt
        self.y += v * math.sin(self.theta) * dt
        self.theta += omega * dt

        # Normalize angle to [-pi, pi]
        self.theta = math.atan2(math.sin(self.theta), math.cos(self.theta))


def main():
    # Setup non-blocking UDP Socket listening on localhost
    UDP_IP = "127.0.0.1"
    UDP_PORT = 5000
    TELEMETRY_PORT = 5001

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind((UDP_IP, UDP_PORT))
    sock.setblocking(False)

    telemetry_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    telemetry_addr = (UDP_IP, TELEMETRY_PORT)

    robot = VirtualRobot(robot_id=1)
    dt = 1.0 / UPDATE_RATE_HZ

    print(
        f"[Virtual Firmware] Robot #{robot.id} running. Listening on UDP {UDP_IP}:{UDP_PORT}..."
    )

    while True:
        loop_start = time.time()

        # Try reading incoming command packet (Expects 2 big-endian floats: v_left, v_right)
        try:
            data, _ = sock.recvfrom(1024)
            if len(data) == 8:
                v_l, v_r = struct.unpack("!ff", data)
                robot.v_left = v_l
                robot.v_right = v_r
                robot.last_command_time = time.time()
        except BlockingIOError:
            pass  # No packet received this iteration

        # Step physics forward
        robot.update_kinematics(dt)

        # Pack 3 floats (x, y, theta) into 12 bytes and send
        telemetry_packet = struct.pack("!fff", robot.x, robot.y, robot.theta)
        telemetry_sock.sendto(telemetry_packet, telemetry_addr)

        # Print current pose to console (clears line with \r)
        print(
            f"\r[Pose] X: {robot.x:6.3f}m | Y: {robot.y:6.3f}m | θ: {math.degrees(robot.theta):6.1f}° | V_L: {robot.v_left:5.2f} | V_R: {robot.v_right:5.2f}",
            end="",
        )

        # Enforce exact rate timing
        elapsed = time.time() - loop_start
        sleep_time = dt - elapsed
        if sleep_time > 0:
            time.sleep(sleep_time)


if __name__ == "__main__":
    main()
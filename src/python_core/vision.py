import socket
import struct

class MockVision:
    """
    Listens for UDP telemetry from the virtual firmware.
    Mimics the API of an overhead AprilTag camera system.
    """
    def __init__(self, ip="127.0.0.1", port=5001):
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.bind((ip, port))
        self.sock.setblocking(False)
        self.robot_pose = (0.5, 0.5, 0.0)  # Default starting x, y, theta

    def update(self):
        """
        Drains the socket buffer to fetch the most recent pose.
        Returns: Tuple of (x, y, theta)
        """
        latest_data = None
        while True:
            try:
                # Read 1024 bytes. We expect 12-byte packets (3 floats)
                data, _ = self.sock.recvfrom(1024)
                latest_data = data
            except BlockingIOError:
                break  # Buffer is empty
        
        if latest_data and len(latest_data) == 12:
            self.robot_pose = struct.unpack("!fff", latest_data)
        
        return self.robot_pose
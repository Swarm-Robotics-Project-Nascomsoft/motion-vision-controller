import socket
import struct
import time

class MockNetwork:
    """
    Handles UDP communication with the virtual firmware.
    Abstracts the network layer so the controller only deals with raw velocities.
    """
    def __init__(self, ip="127.0.0.1", port=5000):
        self.ip = ip
        self.port = port
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        print(f"[MockNetwork] Initialized UDP transmitter targeting {self.ip}:{self.port}")

    def send_velocity_command(self, v_left: float, v_right: float):
        """
        Packs two floats into an 8-byte array and sends via UDP.
        """
        # Struct format "!ff":
        # '!' = network (big-endian) byte order
        # 'f' = 32-bit float (v_left)
        # 'f' = 32-bit float (v_right)
        packet = struct.pack("!ff", v_left, v_right)
        
        # Fire and forget (UDP is connectionless)
        self.sock.sendto(packet, (self.ip, self.port))

if __name__ == "__main__":
    # --- Quick Integration Test ---
    net = MockNetwork()
    
    print("Commanding FORWARD (0.5 m/s) for 3 seconds...")
    start_time = time.time()
    while time.time() - start_time < 3.0:
        net.send_velocity_command(0.5, 0.5) 
        time.sleep(0.1)  # Command sent at 10Hz
        
    print("Commanding TURN (-0.2 m/s, 0.2 m/s) for 2 seconds...")
    start_time = time.time()
    while time.time() - start_time < 2.0:
        net.send_velocity_command(-0.2, 0.2) 
        time.sleep(0.1)
        
    print("Done sending commands.")
    print("Watch the virtual firmware terminal—it should zero out after the 2-second heartbeat timeout!")
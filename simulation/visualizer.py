import sys
import os
import cv2
import numpy as np
import math

# Add the parent directory to Python's path so we can import src/
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))
from src.python_core.vision import MockVision

def draw_robot(canvas, x, y, theta, pixels_per_meter=400):
    """Draws a robot footprint and heading vector on the canvas."""
    px = int(x * pixels_per_meter)
    py = int(y * pixels_per_meter)
    
    # Draw Robot Body (Green circle)
    cv2.circle(canvas, (px, py), 20, (0, 200, 0), -1)
    
    # Draw Heading Line (Red line indicating forward direction)
    end_x = int(px + 20 * math.cos(theta))
    end_y = int(py + 20 * math.sin(theta))
    cv2.line(canvas, (px, py), (end_x, end_y), (0, 0, 255), 3)

def main():
    vision = MockVision()
    print("[Visualizer] Starting OpenCV Arena View...")
    print("[Visualizer] Press 'q' on the window to quit.")

    # Main Rendering Loop
    while True:
        # Create a blank white canvas (800x800 pixels represents a 2m x 2m arena)
        canvas = np.ones((800, 800, 3), dtype=np.uint8) * 255
        
        # 1. Sense: Fetch ground truth from the Mock Vision system
        x, y, theta = vision.update()
        
        # 2. Render: Draw the robot on the canvas
        draw_robot(canvas, x, y, theta)
        
        # 3. Display
        cv2.imshow("PARROT SIL Simulation", canvas)
        
        # Render at ~50fps and wait for 'q' key to exit
        if cv2.waitKey(20) & 0xFF == ord('q'):
            break
            
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()
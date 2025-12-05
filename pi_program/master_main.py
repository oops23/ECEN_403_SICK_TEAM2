""" SICKSense Agripollinate Master Program Version 1.0
    Created by:    Josiah Faircloth 
    date:    11/30/2025

    Master control program integrating LiDAR object detection and image capture.
    Maintains dual TCP connections: one to LiDAR scanner, one to image client.
    Processes LiDAR data in real-time and triggers image burst on new object detection.
"""

import time
import signal
import sys
from datetime import datetime
from lidar_parser import LidarConnection
from object_detector import ObjectDetector
from image_capture import ImageServer


# -------------------------------
# Configuration Parameters
# -------------------------------

# LiDAR Configuration
LIDAR_HOST = "192.168.10.2"
LIDAR_PORT = 2112

# Image Server Configuration
IMAGE_SERVER_HOST = '0.0.0.0'
IMAGE_SERVER_PORT = 12345
IMAGE_SAVE_DIR = "/home/josiah/pi_program/captured_images"
IMAGE_RESOLUTION = '640x480' # max resolution: 3264x2448, default: 640x480
BURST_SIZE = 5  # Number of images to capture per trigger

# Object Detection Configuration
EPS = 0.25                      # DBSCAN cluster distance threshold (meters)
MIN_SAMPLES = 4                 # Minimum points per cluster
NEW_OBJ_DIST = 0.3              # Distance threshold for new objects (meters)
OBJECT_MEMORY_TIME = 1.0        # Time to remember objects (seconds)
MIN_DETECTIONS = 4             # Consecutive scans before triggering
TRIGGER_COOLDOWN = 2.0          # Minimum time between triggers (seconds)
ESTABLISHED_THRESHOLD = 2.0     # Time to be considered "established" (seconds)

# Watchdog Configuration
WATCHDOG_TIMEOUT = 5.0          # Maximum time between scans before watchdog alert (seconds)


# -------------------------------
# Global State
# -------------------------------
running = True
last_scan_time = None


def signal_handler(sig, frame):
    """Handle Ctrl+C gracefully"""
    global running
    print("\n[SHUTDOWN] Received interrupt signal, shutting down...")
    running = False


def watchdog_check(current_time):
    """Check if LiDAR data stream is still active"""
    global last_scan_time
    
    if last_scan_time is None:
        return True  # First scan, no issue
    
    time_since_last_scan = current_time - last_scan_time
    
    if time_since_last_scan > WATCHDOG_TIMEOUT:
        print(f"[WATCHDOG] WARNING: No LiDAR data for {time_since_last_scan:.2f}s!")
        return False
    
    return True


def on_object_detected(centroid, timestamp, object_count, image_server):
    """
    Callback function triggered when new object is detected.
    Captures and sends burst of images to client.
    """
    print(f"[ALERT] New object #{object_count} detected at {centroid} at {timestamp:.2f}s")
    
    # Trigger image capture burst
    if image_server and image_server.connected:
        image_server.capture_and_send_burst(num_images=BURST_SIZE)
    else:
        print("[WARNING] Image server not connected, skipping image capture")


def main():
    """
    Master main function coordinating all subsystems.
    """
    global running, last_scan_time
    
    # Set up signal handler for graceful shutdown
    signal.signal(signal.SIGINT, signal_handler)
    
    print("="*60)
    print("SICKSense Agripollinate Master Program")
    print("Version 1.0 - November 30, 2025")
    print("="*60)
    
    # Initialize components
    lidar = None
    image_server = None
    detector = None
    
    try:
        # 1. Connect to LiDAR
        # print("\n[INIT] Connecting to LiDAR scanner...")
        lidar = LidarConnection(host=LIDAR_HOST, port=LIDAR_PORT)
        print("\n[INIT] Connecting to LiDAR scanner...")
        if not lidar.connect():
            print("[ERROR] Failed to connect to LiDAR. Exiting.")
            return
        
        # 2. Start image server
        print("\n[INIT] Starting image server...")
        image_server = ImageServer(
            host=IMAGE_SERVER_HOST, 
            port=IMAGE_SERVER_PORT,
            save_dir=IMAGE_SAVE_DIR,
            resolution=IMAGE_RESOLUTION
        )
        if not image_server.start_server():
            print("[ERROR] Failed to start image server. Exiting.")
            lidar.disconnect()
            return
        
        # 3. Initialize object detector
        print("\n[INIT] Initializing object detector...")
        detector = ObjectDetector(
            eps=EPS,
            min_samples=MIN_SAMPLES,
            new_obj_dist=NEW_OBJ_DIST,
            object_memory_time=OBJECT_MEMORY_TIME,
            min_detections=MIN_DETECTIONS,
            trigger_cooldown=TRIGGER_COOLDOWN,
            established_threshold=ESTABLISHED_THRESHOLD
        )
        
        # Set trigger callback to capture images
        detector.set_trigger_callback(
            lambda centroid, timestamp, count: on_object_detected(
                centroid, timestamp, count, image_server
            )
        )
        
        print("\n[READY] All systems operational. Processing LiDAR data...")
        print("Press Ctrl+C to stop.\n")
        
        # 4. Main processing loop
        scan_count = 0
        start_time = time.time()
        
        while running:
            # Get next LiDAR scan
            scan_data = lidar.get_scan(timeout=1.0)
            
            if scan_data is None:
                # No scan received, check watchdog
                current_time = time.time()
                if not watchdog_check(current_time):
                    print("[WATCHDOG] Attempting to reconnect to LiDAR...")
                    lidar.disconnect()
                    time.sleep(1)
                    if not lidar.connect():
                        print("[ERROR] Failed to reconnect. Exiting.")
                        break
                    last_scan_time = None
                continue
            
            # Update watchdog timer
            last_scan_time = scan_data["timestamp"]
            scan_count += 1
            
            # Process scan through detector
            result = detector.process_scan(scan_data)
            
            # Print status every 100 scans
            if scan_count % 100 == 0:
                stats = detector.get_statistics()
                elapsed = time.time() - start_time
                print(f"[STATUS] Scans: {scan_count} | Objects: {stats['total_objects_detected']} "
                      f"| Active: {stats['active_objects']} | Elapsed: {elapsed:.1f}s")
        
        # Shutdown
        print("\n[SHUTDOWN] Main loop stopped")
        elapsed = time.time() - start_time
        stats = detector.get_statistics()
        print(f"\nFinal Statistics:")
        print(f"  Total scans processed: {scan_count}")
        print(f"  Total objects detected: {stats['total_objects_detected']}")
        print(f"  Runtime: {elapsed:.2f} seconds")
        print(f"  Average scan rate: {scan_count/elapsed:.1f} Hz")
        
    except Exception as e:
        print(f"\n[ERROR] Unexpected error: {e}")
        import traceback
        traceback.print_exc()
    
    finally:
        # Clean up all connections
        print("\n[CLEANUP] Closing connections...")
        
        if lidar:
            lidar.disconnect()
        
        if image_server:
            image_server.stop_server()
        
        print("[SHUTDOWN] Complete")


if __name__ == "__main__":
    main()

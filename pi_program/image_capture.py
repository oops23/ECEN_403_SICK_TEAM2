""" SICKSense Agripollinate Image Capture Module Version 1.0
    Created by:    Josiah Faircloth 
    date:    11/30/2025

    Handles image capture from USB camera and transmission to client via TCP.
    Adapted from Image Server Version 0.3 for integration into master program.
"""

import socket
import os
import subprocess
from datetime import datetime
import threading
import time


class ImageServer:
    """
    Manages TCP connection to image client and handles burst image capture/transmission.
    """
    
    def __init__(self, host='0.0.0.0', port=12345, save_dir="/home/josiah", resolution="640x480"):
        self.host = host
        self.port = port
        self.save_dir = save_dir
        self.resolution = resolution
        self.server_socket = None
        self.client_socket = None
        self.client_address = None
        self.connected = False
        self.running = False
        
        # Create save directory if it doesn't exist
        if not os.path.exists(self.save_dir):
            os.makedirs(self.save_dir)
    
    def start_server(self):
        """Start server and wait for client connection"""
        try:
            self.server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.server_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            self.server_socket.bind((self.host, self.port))
            self.server_socket.listen(1)
            self.running = True
            print(f"Image server listening on {self.host}:{self.port}")
            print("Waiting for client connection...")
            
            # Accept client connection
            self.client_socket, self.client_address = self.server_socket.accept()
            self.connected = True
            print(f"Image client connected: {self.client_address}")
            return True
            
        except Exception as e:
            print(f"Failed to start image server: {e}")
            self.running = False
            return False
    
    def stop_server(self):
        """Stop server and close connections"""
        self.running = False
        
        if self.client_socket:
            try:
                self.client_socket.send("QUIT".encode())
                self.client_socket.close()
            except:
                pass
        
        if self.server_socket:
            try:
                self.server_socket.close()
            except:
                pass
        
        self.connected = False
        print("Image server stopped")
    
    def capture_image(self):
        """Capture an image from USB camera using fswebcam"""
        try:
            # Create unique filename based on timestamp
            timestamp = datetime.now().strftime("%m-%d-%Y_%H.%M.%S.%f")[:-3]
            image_path = os.path.join(self.save_dir, f"captured_{timestamp}.jpg")
            
            # Capture image via USB camera
            subprocess.run(
                ["fswebcam", "-r", self.resolution, image_path], 
                check=True,
                stdout=subprocess.DEVNULL,
                stderr=subprocess.DEVNULL
            )
            print(f"Image captured: {image_path}")
            return image_path
            
        except subprocess.CalledProcessError:
            print("Failed to capture image")
            return None
    
    def send_image(self, image_path):
        """Send an image file to the client"""
        if not self.connected or not self.client_socket:
            print("No client connected")
            return False
        
        try:
            # Check if file exists
            if not os.path.exists(image_path):
                error_msg = f"ERROR: Image file '{image_path}' not found"
                self.client_socket.send(error_msg.encode())
                return False
            
            # Get & send file info (filename and size)
            file_size = os.path.getsize(image_path)
            filename = os.path.basename(image_path)
            file_info = f"{filename}:{file_size}"
            self.client_socket.send(file_info.encode())
            
            # Wait for client acknowledgment
            ack = self.client_socket.recv(1024).decode()
            if ack != "READY":
                print("Client not ready to receive file")
                return False
            
            # Send the image file in chunks
            with open(image_path, 'rb') as f:
                bytes_sent = 0
                while bytes_sent < file_size:
                    chunk = f.read(4096)
                    if not chunk:
                        break
                    self.client_socket.send(chunk)
                    bytes_sent += len(chunk)
            
            print(f"Successfully sent '{filename}' ({file_size} bytes)")
            return True
            
        except Exception as e:
            print(f"Error sending image: {e}")
            self.connected = False
            return False
    
    def capture_and_send_burst(self, num_images=5):
        """
        Capture and send a burst of images to the client.
        This is the main trigger response function.
        
        Args:
            num_images: Number of images to capture in the burst
        
        Returns:
            int: Number of images successfully captured and sent
        """
        if not self.connected:
            print("Cannot send burst: No client connected")
            return 0
        
        print(f"[IMAGE BURST] Capturing and sending {num_images} images...")
        success_count = 0
        
        for i in range(num_images):
            image_path = self.capture_image()
            
            if image_path:
                if self.send_image(image_path):
                    success_count += 1
                else:
                    print(f"Failed to send image {i+1}/{num_images}")
                    break
            else:
                try:
                    self.client_socket.send("ERROR: Failed to capture image".encode())
                except:
                    pass
                break
            
            # Small delay between captures
            # time.sleep(0.05)
        
        print(f"[IMAGE BURST] Completed: {success_count}/{num_images} images sent")
        return success_count
    
    def __enter__(self):
        """Context manager support"""
        self.start_server()
        return self
    
    def __exit__(self, exc_type, exc_val, exc_tb):
        """Context manager cleanup"""
        self.stop_server()

""" SICKSense Agripollinate Image Client Version 0.3 
    Created by:    Josiah Faircloth 
    date:    10/19/2025

    Compatible with Image Server Version 0.3
    
    Recieves a burst of images using Transmission Control Protocol (TCP). 
    Server connection is constant until aborted. Each image is saved with 
    a unique timestamped filename.
"""

import socket
import os
from datetime import datetime

def receive_image(client_socket, download_dir="downloads"):
    """Receive an image file from the server"""
    try:
        # Create download directory if it doesn't exist
        if not os.path.exists(download_dir):
            os.makedirs(download_dir)
        
        # Receive file info (filename and size)
        file_info = client_socket.recv(1024).decode()
        
        # Check for error messages
        if not file_info:
            return True  # No data, continue listening

        if file_info.startswith("ERROR:"):
            print(file_info)
            return False
        
        if file_info == "QUIT":
            print("Server is shutting down.")
            return False
        
        # Parse filename and file size
        try:
            filename, file_size_str = file_info.split(':')
            file_size = int(file_size_str)
        except ValueError:
            print("Invalid file info received from server")
            return False

        print(f"Receiving file: {filename} ({file_size} bytes)")
        
        # Send acknowledgment
        client_socket.send("READY".encode())
        
        # Prepare file path
        file_path = os.path.join(download_dir, filename)
        
        # Receive the file in chunks
        with open(file_path, 'wb') as f:
            bytes_received = 0
            while bytes_received < file_size:
                # Calculate how much to receive (remaining bytes or chunk size)
                remaining = file_size - bytes_received
                chunk_size = min(4096, remaining)
                
                chunk = client_socket.recv(chunk_size)
                if not chunk:
                    print("Connection lost while receiving file")
                    return False
                
                f.write(chunk)
                bytes_received += len(chunk)
                print(f"Received {bytes_received}/{file_size} bytes ({(bytes_received/file_size)*100:.1f}%)")
        
        print(f"Successfully received and saved '{filename}' to '{file_path}'")
        return True
        
    except Exception as e:
        print(f"Error receiving image: {e}")
        return False

def main():
    # Server configuration - Change this to the server's IP address
    HOST = '10.250.76.217'  # Rasp Pi : 192.168.1.74 (Home WIFI) : 10.250.76.217 (TAMU IoT)
    PORT = 12345

    try:
        # Create TCP socket and connect to server
        client_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        client_socket.connect((HOST, PORT))
        print(f"Connected to server at {HOST}:{PORT}")
        print("Waiting for images... (Ctrl+C to quit)\n")

        while True:
            # Receive image from server
            success = receive_image(client_socket)
            
            if not success:
                break
                
    except ConnectionRefusedError:
        print(f"Could not connect to server at {HOST}:{PORT}")
        print("Make sure the server is running.")
        
    except KeyboardInterrupt:
        print("\nClient shutting down...")
        
    except Exception as e:
        print(f"Client error: {e}")
        
    finally:
        client_socket.close()
        print("Connection Closed.")


if __name__ == "__main__":
    main()

""" SICKSense Agripollinate Image Server Version 0.1 
    Created by:    Josiah Faircloth 
    date:    9/30/2025

    Compatible with Image Client Version 0.1
    
    Sends an image file (specified by keyboard input) to a connected client using 
    Transmission Control Protocol (TCP).    
"""

import socket
import os

def send_image(client_socket, image_path):
    """Send an image file to the client"""
    try:
        # Check if file exists
        if not os.path.exists(image_path):
            error_msg = f"ERROR: Image file '{image_path}' not found"
            client_socket.send(error_msg.encode())
            return False
        
        # Get file size
        file_size = os.path.getsize(image_path)
        
        # Send file info (filename and size)
        filename = os.path.basename(image_path)
        file_info = f"{filename}:{file_size}"
        client_socket.send(file_info.encode())
        
        # Wait for client acknowledgment
        ack = client_socket.recv(1024).decode()
        if ack != "READY":
            print("Client not ready to receive file")
            return False
        
        # Send the image file in chunks
        with open(image_path, 'rb') as f:
            bytes_sent = 0
            while bytes_sent < file_size:
                chunk = f.read(4096)  # Read 4KB chunks
                if not chunk:
                    break
                client_socket.send(chunk)
                bytes_sent += len(chunk)
                print(f"Sent {bytes_sent}/{file_size} bytes ({(bytes_sent/file_size)*100:.1f}%)")
        
        print(f"Successfully sent '{filename}' ({file_size} bytes)")
        return True
        
    except Exception as e:
        print(f"Error sending image: {e}")
        return False

def main():
    # Server configuration
    HOST = '10.244.115.156'
    PORT = 12345
    
    # Create TCP socket
    server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    
    try:
        # Bind and listen
        server_socket.bind((HOST, PORT))
        server_socket.listen(1)
        print(f"Server listening on {HOST}:{PORT}")
        print("Waiting for client connection...")
        
        while True:
            # Accept client connection
            client_socket, client_address = server_socket.accept()
            print(f"Connected to client: {client_address}")
            
            try:
                # Get image path from user
                image_path = input("Enter the path to the image file (or 'quit' to exit): ").strip()
                
                if image_path.lower() == 'quit':
                    client_socket.send("QUIT".encode())
                    break
                
                # Send the image
                success = send_image(client_socket, image_path)

                if success:
                    print("Image sent successfully!")
                else:
                    print("Failed to send image.")
                
            except Exception as e:
                print(f"Error handling client: {e}")
            
            finally:
                client_socket.close()
                print("Client disconnected")
    
    except KeyboardInterrupt:
        print("\nServer shutting down...")
    except Exception as e:
        print(f"Server error: {e}")
    finally:
        server_socket.close()

if __name__ == "__main__":
    main()
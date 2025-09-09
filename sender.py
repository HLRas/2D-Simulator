import socket
import time
import random
import subprocess
import threading

def main():
    print("TCP Sender - Starting server...")
    
    # Create server socket
    server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)  # Allow reuse of address
    
    try:
        server_socket.bind((socket.gethostname(), 1234))
        server_socket.listen(5)
        print(f"Server listening on {socket.gethostname()}:1234")
        print("Waiting for client connections...")
        
        while True:
            # Accept client connection
            client_socket, address = server_socket.accept()
            print(f"Connection from {address} has been established!")
            
            try:
                # Send random coordinates every 3 seconds
                for i in range(10):  # Send 10 coordinates then disconnect (adjust as needed)
                    # Generate random coordinates (adjust range as needed for your simulation)
                    x = random.randint(50, 800)  # Adjust range based on your simulation size
                    y = random.randint(50, 600)  # Adjust range based on your simulation size
                    
                    # Format and send coordinate
                    message = f"{x},{y}"
                    client_socket.send(message.encode("utf-8"))
                    print(f"Sent coordinates: {message}")
                    
                    # Wait 3 seconds before sending next coordinate
                    time.sleep(3)
                
                print("Finished sending coordinates, closing connection")
                
            except Exception as e:
                print(f"Error sending data: {e}")
            finally:
                client_socket.close()
                print("Client connection closed")
                
    except KeyboardInterrupt:
        print("\nShutting down server...")
    except Exception as e:
        print(f"Server error: {e}")
    finally:
        server_socket.close()
        print("Server socket closed")
        
        # Terminate main.py process
        if main_process.poll() is None:  # Process is still running
            print("Terminating main.py...")
            main_process.terminate()
            main_process.wait()  # Wait for process to finish
            print("main.py terminated")

if __name__ == '__main__':
    main()

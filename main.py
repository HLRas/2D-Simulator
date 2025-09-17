import pygame
import sys
import math
import os
import socket
import threading
import time
import serial
from car import Car
from map_generation import Map
# from menu import MainMenu  # Commented out for headless mode
from config import *

# Configuration for headless/automated mode
# Configuration flags for headless and automated modes
HEADLESS_MODE = True  # Set to True to run without GUI (using SDL dummy driver)
AUTO_PATHFINDING = True  # Set to True to automatically start pathfinding after positioning
PATHFINDING_METHOD = "cross_track"  # Options: "carrot" or "cross_track"
ENABLE_ARDUINO = True  # Set to False to disable Arduino communication for testing

# Set SDL to use dummy video driver for headless operation (only if headless mode is enabled)
if HEADLESS_MODE:
    os.environ['SDL_VIDEODRIVER'] = 'dummy'

# --- TCP Client for receiving coordinates (headless mode only) ---
received_coords = None
last_coord_time = 0
coord_lock = threading.Lock()

# --- Arduino Serial Communication for Wheel Speeds ---
arduino_serial = None
wheel_speed_queue = []
arduino_lock = threading.Lock()

def arduino_thread():
    """Dedicated thread for Arduino communication"""
    global arduino_serial, wheel_speed_queue
    
    # Initialize Arduino connection to specific port
    try:
        arduino_serial = serial.Serial('/dev/ttyACM0', 9600, timeout=0.1)  # Reduced timeout
        time.sleep(2)  # Wait for Arduino reset
        print("[Arduino] Connected to /dev/ttyACM0")
            
    except Exception as e:
        print(f"[Arduino] Connection error: {e}")
        print("[Arduino] Running in simulation-only mode (no Arduino)")
        # Continue running without Arduino - don't return
        arduino_serial = None
    
    # Main Arduino communication loop - send every 10ms
    last_sent_speeds = (0.0, 0.0)
    last_send_time = time.time()
    send_interval = 0.01  # 10ms = 0.01 seconds
    loop_count = 0
    
    while True:
        try:
            loop_count += 1
            current_time = time.time()
            
            # Debug: Print every 1000 loops to see if thread is running
            if loop_count % 1000 == 0:
                print(f"[Arduino] Thread running, loop {loop_count}")
            
            # Only try serial communication if Arduino is connected
            if arduino_serial:
                # Check for incoming serial data from Arduino (non-blocking)
                if arduino_serial.in_waiting > 0:
                    try:
                        incoming_data = arduino_serial.readline().decode('utf-8').strip()
                        if incoming_data:
                            print(f"[Arduino] Received: {incoming_data}")
                    except Exception as read_error:
                        print(f"[Arduino] Read error: {read_error}")
            
            # Check if it's time to send (every 10ms)
            if current_time - last_send_time >= send_interval:
                # Get latest wheel speeds
                left_speed = last_sent_speeds[0]  # Default to last sent
                right_speed = last_sent_speeds[1]
                
                # Check for new wheel speeds (get the most recent)
                with arduino_lock:
                    if wheel_speed_queue:
                        # Get most recent speed (clear all old ones)
                        while wheel_speed_queue:
                            left_speed, right_speed = wheel_speed_queue.pop(0)
                
                # Send wheel speeds every 10ms regardless of change
                if arduino_serial:
                    try:
                        data = f"{left_speed:.3f},{right_speed:.3f}\n"
                        arduino_serial.write(data.encode('utf-8'))
                        arduino_serial.flush()
                        last_sent_speeds = (left_speed, right_speed)
                        last_send_time = current_time
                        
                        # Less frequent logging to avoid spam
                        if loop_count % 100 == 0:
                            print(f"[Arduino] Sent: L={left_speed:.3f}, R={right_speed:.3f}")
                    except Exception as write_error:
                        print(f"[Arduino] Write error: {write_error}")
                else:
                    # Simulate sending (for debugging)
                    last_sent_speeds = (left_speed, right_speed)
                    last_send_time = current_time
                    if loop_count % 1000 == 0:  # Much less frequent logging
                        print(f"[Arduino] Simulated send: L={left_speed:.3f}, R={right_speed:.3f}")
            
            # Short sleep to prevent excessive CPU usage
            time.sleep(0.001)  # 1ms sleep
            
        except Exception as e:
            print(f"[Arduino] Communication error: {e}")
            time.sleep(0.01)  # Brief pause before retry
            continue
    
    # Cleanup
    if arduino_serial:
        arduino_serial.close()
        print("[Arduino] Connection closed")

def queue_wheel_speeds(left_speed, right_speed):
    """Queue wheel speeds for sending to Arduino"""
    with arduino_lock:
        wheel_speed_queue.append((left_speed, right_speed))

def tcp_receiver_thread():
    global received_coords
    s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    try:
        s.connect((socket.gethostname(), 1234))
        print("[Receiver] Connected, waiting 5 seconds...")
        time.sleep(5)  # Wait 5 seconds AFTER connecting
        print("[Receiver] Now receiving first coordinate...")
        
        msg = s.recv(21)  # Receive exactly one message limited to 21 characters
        if msg:
            try:
                decoded = msg.decode("utf-8").strip()
                if "," in decoded:
                    parts = decoded.split(",")
                    
                    x_str, y_str, orientation_str = parts[0], parts[1], parts[2]
                    x, y, orientation = float(x_str), float(y_str), float(orientation_str)
                    with coord_lock:
                        received_coords = (x, y, orientation)
                    print(f"[Receiver] Received coordinate: {x}, {y}, {orientation}")
            except Exception as e:
                print(f"[Receiver] Error parsing message: {msg} ({e})")
        else:
            print("[Receiver] No message received")
            
        print("[Receiver] Closing connection after receiving first coordinate")
            
    except Exception as e:
        print(f"[Receiver] Socket error: {e}")
    finally:
        s.close()

def handle_automated_pathfinding(frame_count, game_map, car):
    """Handle automated pathfinding setup for headless mode"""
    if not AUTO_PATHFINDING:
        return False
        
    auto_pathfinding_started = getattr(handle_automated_pathfinding, 'started', False)
    
    if not auto_pathfinding_started:
        if frame_count == 10:  # Set start position after a few frames
            car_center = car.get_rect().center
            cube = game_map.get_cube(car_center)
            if cube:
                if game_map.start:
                    game_map.start.make_clear()
                    game_map.mark_dirty(game_map.start)
                
                cube.make_start()
                game_map.start = cube
                game_map.mark_dirty(cube)
                print(f"Frame {frame_count}: Auto-set start position at ({car.x:.1f}, {car.y:.1f})")
                
        elif frame_count == 50:  # Start pathfinding after start is set
            if game_map.start:
                nearest_space = game_map._find_nearest_parking_space(car)
                if nearest_space and nearest_space.target_cube:
                    game_map.end = nearest_space.target_cube
                    game_map._update_neighbors_if_needed()
                    game_map.pathfinder.clear_path(game_map.cubes, game_map.mark_dirty)
                    path_found = game_map.pathfinder.pathfind(game_map.cubes, game_map.start, game_map.end, game_map.mark_dirty)
                    
                    if path_found:
                        if PATHFINDING_METHOD == "cross_track":
                            car.start_cross_track_following(game_map.pathfinder.path_points)
                            print(f"Frame {frame_count}: Auto-started cross-track pathfinding to parking space")
                        else:  # Default to carrot
                            car.start_path_following(game_map.pathfinder.path_points)
                            print(f"Frame {frame_count}: Auto-started carrot pathfinding to parking space")
                        handle_automated_pathfinding.started = True
                        auto_pathfinding_started = True
                    else:
                        print(f"Frame {frame_count}: Pathfinding failed!")
    
    return auto_pathfinding_started

def run_simulation(layout_type):
    """Run the simulation with the specified layout type"""
    global received_coords, last_coord_time
    # Create game objects with selected layout
    game_map = Map(layout_type)
    car = Car(50, 50)

    # Set up display
    layout_names = ["Default Layout", "Empty Layout", "Minimal Layout"]
    caption = f"2D Car Simulation - {layout_names[layout_type]}"
    pygame.display.set_caption(caption)
    clock = pygame.time.Clock()

    # Performance tracking
    frame_count = 0

    # Start TCP receiver thread in headless mode
    if HEADLESS_MODE:
        receiver_thread = threading.Thread(target=tcp_receiver_thread, daemon=True)
        receiver_thread.start()
        
        # Start Arduino communication thread only if enabled
        if ENABLE_ARDUINO:
            arduino_comm_thread = threading.Thread(target=arduino_thread, daemon=True)
            arduino_comm_thread.start()
        else:
            print("[Arduino] Communication disabled - running simulation only")

    if HEADLESS_MODE:
        print("Starting headless simulation...")
        print("Waiting for TCP coordinate before starting pathfinding...")
    elif AUTO_PATHFINDING:
        print("Will automatically set start position and begin pathfinding...")

    # For coordinate update timing
    coordinate_processed = False

    # Main game loop
    while True:
        dt = clock.tick(FPS) / 1000.0  # Delta time in seconds
        frame_count += 1
        
        # --- Check for received coordinates (headless mode only) ---
        if HEADLESS_MODE and not coordinate_processed:
            with coord_lock:
                coords = received_coords
            if coords:
                x, y, orientation = coords
                print(f"[Receiver] Setting car position to ({x:.1f}, {y:.1f}) with orientation {math.degrees(orientation):.1f}° at frame {frame_count}")
                car.set_position((x,y))
                car.set_orientation(orientation)
                
                coordinate_processed = True
                
                # Execute pathfinding once after receiving coordinate
                # Set start position
                car_center = car.get_rect().center
                cube = game_map.get_cube(car_center)
                if cube:
                    if game_map.start:
                        game_map.start.make_clear()
                        game_map.mark_dirty(game_map.start)
                    cube.make_start()
                    game_map.start = cube
                    game_map.mark_dirty(cube)
                    print(f"[Receiver] Auto-set start position at ({car.x:.1f}, {car.y:.1f})")
                
                # Find nearest parking space and pathfind
                nearest_space = game_map._find_nearest_parking_space(car)
                if nearest_space and nearest_space.target_cube:
                    game_map.end = nearest_space.target_cube
                    game_map._update_neighbors_if_needed()
                    game_map.pathfinder.clear_path(game_map.cubes, game_map.mark_dirty)
                    path_found = game_map.pathfinder.pathfind(game_map.cubes, game_map.start, game_map.end, game_map.mark_dirty)
                    if path_found:
                        if PATHFINDING_METHOD == "cross_track":
                            car.start_cross_track_following(game_map.pathfinder.path_points)
                            print(f"[Receiver] Auto-started cross-track pathfinding to parking space")
                        else:  # Default to carrot
                            car.start_path_following(game_map.pathfinder.path_points)
                            print(f"[Receiver] Auto-started carrot pathfinding to parking space")
                    else:
                        print(f"[Receiver] Pathfinding failed!")

        # Handle automated pathfinding 
        if AUTO_PATHFINDING and not HEADLESS_MODE:
            # Normal auto-pathfinding for GUI mode
            auto_pathfinding_started = handle_automated_pathfinding(frame_count, game_map, car)
        elif AUTO_PATHFINDING and HEADLESS_MODE and not coordinate_processed:
            # Auto-pathfinding for headless mode (without TCP coordinates)
            auto_pathfinding_started = handle_automated_pathfinding(frame_count, game_map, car)

        # Print periodic status updates in headless mode
        if HEADLESS_MODE and frame_count % 500 == 0:
            status = "Carrot" if car.following_path else "Cross-track" if car.cross_track_following else "Manual"
            print(f"Frame {frame_count}: Car at ({car.x:.1f}, {car.y:.1f}), Mode: {status}")

        # Handle events (only in GUI mode)
        if not HEADLESS_MODE:
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    pygame.quit()
                    sys.exit()

                elif event.type == pygame.KEYDOWN:
                    # Return to main menu with ESC key
                    if event.key == pygame.K_ESCAPE:
                        return main()  # Restart from menu

                    game_map.check_inputs(event, car)

                    # Stop autopilot with SPACE key
                    if event.key == pygame.K_SPACE and (car.following_path or car.cross_track_following):
                        car.stop_path_following()
                        print("Autopilot manually stopped")

                    # Add debugging info
                    if event.key == pygame.K_F1:
                        print(f"Car position: ({car.x:.1f}, {car.y:.1f})")
                        print(f"Car angle: {math.degrees(car.angle):.1f}°")
                        print(f"Wheel speeds: L={car.wheel_L:.2f}, R={car.wheel_R:.2f}")
                        print(f"Carrot following: {car.following_path}")
                        print(f"Cross-track following: {car.cross_track_following}")

            # Handle continuous inputs
            keys = pygame.key.get_pressed()
            if any(keys):
                car.check_inputs(dt, keys)
        else:
            # In headless mode, still need to process pygame events to prevent hanging
            pygame.event.pump()

        # Update path following if active
        car.update_path_following(dt)

        # Update car physics with collision detection
        obstacles = game_map.get_obstacles()
        car.find_next_pos(dt, obstacles)
        
        # Send wheel speeds to Arduino when they change (headless mode only)
        if HEADLESS_MODE and ENABLE_ARDUINO:
            speeds = car.get_speeds()
            queue_wheel_speeds(speeds[0], speeds[1])

        # Check parking status
        for space in game_map.parking_spaces:
            if space.is_car_in_space(car):
                if not space.occupied:
                    space.set_occupied(True, game_map.cubes)
                    print(f"SUCCESS! Car parked in space at frame {frame_count}!")
                    print(f"Final car position: ({car.x:.1f}, {car.y:.1f})")
                    
                    # Send zero speeds to Arduino to stop the car
                    if HEADLESS_MODE and ENABLE_ARDUINO:
                        queue_wheel_speeds(0.0, 0.0)
                        print("[Arduino] Sent stop command: L=0.000, R=0.000")
                    
                    print("Headless simulation completed successfully!")
                    pygame.quit()
                    return
            else:
                # Only set to unoccupied if it's not permanently occupied
                if space.occupied and not space.permanently_occupied:
                    space.set_occupied(False, game_map.cubes)

        # Draw everything (only in GUI mode)
        if not HEADLESS_MODE:
            game_map.draw()
            car.draw() 

            # Show FPS and controls info
            fps = clock.get_fps()
            if car.following_path:
                status = " - CARROT FOLLOWING"
            elif car.cross_track_following:
                status = " - CROSS-TRACK FOLLOWING"
            else:
                status = ""
            pygame.display.set_caption(f"{caption} - FPS: {fps:.1f}{status} - ESC: Menu")

            pygame.display.flip()

def main():
    """Main application entry point"""
    # Initialize Pygame
    pygame.init()
    pygame.font.init()
    
    if HEADLESS_MODE:
        print("Running in headless mode - skipping menu")
        print("Using default layout (Layout 0)")
        # Skip menu and use default layout for headless mode
        selected_layout = 0
    else:
        # Show main menu and get selected layout
        from menu import MainMenu
        menu = MainMenu()
        selected_layout = menu.run()
    
    # Start the simulation with the selected layout
    run_simulation(selected_layout)

if __name__ == '__main__':
    main()
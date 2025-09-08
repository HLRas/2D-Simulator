import pygame
import sys
import math
import os
from car import Car
from map_generation import Map
# from menu import MainMenu  # Commented out for headless mode
from config import *

# Configuration for headless/automated mode
# Configuration flags for headless and automated modes
HEADLESS_MODE = True  # Set to True to run without GUI (using SDL dummy driver)
AUTO_PATHFINDING = True  # Set to True to automatically start pathfinding after positioning

# Set SDL to use dummy video driver for headless operation (only if headless mode is enabled)
if HEADLESS_MODE:
    os.environ['SDL_VIDEODRIVER'] = 'dummy'

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
                        car.start_path_following(game_map.pathfinder.path_points)
                        print(f"Frame {frame_count}: Auto-started carrot pathfinding to parking space")
                        handle_automated_pathfinding.started = True
                        auto_pathfinding_started = True
                    else:
                        print(f"Frame {frame_count}: Pathfinding failed!")
    
    return auto_pathfinding_started

def run_simulation(layout_type):
    """Run the simulation with the specified layout type"""
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
    
    if HEADLESS_MODE and AUTO_PATHFINDING:
        print("Starting headless simulation...")
        print("Will automatically set start position and begin pathfinding...")
    elif AUTO_PATHFINDING:
        print("Will automatically set start position and begin pathfinding...")
    
    # Main game loop
    while True:
        dt = clock.tick(FPS) / 1000.0  # Delta time in seconds
        frame_count += 1
        
        # Handle automated pathfinding (if enabled)
        if AUTO_PATHFINDING:
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
        
        # Check parking status
        for space in game_map.parking_spaces:
            if space.is_car_in_space(car):
                if not space.occupied:
                    space.set_occupied(True, game_map.cubes)
                    print(f"SUCCESS! Car parked in space at frame {frame_count}!")
                    print(f"Final car position: ({car.x:.1f}, {car.y:.1f})")
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

if __name__ == '__main__':
    main()
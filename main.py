import pygame
import sys
import math
from car import Car
from map_generation import Map
from menu import MainMenu
from config import *

def main():
    """Main application entry point"""
    # Initialize Pygame
    pygame.init()
    pygame.font.init()
    
    # Show main menu and get selected layout
    menu = MainMenu()
    selected_layout = menu.run()
    
    # Start the simulation with the selected layout
    run_simulation(selected_layout)

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
    
    # Main game loop
    while True:
        dt = clock.tick(FPS) / 1000.0  # Delta time in seconds
        frame_count += 1
        
        # Handle events
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
                    print(f"Car parked in space!")
            else:
                # Only set to unoccupied if it's not permanently occupied
                if space.occupied and not space.permanently_occupied:
                    space.set_occupied(False, game_map.cubes)
        
        # Draw everything
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
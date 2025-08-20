import pygame
import sys
import math
from car import Car
from map_generation import Map
from config import *

def main():
    """Main game loop with improved performance and structure"""
    # Initialize Pygame
    pygame.init()
    pygame.font.init()
    
    # Create game objects
    game_map = Map()
    car = Car(50, 50)
    
    # Set up display
    pygame.display.set_caption("2D Car Simulation")
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
                game_map.check_inputs(event, car)
                
                # Stop autopilot with SPACE key
                if event.key == pygame.K_SPACE and car.following_path:
                    car.stop_path_following()
                    print("Autopilot manually stopped")
                
                # Add debugging info
                if event.key == pygame.K_F1:
                    print(f"Car position: ({car.x:.1f}, {car.y:.1f})")
                    print(f"Car angle: {math.degrees(car.angle):.1f}°")
                    print(f"Wheel speeds: L={car.wheel_L:.2f}, R={car.wheel_R:.2f}")
                    print(f"Following path: {car.following_path}")
        
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
                    space.occupied = True
                    print(f"Car parked in space!")
            else:
                space.occupied = False
        
        # Draw everything
        game_map.draw()
        car.draw() 
        
        # Show FPS every second
        fps = clock.get_fps()
        status = " - AUTO PILOT" if car.following_path else ""
        pygame.display.set_caption(f"2D Car Simulation - FPS: {fps:.1f}{status}")
        
        pygame.display.flip()

if __name__ == '__main__':
    main()
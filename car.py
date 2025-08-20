import pygame
import math
from config import *

class Car:
    def __init__(self, x=50, y=50):
        """Initialize the car with its properties"""
        # Position
        self.x = float(x)
        self.y = float(y)
        self.prev_x = self.x
        self.prev_y = self.y

        # Movement properties
        self.wheel_acceleration = CAR_ACCELERATION
        self.wheel_friction = CAR_FRICTION
        self.max_angular_wheelspeed = MAX_WHEEL_SPEED
        
        # Wheel speeds
        self.wheel_L = 0.0
        self.wheel_R = 0.0
        
        # Vehicle properties
        self.speed = 0.0
        self.angle = 0.0
        self.width = CAR_WIDTH
        self.length = CAR_LENGTH
        self.wheelrad = WHEEL_RADIUS

        # Collision detection
        self.collision_points = []
        self._update_collision_points()

        # Visual representation
        self._create_surface()
        self.screen = pygame.display.get_surface()
        
        # Path following
        self.following_path = False
        self.path_points = []
        self.current_target_index = 0
        self.carrot_distance = 25  
        self.arrival_threshold = 18  
        
    def _create_surface(self):
        """Create the car's visual surface"""
        self.car_surface = pygame.Surface((self.length, self.width), pygame.SRCALPHA)
        self.car_surface.fill(YELLOW)
        
        # Draw direction indicator
        front_center = (self.length - 1, self.width // 2)
        mid_center = (self.length // 2, self.width // 2)
        pygame.draw.line(self.car_surface, BLACK, mid_center, front_center, 4)
        pygame.draw.rect(self.car_surface, BLACK, self.car_surface.get_rect(), 2)
        
        self.car_rect = self.car_surface.get_rect()

    def _update_collision_points(self):
        """Update collision detection points based on car's current position and angle"""
        # Define corner points relative to car center
        half_length = self.length / 2
        half_width = self.width / 2
        
        corners = [
            (-half_length, -half_width),  # Back left
            (half_length, -half_width),   # Front left
            (half_length, half_width),    # Front right
            (-half_length, half_width)    # Back right
        ]
        
        # Rotate and translate points
        self.collision_points = []
        cos_angle = math.cos(self.angle)
        sin_angle = math.sin(self.angle)
        
        for dx, dy in corners:
            # Rotate
            rotated_x = dx * cos_angle - dy * sin_angle
            rotated_y = dx * sin_angle + dy * cos_angle
            
            # Translate to world position
            world_x = self.x + rotated_x
            world_y = self.y + rotated_y
            
            self.collision_points.append((world_x, world_y))

    def check_collision(self, obstacles):
        """Check if car collides with any obstacles"""
        for point in self.collision_points:
            # Check bounds
            if (point[0] < 0 or point[0] >= SCREEN_WIDTH or 
                point[1] < 0 or point[1] >= SCREEN_HEIGHT):
                return True
                
            # Check grid obstacles
            grid_x = int(point[0] // CUBE_SIZE)
            grid_y = int(point[1] // CUBE_SIZE)
            
            if (0 <= grid_x < len(obstacles[0]) and 0 <= grid_y < len(obstacles) and
                obstacles[grid_y][grid_x].is_barrier()):
                return True
        
        return False
        
    def draw(self):
        """Draw the car on screen"""
        self.car_rect.center = (self.x, self.y)
        rotated_surf = pygame.transform.rotate(self.car_surface, math.degrees(self.angle))
        rotated_rect = rotated_surf.get_rect(center=self.car_rect.center)
        self.screen.blit(rotated_surf, rotated_rect.topleft)

    def get_grid_position(self):
        """Get car's position in grid coordinates"""
        return (int(self.x // CUBE_SIZE), int(self.y // CUBE_SIZE))
    
    def apply_friction(self, dt):
        """Apply friction to the wheels"""
        friction_force = self.wheel_friction * dt
        
        # Left wheel friction
        if abs(self.wheel_L) <= friction_force:
            self.wheel_L = 0
        elif self.wheel_L > 0:
            self.wheel_L -= friction_force
        else:
            self.wheel_L += friction_force
            
        # Right wheel friction
        if abs(self.wheel_R) <= friction_force:
            self.wheel_R = 0
        elif self.wheel_R > 0:
            self.wheel_R -= friction_force
        else:
            self.wheel_R += friction_force

    def find_next_pos(self, dt, obstacles):
        """Calculate and apply next position with collision detection"""
        # Store previous position
        self.prev_x, self.prev_y = self.x, self.y
        prev_angle = self.angle
        
        # Apply friction
        self.apply_friction(dt)
        
        # Calculate movement
        self.speed = (self.wheel_L + self.wheel_R) * self.wheelrad / 2
        self.angle += (self.wheel_R - self.wheel_L) * self.wheelrad / self.width * dt * TURN_RATE
        
        # Calculate new position
        new_x = self.x + self.speed * math.cos(self.angle) * dt * self.wheelrad
        new_y = self.y - self.speed * math.sin(self.angle) * dt * self.wheelrad
        
        # Update position (collision disabled for now)
        self.x, self.y = new_x, new_y
        
        # Keep car within screen bounds
        self.x = max(0, min(self.x, SCREEN_WIDTH))
        self.y = max(0, min(self.y, SCREEN_HEIGHT))
        
        # Update collision points with final position
        self._update_collision_points()

    def change_wheel_speed(self, wheel, dt, accelerate):
        """Change speed of specified wheel"""
        delta = self.wheel_acceleration * dt * (1 if accelerate else -1)
        
        if wheel == 'left':
            self.wheel_L = max(-self.max_angular_wheelspeed, 
                              min(self.wheel_L + delta, self.max_angular_wheelspeed))
        elif wheel == 'right':
            self.wheel_R = max(-self.max_angular_wheelspeed, 
                              min(self.wheel_R + delta, self.max_angular_wheelspeed))

    def get_rect(self):
        """Get the rectangle of the car for collision detection"""
        return self.car_rect
    
    def apply_brakes(self, dt):
        """Apply brakes to both wheels"""
        brake_force = self.wheel_acceleration * dt
        
        # Brake left wheel
        if abs(self.wheel_L) <= brake_force:
            self.wheel_L = 0
        elif self.wheel_L > 0:
            self.wheel_L -= brake_force
        else:
            self.wheel_L += brake_force
            
        # Brake right wheel
        if abs(self.wheel_R) <= brake_force:
            self.wheel_R = 0
        elif self.wheel_R > 0:
            self.wheel_R -= brake_force
        else:
            self.wheel_R += brake_force

    def check_inputs(self, dt, keys):
        """Process keyboard inputs"""
        # Only allow manual control if not following a path
        if not self.following_path:
            if keys[pygame.K_z]:
                self.change_wheel_speed('left', dt, True)
            if keys[pygame.K_x]:
                self.change_wheel_speed('left', dt, False)
            if keys[pygame.K_COMMA]:
                self.change_wheel_speed('right', dt, True)
            if keys[pygame.K_PERIOD]:
                self.change_wheel_speed('right', dt, False)
    
    def start_path_following(self, path_points):
        """Start following a path using carrot-stick method"""
        if len(path_points) < 2:
            return False
            
        self.path_points = path_points[:]  # Copy the path
        self.current_target_index = 0
        self.following_path = True
        print(f"Started path following with {len(path_points)} points")
        return True
    
    def stop_path_following(self):
        """Stop path following and apply brakes"""
        self.following_path = False
        self.path_points = []
        self.current_target_index = 0
        print("Stopped path following")
    
    def _find_carrot_point(self):
        """Find the carrot point ahead on the path"""
        if not self.path_points or self.current_target_index >= len(self.path_points):
            return None
            
        car_pos = (self.x, self.y)
        
        # Dynamic carrot distance based on current speed for smoother high-speed driving
        current_speed = abs(self.wheel_L + self.wheel_R) / 2
        dynamic_carrot_distance = self.carrot_distance + (current_speed * 10)  # Further ahead at higher speeds
        
        # Start from current target and look ahead
        for i in range(self.current_target_index, len(self.path_points)):
            point = self.path_points[i]
            distance = math.sqrt((point[0] - car_pos[0])**2 + (point[1] - car_pos[1])**2)
            
            # If this point is at the desired carrot distance or beyond
            if distance >= dynamic_carrot_distance:
                return point
                
        # If no point is far enough, return the last point
        return self.path_points[-1]
    
    def _update_target_index(self):
        """Update the current target index based on car position"""
        if not self.path_points:
            return
            
        car_pos = (self.x, self.y)
        
        # Check if we've reached the current target
        if self.current_target_index < len(self.path_points):
            target = self.path_points[self.current_target_index]
            distance = math.sqrt((target[0] - car_pos[0])**2 + (target[1] - car_pos[1])**2)
            
            if distance < self.arrival_threshold:
                self.current_target_index += 1
                # Only show progress every 5 waypoints to reduce spam
                if (self.current_target_index - 1) % 5 == 0 or self.current_target_index >= len(self.path_points) - 2:
                    print(f"Reached waypoint {self.current_target_index-1}, moving to next ({self.current_target_index}/{len(self.path_points)})")
                if self.current_target_index >= len(self.path_points):
                    # Reached the end of the path
                    print("Reached destination! Path following complete.")
                    self.stop_path_following()
                    return
    
    def _calculate_steering_command(self, target_point):
        """Calculate wheel commands to steer towards target point"""
        if not target_point:
            return 0, 0
            
        # Calculate angle to target
        dx = target_point[0] - self.x
        dy = target_point[1] - self.y
        # Fix coordinate system - in pygame Y increases downward, but car's physics 
        # expects negative Y for forward movement, so we need to flip dy
        target_angle = math.atan2(-dy, dx)
        
        # Calculate angle difference (shortest path)
        angle_diff = target_angle - self.angle
        while angle_diff > math.pi:
            angle_diff -= 2 * math.pi
        while angle_diff < -math.pi:
            angle_diff += 2 * math.pi
        
        # Calculate distance to target for speed control
        distance = math.sqrt(dx*dx + dy*dy)
        
        # PID-like control for steering
        max_turn_rate = 2.5
        turn_command = max(-max_turn_rate, min(max_turn_rate, angle_diff * 1.5))
        
        # Improved speed control - reward maintaining velocity
        base_speed = 0.5 
        
        # Reduce turn penalty to maintain speed through curves
        turn_penalty = abs(turn_command) * 0.05 
        
        # Velocity maintenance bonus - less aggressive slowdown
        current_velocity = abs(self.wheel_L + self.wheel_R) / 2
        velocity_bonus = min(0.2, current_velocity * 0.1)  # Reward for maintaining speed
        
        target_speed = max(0.2, base_speed + velocity_bonus - turn_penalty)
        
        # Only slow down significantly when very close to the FINAL destination
        if (distance < self.arrival_threshold * 1.5 and 
            self.current_target_index >= len(self.path_points) - 3):  # Only slow for last few waypoints
            target_speed *= 0.6
        
        # Calculate differential drive commands
        left_command = target_speed - turn_command * 0.4
        right_command = target_speed + turn_command * 0.4
        
        return left_command, right_command
    
    def _apply_wheel_commands(self, left_cmd, right_cmd, dt):
        """Apply wheel speed commands smoothly"""
        # Limit command values
        max_cmd = self.max_angular_wheelspeed
        left_cmd = max(-max_cmd, min(max_cmd, left_cmd))
        right_cmd = max(-max_cmd, min(max_cmd, right_cmd))
        
        # Smooth transition to target speeds
        speed_change_rate = self.wheel_acceleration * dt * 2  # Faster response for path following
        
        # Left wheel
        if abs(left_cmd - self.wheel_L) <= speed_change_rate:
            self.wheel_L = left_cmd
        elif left_cmd > self.wheel_L:
            self.wheel_L += speed_change_rate
        else:
            self.wheel_L -= speed_change_rate
            
        # Right wheel
        if abs(right_cmd - self.wheel_R) <= speed_change_rate:
            self.wheel_R = right_cmd
        elif right_cmd > self.wheel_R:
            self.wheel_R += speed_change_rate
        else:
            self.wheel_R -= speed_change_rate
    
    def update_path_following(self, dt):
        """Update path following behavior"""
        if not self.following_path:
            return
            
        # Update current target based on position
        self._update_target_index()
        
        if not self.following_path:  # Might have been stopped in _update_target_index
            return
            
        # Find carrot point
        carrot_point = self._find_carrot_point()
        
        if carrot_point:
            # Calculate steering commands
            left_cmd, right_cmd = self._calculate_steering_command(carrot_point)
            
            # Apply commands
            self._apply_wheel_commands(left_cmd, right_cmd, dt)
        else:
            # No valid carrot point, stop
            self.stop_path_following()
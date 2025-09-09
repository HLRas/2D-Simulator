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
        
        # Cross-track error following
        self.cross_track_following = False
        self.cross_track_path = []
        self.cross_track_index = 0
        self.lookahead_distance = 30  
        
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
        self.cross_track_following = False
        self.path_points = []
        self.cross_track_path = []
        self.current_target_index = 0
        self.cross_track_index = 0
        print("Stopped path following")
    
    def set_position(self, pos: tuple):
        """Set position of car"""
        self.x = pos[0]
        self.y = pos[1]
        return
    
    def set_orientation(self, orient: float):
        """Set orientation of car"""
        self.angle = orient
        return

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
        if self.following_path:
            self._update_carrot_following(dt)
        elif self.cross_track_following:
            self._update_cross_track_following(dt)
    
    def _update_carrot_following(self, dt):
        """Update carrot-stick path following"""
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
    
    def start_cross_track_following(self, path_points):
        """Start following a path using cross-track error method"""
        if len(path_points) < 2:
            return False
            
        self.cross_track_path = path_points[:]  # Copy the path
        self.cross_track_index = 0
        self.cross_track_following = True
        self.following_path = False  # Disable carrot following
        print(f"Started cross-track error following with {len(path_points)} points")
        return True
    
    def _update_cross_track_following(self, dt):
        """Update cross-track error path following"""
        if not self.cross_track_path:
            return
            
        # Find closest point on path
        closest_point, closest_index = self._find_closest_point_on_path()
        
        if closest_point is None:
            self.stop_path_following()
            return
            
        # Update current index
        self.cross_track_index = max(self.cross_track_index, closest_index)
        
        # Check if reached end
        if self.cross_track_index >= len(self.cross_track_path) - 1:
            distance_to_end = math.sqrt(
                (self.x - self.cross_track_path[-1][0])**2 + 
                (self.y - self.cross_track_path[-1][1])**2
            )
            if distance_to_end < self.arrival_threshold:
                print("Reached destination! Cross-track error following complete.")
                self.stop_path_following()
                return
        
        # Find lookahead point
        lookahead_point = self._find_lookahead_point()
        
        if lookahead_point:
            # Calculate cross-track error
            cross_track_error = self._calculate_cross_track_error(closest_point)
            
            # Calculate steering commands using cross-track error
            left_cmd, right_cmd = self._calculate_cross_track_steering(lookahead_point, cross_track_error)
            
            # Apply commands
            self._apply_wheel_commands(left_cmd, right_cmd, dt)
    
    def _find_closest_point_on_path(self):
        """Find the closest point on the path to the car"""
        if not self.cross_track_path:
            return None, 0
            
        car_pos = (self.x, self.y)
        min_distance = float('inf')
        closest_point = None
        closest_index = 0
        
        # Start searching from current index to avoid going backwards
        start_index = max(0, self.cross_track_index - 2)
        
        for i in range(start_index, len(self.cross_track_path)):
            point = self.cross_track_path[i]
            distance = math.sqrt((point[0] - car_pos[0])**2 + (point[1] - car_pos[1])**2)
            
            if distance < min_distance:
                min_distance = distance
                closest_point = point
                closest_index = i
                
        return closest_point, closest_index
    
    def _find_lookahead_point(self):
        """Find a point ahead on the path for steering"""
        if not self.cross_track_path:
            return None
            
        car_pos = (self.x, self.y)
        
        # Start from current index and look ahead
        for i in range(self.cross_track_index, len(self.cross_track_path)):
            point = self.cross_track_path[i]
            distance = math.sqrt((point[0] - car_pos[0])**2 + (point[1] - car_pos[1])**2)
            
            # If this point is at the desired lookahead distance or beyond
            if distance >= self.lookahead_distance:
                return point
                
        # If no point is far enough, return the last point
        return self.cross_track_path[-1]
    
    def _calculate_cross_track_error(self, closest_point):
        """Calculate the cross-track error (perpendicular distance from path)"""
        if not closest_point or self.cross_track_index >= len(self.cross_track_path) - 1:
            return 0
            
        # Get the path segment
        current_point = self.cross_track_path[self.cross_track_index]
        next_point = self.cross_track_path[min(self.cross_track_index + 1, len(self.cross_track_path) - 1)]
        
        # Calculate cross-track error using the perpendicular distance to the line segment
        # Vector from current to next point on path
        path_dx = next_point[0] - current_point[0]
        path_dy = next_point[1] - current_point[1]
        
        # Vector from current path point to car
        car_dx = self.x - current_point[0]
        car_dy = self.y - current_point[1]
        
        # Calculate cross product to get signed distance
        if abs(path_dx) < 1e-6 and abs(path_dy) < 1e-6:
            return 0  # Path segment too short
            
        # Normalize path vector
        path_length = math.sqrt(path_dx**2 + path_dy**2)
        path_dx /= path_length
        path_dy /= path_length
        
        # Cross-track error is the perpendicular component
        cross_track_error = car_dx * (-path_dy) + car_dy * path_dx
        
        return cross_track_error
    
    def _calculate_cross_track_steering(self, lookahead_point, cross_track_error):
        """Calculate steering commands using cross-track error and lookahead point"""
        if not lookahead_point:
            return 0, 0
            
        # PID gains for cross-track error correction
        kp_cross_track = 0.02  # Proportional gain for cross-track error
        
        # Calculate angle to lookahead point (same as carrot method)
        dx = lookahead_point[0] - self.x
        dy = lookahead_point[1] - self.y
        target_angle = math.atan2(-dy, dx)
        
        # Calculate angle difference
        angle_diff = target_angle - self.angle
        while angle_diff > math.pi:
            angle_diff -= 2 * math.pi
        while angle_diff < -math.pi:
            angle_diff += 2 * math.pi
        
        # Combine heading error and cross-track error
        max_turn_rate = 2.5
        heading_command = max(-max_turn_rate, min(max_turn_rate, angle_diff * 1.5))
        cross_track_command = -cross_track_error * kp_cross_track  # Negative to correct towards path
        
        total_turn_command = heading_command + cross_track_command
        total_turn_command = max(-max_turn_rate, min(max_turn_rate, total_turn_command))
        
        # Speed control (similar to carrot method but slightly slower for precision)
        base_speed = 0.4
        turn_penalty = abs(total_turn_command) * 0.08
        target_speed = max(0.15, base_speed - turn_penalty)
        
        # Calculate differential drive commands
        left_command = target_speed - total_turn_command * 0.4
        right_command = target_speed + total_turn_command * 0.4
        
        return left_command, right_command
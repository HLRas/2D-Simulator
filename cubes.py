import pygame
from config import *

class Cube:
    def __init__(self, row:int, col:int, type:int=0, squareSize:int=CUBE_SIZE) -> None:
        """Initialize the Cube object\n
        row : y (row) position (topleft) of cube
        col : x (col) position (topleft) of cube
        type : what type of cube (0=free, 1=barrier, 2=start, 3=end)
        """
        self.neighbours = []
        self.row = row
        self.col = col
        self.x = col * squareSize
        self.y = row * squareSize
        self.type = type if type in range(7) else 0
        self.squareSize = squareSize
        self.totRow = SCREEN_HEIGHT // self.squareSize
        self.totCol = SCREEN_WIDTH // self.squareSize
        self.set_color()

        self.rect = pygame.Rect(self.x, self.y, squareSize, squareSize)
        # Don't draw here - let the map handle drawing

    def set_color(self) -> None:
        """Set the color of the cube based on its type\n
        0 = Free\n
        1 = Barrier\n
        2 = Start\n
        3 = End\n
        4 = Closed\n
        5 = Open"""
        match self.type:
            case 0:
                self.color = WHITE
            case 1:
                self.color = BLACK
            case 2:
                self.color = ORANGE
            case 3:
                self.color = TURQUOISE
            case 4:
                self.color = RED
            case 5:
                self.color = GREEN
            case 6:
                self.color = PURPLE
            case _:
                self.color = WHITE
    
    def get_pos(self):
        return self.x,self.y

    def make_clear(self):
        self.change_type(0)

    def make_start(self):
        self.change_type(2)

    def make_closed(self):
        self.change_type(4)

    def make_open(self):
        self.change_type(5)

    def make_barrier(self):
        self.change_type(1)

    def make_end(self):
        self.change_type(3)

    def make_path(self):
        self.change_type(6)

    def make_occupied(self):
        """Make this cube red to indicate an occupied parking space entry"""
        self.change_type(4)  # Use type 4 (RED) for occupied parking entries

    def is_barrier(self):
        return self.type == 1

    def change_type(self, type:int) -> None:
        """Set the type of the cube and update its color\n
        type: The new type of the cube (0=free, 1=barrier, 2=start, 3=end, 4=closed, 5=open, 6=path)"""
        self.type = type if type in range(7) else 0
        self.set_color()
        # Don't auto-draw here - let the map handle drawing

    def update_neighbours(self, cubes):
        self.neighbours = []

        # DOWN
        if self.row < self.totRow - 1 and not cubes[self.row + 1][self.col].is_barrier():
            self.neighbours.append(cubes[self.row + 1][self.col])
        # UP
        if self.row > 0 and not cubes[self.row - 1][self.col].is_barrier():
            self.neighbours.append(cubes[self.row - 1][self.col])
        # RIGHT
        if self.col < self.totCol - 1 and not cubes[self.row][self.col + 1].is_barrier():
            self.neighbours.append(cubes[self.row][self.col + 1])
        # LEFT
        if self.col > 0 and not cubes[self.row][self.col - 1].is_barrier():
            self.neighbours.append(cubes[self.row][self.col - 1])

        # DOWN-RIGHT
        if (self.row < self.totRow - 1 and self.col < self.totCol - 1 
            and not cubes[self.row + 1][self.col + 1].is_barrier()):
            self.neighbours.append(cubes[self.row + 1][self.col + 1])

        # DOWN-LEFT
        if (self.row < self.totRow - 1 and self.col > 0 
            and not cubes[self.row + 1][self.col - 1].is_barrier()):
            self.neighbours.append(cubes[self.row + 1][self.col - 1])

        # UP-RIGHT
        if (self.row > 0 and self.col < self.totCol - 1 
            and not cubes[self.row - 1][self.col + 1].is_barrier()):
            self.neighbours.append(cubes[self.row - 1][self.col + 1])

        # UP-LEFT
        if (self.row > 0 and self.col > 0 
            and not cubes[self.row - 1][self.col - 1].is_barrier()):
            self.neighbours.append(cubes[self.row - 1][self.col - 1])

        
    def draw(self) -> None:
        """Draw the cube on the screen"""
        pygame.draw.rect(pygame.display.get_surface(), self.color, self.rect)
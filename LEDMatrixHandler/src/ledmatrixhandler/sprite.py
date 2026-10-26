from ledmatrix import Canvas
from .display import Display
from PIL import Image

import math 

from .sprite_vector import Vec2

class Sprite(Display):
    offset: Vec2 = Vec2.zero()
    offset_velocity: Vec2 = Vec2.zero()

    sprite_offset: Vec2 = Vec2.zero()
    sprite_bounds: Vec2 = Vec2.zero()

    def __init__(self, file_dir: str) -> None:

        try:
            with Image.open(file_dir) as img: 
                self.__img = img
                self.__data = img.getdata()
                print("loaded image", file_dir)
        except:
            print("couldn't find image: ", file_dir)

    def update(self, dt: float) -> None:
        pass

        # self.offset += self.offset_velocity * dt

    def render(self, canvas: Canvas) -> None:
        for y in range(math.floor(self.offset.y), self.__img.height):
            for x in range(math.floor(self.offset.x), self.__img.width):
                canvas.set_pixel(x, y, self.__img.getpixel((x, y)))

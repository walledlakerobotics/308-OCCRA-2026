from ledmatrix import Canvas
from .display import Display
from PIL import Image
import json

# import math

# from .sprite_vector import Vec2

class Sprite(Display):

    current_animation: str = "idle"
    animated: bool = False

    def __init__(self, file_dir: str) -> None:

        self.__file_dir = file_dir

        try:
            with Image.open(file_dir) as img: 
                self.__img = img
                self.__img.load()
                print("loaded image", file_dir)

        except:
            print("couldn't find image: ", file_dir)

    def update(self, dt: float) -> None:
        pass

    def render(self, canvas: Canvas) -> None:

        offset_data = (0, 0, self.__img.width, self.__img.height)

        # if self.animated:
        #     offset_data = self.update_frame_offset(self.current_animation, self.frames)

        sprite_offx = offset_data[0]
        sprite_offy = offset_data[1]

        width = offset_data[2]
        height = offset_data[3]

        for y in range(width):
            for x in range(height):
                pixel = self.__img.getpixel((x + sprite_offx, y + sprite_offy))

                if pixel is None:
                    continue

                canvas.set_pixel(
                    x,
                    y,
                    pixel
                )

    # def update_frame_offset(self, animation_name: str) -> tuple:
    #     json_path: str = self.__file_dir.replace(".png", ".json")

    #     with open(json_path) as file:
    #         data = json.load(file)

    #     frames = data["frames"]
    #     tags = data["meta"]["frameTags"]

from ledmatrix import Canvas
from .display import Display
from PIL import Image
import json

import math
# from .sprite_vector import Vec2

class Sprite(Display):

    current_animation: str = "idle"
    animated: bool = True

    offset = (0.0, 0.0)

    def __init__(self, file_dir: str, animated) -> None:

        self.__file_dir = file_dir
        self.animated = animated

        try:
            with Image.open(file_dir) as img: 
                self.__img = img
                self.__img.load()
                print("loaded image", file_dir)

        except:
            print("couldn't find image: ", file_dir)

        if self.animated: 
            try:
                json_path: str = self.__file_dir.replace(".png", ".json")

                with open(json_path) as file:
                    self.data = json.load(file)
                    print("loaded json data", json_path)

            except:
                print("cannot load json data: ", json_path)

    def update(self, dt: float) -> None:
        pass
        # self.update_frame_offset(self.current_animation)

    def render(self, canvas: Canvas) -> None:

        offset_data = (0, 0, self.__img.width, self.__img.height)

        self.get_offset_data()

        sprite_offx = offset_data[0]
        sprite_offy = offset_data[1]

        width = offset_data[2]
        height = offset_data[3]

        for y in range(width):
            for x in range(height):

                offx = math.floor(x + self.offset[0])
                offy = math.floor(y + self.offset[1])

                if offx >= canvas.width or offy >= canvas.height:
                    continue

                pixel = self.__img.getpixel((x + sprite_offx, y + sprite_offy))

                if pixel is None:
                    continue

                canvas.set_pixel(
                    offx,
                    offy,
                    pixel,
                )

    def set_animation(self, animation: str): 
        self.animation = animation

    def get_offset_data(self): 
        frame_name = self.__file_dir.removeprefix("assets/")

        frames = self.data["frames"]
        tags = self.data["meta"]["frameTags"]

        frame_tags = tags[0]

        frame_from = frame_tags["from"]
        frame_to = frame_tags["to"]

        frame_data = []

        for n in range(frame_from, frame_to): 
            frame = frames[f"{frame_name} {n}"]

            print(frame)

            # x = frame["x"]
            # y = frame["y"]
            # w = frame["w"]
            # h = frame["h"]

            # frame_data.append((x, y, w, h))

        return frame_data

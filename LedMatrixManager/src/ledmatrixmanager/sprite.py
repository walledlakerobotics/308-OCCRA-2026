from .display import Display
from PIL import Image
import json

from typing import Callable

import math
# from .sprite_vector import Vec2

class Sprite(Display):
    __current_animation: str = "idle"
    animated: bool = True

    __offset = (0.0, 0.0)

    def __init__(self, file_dir: str, animated) -> None:

        self.__file_dir = file_dir
        self.animated = animated

        with Image.open(file_dir) as img:
            self.__img = img.copy()

        print("loaded image", file_dir)
        self.__offset_sprite_data = (0, 0, self.__img.width, self.__img.height)
        self.__offset_data_producer = lambda : self.__offset_sprite_data

        if self.animated: 
            try:
                json_path: str = self.__file_dir.replace(".png", ".json")

                with open(json_path) as file:
                    self.data = json.load(file)
                    print("loaded json data", json_path)

            except:
                print("cannot load json data: ", json_path)

    # needs to add Anti-aliasing
    def render(self, canvas) -> None:

        self.__offset_sprite_data = self.__offset_data_producer()
        
        sprite_offx = self.__offset_sprite_data[0]
        sprite_offy = self.__offset_sprite_data[1]

        width = self.__offset_sprite_data[2]
        height = self.__offset_sprite_data[3]

        for y in range(height):
            for x in range(width):
                offx = math.floor(x + self.__offset[0])
                offy = math.floor(y + self.__offset[1])

                if offx >= canvas.width or offy >= canvas.height:
                    continue

                pixel = self.__img.getpixel((x + sprite_offx, y + sprite_offy))

                if pixel is None:
                    continue

                canvas.SetPixel(offx, offy, pixel[0], pixel[1], pixel[2])                

    def set_animation(self, animation: str): 
        self.__current_animation = animation

    def set_offset(self, offset: tuple[int, int]):
        self.__offset = offset

    def set_offset_data(self, data: Callable[[], tuple[int, int, int, int]]):
        self.__offset_data_producer = data


    def parse_offset_data(self) -> list[tuple[int, int, int, int]]:
        frame_name = self.__file_dir.removeprefix("assets/")

        frames = self.data["frames"]
        tags = self.data["meta"]["frameTags"]

        # needs to get the index of the name
        frame_tags = tags[self.__get_tag_index()]

        frame_from = frame_tags["from"]
        frame_to = frame_tags["to"]

        frame_data = []

        for n in range(frame_from, frame_to + 1):

            current_frame_name = frame_name + " " + str(n)

            try: 
                frame = frames[current_frame_name]

                f = frame["frame"]

                x = f["x"]
                y = f["y"]
                w = f["w"]
                h = f["h"]

                frame_data.append((x, y, w, h))
            except: 
                print("error loading frame: ", current_frame_name)

        return frame_data

    def __get_tag_index(self) -> int | None:

        tags = self.data["meta"]["frameTags"]

        for i in range(len(tags)):
            if tags[i]["name"] == self.__current_animation:
                return i

        return None

class Animation(Display):

    def __init__(self, sprite: Sprite) -> None:
        self.sprite = sprite
        self.__offset_sprite_data_arr = sprite.parse_offset_data()

        if not self.__offset_sprite_data_arr:
            raise ValueError("Sprite animation has no frames")

        self.current_frame = 0
        self.playing = True
        self.__set_current_frame()

    def __set_current_frame(self) -> None:
        self.__current_offset = self.__offset_sprite_data_arr[self.current_frame]
        self.sprite.set_offset_data(lambda: self.__current_offset)

    def next_frame(self):
        self.current_frame = (self.current_frame + 1) % len(self.__offset_sprite_data_arr)
        self.__set_current_frame()

    def prev_frame(self):
        self.current_frame = (self.current_frame - 1) % len(self.__offset_sprite_data_arr)
        self.__set_current_frame()

    def set_animation(self, animation: str):
        self.sprite.set_animation(animation)

        self.__offset_sprite_data_arr = self.sprite.parse_offset_data()
        if not self.__offset_sprite_data_arr:
            raise ValueError("Sprite animation has no frames")

        self.current_frame = 0
        self.__set_current_frame()

    def update(self, _: float) -> None:
        if self.playing:
            self.next_frame()
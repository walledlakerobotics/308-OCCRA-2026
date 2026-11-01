try: 
    from rgbmatrix import FrameCanvas as Canvas  # type: ignore
except: 
    from RGBMatrixEmulator.emulation.canvas import Canvas

from networktables import NetworkTablesInstance

from .main_loop import MainLoop
from .sprite import Sprite, Animation 
from .display import Display

import math


def main() -> None:

    loop = MainLoop(12, 64, 32, 80)

    sprite = Sprite("assets/eye_source-Sheet.png", True)
    ani = Animation(sprite)

    ani.set_animation("blink")

    loop.add(sprite)
    loop.add(ani)

    loop.add(Spin())

    loop.start()

class Spin(Display):

    def __init__(self) -> None:

        self.theata = 0

    def start(self, nt_inst: NetworkTablesInstance):
        pass

    def update(self, dt: float) -> None:
        pass
        
    def draw(self, canvas: Canvas) -> None:

        for i in range(360):
            canvas.SetPixel(
                math.floor(32 + math.cos(i) * 20),
                math.floor(15 + math.sin(i) * 5),
                math.floor(255 * (i / 360)),
                math.floor(255 * (i / 700)),
                math.floor(255 * (i / 400)),
            )

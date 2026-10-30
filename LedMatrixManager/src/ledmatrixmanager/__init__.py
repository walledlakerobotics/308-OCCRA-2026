from .main_loop import MainLoop
from .sprite import Sprite, Animation 
from .display import Display


def main() -> None:

    loop = MainLoop(30, 64, 32)

    sprite = Sprite("assets/eye_source-Sheet.png", True)
    ani = Animation(sprite)

    ani.set_animation("blink")

    loop.add(sprite)
    loop.add(ani)

    loop.start()

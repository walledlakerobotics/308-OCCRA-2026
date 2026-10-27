from typing import Any, cast

from .main_loop import MainLoop
from .sprite import Sprite, Animation


def main() -> None:
    loop = MainLoop(30, 64, 32)

    loop.add(lambda: cast(Any, create_thing()))

    loop.start()

def create_thing(): 
    sprite = Sprite("assets/eye_source-Sheet.png", True)
    animation = Animation(sprite)

    sprite.add(animation)

    return [sprite]

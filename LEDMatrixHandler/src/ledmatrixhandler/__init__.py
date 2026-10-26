from .main_loop import MainLoop
from .sprite import Sprite
from .sprite_vector import Vec2

def main() -> None:
    loop = MainLoop(30)

    sprite = Sprite("assets/eye_test.png")

    loop.add(sprite)
    
    loop.start()

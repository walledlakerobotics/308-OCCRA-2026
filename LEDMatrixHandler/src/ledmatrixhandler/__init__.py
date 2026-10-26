from .main_loop import MainLoop

def main() -> None:
    loop = MainLoop(30)

    loop.start()

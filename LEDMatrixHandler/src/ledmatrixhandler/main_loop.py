from .display import Display

from typing import Callable 
import time
from ledmatrix import open_device, Canvas, MatrixGeometry

class MainLoop:

    def __init__(self, fps: float, width: int, height: int): 
        self.__running: bool = False
        self.__displays: list[Display] = list()

        self.__frames_per_second: float = fps
        self.__last_time: float = 0.0

        # matrix things
        self.__canvas: Canvas = Canvas(MatrixGeometry(width, height))

        try: 
            with open_device() as device: 
                device.set_fps(fps)
                device.set_brightness(50)

                self.__device = device
        except: 
            self.__testing = True
            print("opening sim...")

            # pygame stuff

            print("loaded pygame!")

    def __update(self, dt: float) -> None: 
        for m in self.__displays:
            m.update(dt)

    def __render(self) -> None: 
        self.__process_render()

        if self.__device.is_open:
            self.__device.show_frame(self.__canvas)

    def __sim_render(self) -> None:
        self.__process_render()

        # pygame buffer read

    def __process_render(self) -> None: 
        self.__canvas.clear()

        self.__displays.sort(key=lambda a : a.z_index)

        for d in self.__displays: 
            d.render(self.__canvas)

    def start(self) -> None: 
        self.__running = True

        self.__loop()

    def __loop(self) -> None:
        while self.__running: 

            start_time: float = time.perf_counter()
            delta_time: float = start_time - self.__last_time
            self.__last_time = start_time

            self.__update(delta_time)

            if not self.__testing:
                self.__render()
            else: 
                self.__sim_render()

            target_frame_time: float = 1.0 / self.__frames_per_second
            elapsed_time: float = time.perf_counter() - start_time
            remaining_time: float = target_frame_time - elapsed_time

            if remaining_time < self.__frames_per_second:
                time.sleep(remaining_time)

    def stop(self) -> None: 
        self.__running = False

    def add(self, display: Callable[[], list[Display]]):
        self.__displays.extend(display())

    def remove(self, tag: str):
        for d in self.__displays: 
            if d.tag == tag: 
                self.__displays.remove(d)

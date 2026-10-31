try:
    from rgbmatrix import RGBMatrix, RGBMatrixOptions  # type: ignore
    print("Using real hardware library")
except ImportError:
    from RGBMatrixEmulator import RGBMatrix, RGBMatrixOptions
    print("Using sim library")

from .display import Display
import time


class MainLoop:
    def __init__(self, fps: float, width: int, height: int, brightness: int):
        self.__running = False
        self.__last_time = 0.0
        self.__frame_per_second = fps

        self.__displays: list[Display] = list()

        try:
            self.__options = RGBMatrixOptions()

            self.__options.brightness = brightness
            self.__options.rows = height
            self.__options.cols = width
            self.__options.hardware_mapping = "adafruit-hat"
            self.__options.chain_length = 1 # one panel
            self.__options.parallel = 1
            self.__options.gpio_slowdown = 4

            self.__led_matrix = RGBMatrix(self.__options)
            self.__canvas = self.__led_matrix.CreateFrameCanvas()
        except:
            print("instance error")

    def __update(self, dt: float) -> None:
        for m in self.__displays:
            m.update(dt)

    def __render(self) -> None:
        self.__canvas.Clear()

        for d in self.__displays:
            d.draw(self.__canvas)

        self.__canvas = self.__led_matrix.SwapOnVSync(self.__canvas)

    def __loop(self):
        while self.__running:
            start_time: float = time.perf_counter()
            delta_time: float = start_time - self.__last_time
            self.__last_time = start_time

            self.__update(delta_time)

            self.__render()

            target_frame_time: float = 1.0 / self.__frame_per_second
            elapsed_time: float = time.perf_counter() - start_time
            remaining_time: float = target_frame_time - elapsed_time

            if remaining_time > 0:
                time.sleep(remaining_time)

    def start(self):
        self.__running = True
        self.__last_time = time.perf_counter()

        self.__loop()

    def stop(self):
        self.__running = False

    def add(self, display):
        self.__displays.append(display)

    def remove(self, tag):
        for d in self.__displays:
            if d.tag == tag:
                self.__displays.remove(d)

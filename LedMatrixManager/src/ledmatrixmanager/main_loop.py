try:
    from rgbmatrix import RGBMatrix, RGBMatrixOptions
except ImportError:
    from RGBMatrixEmulator import RGBMatrix, RGBMatrixOptions
    
from .display import Display
import time
class MainLoop:

    def __init__(self, fps: float, width: int, height: int): 
            self.__frame_per_second = fps
            
            self.__displays: list[Display] = list()
            self.__options = RGBMatrixOptions()
    
            self.__options.brightness = 50
            self.__options.rows = height
            self.__options.cols = width
    
            self.__led_matrix = RGBMatrix(self.__options)
            self.__canvas = self.__led_matrix.CreateFrameCanvas()
    
    def __update(self, dt: float) -> None: 
        for m in self.__displays:
            m.update(dt)

    def __render(self) -> None: 
        self.__canvas.Clear()    

        for d in self.__displays: 
            d.render(self.__canvas)

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

            if remaining_time < self.__frame_per_second and not remaining_time < 0:
                time.sleep(remaining_time)


    def start(self): 
        self.__running = True

        self.__loop()

    def stop(self):
        self.__running = False

    def add(self, display):
        self.__displays.append(display)

    def remove(self, tag):
        for d in self.__displays: 
            if d.tag == tag: 
                self.__displays.remove(d)
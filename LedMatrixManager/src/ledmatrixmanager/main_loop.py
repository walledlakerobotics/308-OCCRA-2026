try:
    from rgbmatrix import RGBMatrix, RGBMatrixOptions
except ImportError:
    from RGBMatrixEmulator import RGBMatrix, RGBMatrixOptions
    
from .loop import Loop
from .display import Display

class MainLoop(Loop):

    def __init__(self, fps: float, width: int, height: int): 
            self.__displays: list[Display] = list()
            self.__options = RGBMatrixOptions()
    
            self.__options.brightness = 50.0
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

    def add(self, display):
        self.__displays.append(display)

    def remove(self, tag):
        for d in self.__displays: 
            if d.tag == tag: 
                self.__displays.remove(d)
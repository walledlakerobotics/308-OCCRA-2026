try: 
    from rgbmatrix import FrameCanvas as Canvas
except: 
    from RGBMatrixEmulator.emulation.canvas import Canvas

from abc import abstractmethod
import uuid

class Display: 

    def __init__(self) -> None:
        self.tag: str = str(uuid.uuid4())

    @abstractmethod
    def update(self, dt: float) -> None:
        pass
        
    @abstractmethod
    def render(self, canvas: Canvas) -> None:
        pass

    def get_tag(self) -> str:
        return self.tag

    def set_tag(self, tag: str) -> None:
        self.tag = tag

    def set_z_index(self, z_index: int) -> None:
        self.z_index = z_index

    def get_z_index(self) -> int:
        return self.z_index
from abc import abstractmethod
from ledmatrix import Canvas
import uuid

class Display: 

    def __init__(self) -> None:
        self.tag: str = str(uuid.uuid4())

    @abstractmethod
    def update(self, dt: float) -> None:
        for c in getattr(self, "children", []):
            c.update(dt)
        
    @abstractmethod
    def render(self, canvas: Canvas) -> None:
        for c in getattr(self, "children", []):
            c.render(canvas)

    def get_tag(self) -> str:
        return self.tag

    def set_tag(self, tag: str) -> None:
        self.tag = tag

    def set_z_index(self, z_index: int) -> None:
        self.z_index = z_index

    def get_z_index(self) -> int:
        return self.z_index

    def add(self, display: Display) -> None:
        if not hasattr(self, "children"):
            self.children: list[Display] = list()

        self.children.append(display)

    def remove(self, display: Display) -> None:
        if not hasattr(self, "children"):
            return

        self.children.remove(display)

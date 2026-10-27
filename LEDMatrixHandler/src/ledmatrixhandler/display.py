from abc import abstractmethod
from ledmatrix import Canvas

import uuid
from typing import Optional

class Display: 

    tag: str = str(uuid.uuid4())
    z_index: int = 0

    def __init__(self, tag: Optional[str]) -> None:
        if tag != None: 
            self.tag = tag
            
    @abstractmethod
    def update(self, dt: float) -> None:
        pass

    @abstractmethod
    def render(self, canvas: Canvas) -> None:
        pass

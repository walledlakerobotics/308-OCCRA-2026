from abc import abstractmethod
import time
from .display import Display

class Loop: 

    __running = False

    @abstractmethod
    def __init__(self, fps: float, width: int, height: int):
        self.__frames_per_second = fps

    @abstractmethod
    def __update(self, dt: float) -> None: 
        pass

    @abstractmethod
    def __render(self) -> None: 
        pass 

    def __loop(self):
        while self.__running: 
            pass
            # start_time: float = time.perf_counter()
            # delta_time: float = start_time - self.__last_time
            # self.__last_time = start_time

            # self.__update(delta_time)

            # self.__render()

            # target_frame_time: float = 1.0 / self.__frames_per_second
            # elapsed_time: float = time.perf_counter() - start_time
            # remaining_time: float = target_frame_time - elapsed_time

            # if remaining_time < self.__frames_per_second and not remaining_time < 0:
            #     time.sleep(remaining_time)
        
    def start(self): 
        self.__running = True

        self.__loop()

    def stop(self):
        self.__running = False

    @abstractmethod
    def add(self, display: Display):
        pass

    @abstractmethod
    def remove(self, tag: str):
        pass
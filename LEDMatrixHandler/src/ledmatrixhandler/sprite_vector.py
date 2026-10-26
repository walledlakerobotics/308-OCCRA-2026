import math

class Vec2:

    x: float = 0.0
    y: float = 0.0

    def __init__(self, x: float, y: float) -> None:
        self.x = x
        self.y = y

    def length(self) -> float: 
        return math.hypot(self.x, self.y)

    def length_to(self, other) -> float: 
        delta_vector = other - self

        return math.hypot(delta_vector.x, delta_vector.y)

    def normalize(self) -> Vec2: 
        return Vec2(self.x / self.length(), self.y / self.length())

    def dot(self, other) -> float:
        return (self.x * other.x) + (self.y * other.y)

    @staticmethod
    def zero() -> Vec2: 
        return Vec2(0, 0)

    @staticmethod
    def left() -> Vec2: 
        return Vec2(-1, 0)

    @staticmethod
    def right() -> Vec2: 
        return Vec2(1, 0)

    @staticmethod
    def up() -> Vec2: 
        return Vec2(0, -1)

    @staticmethod
    def down() -> Vec2: 
        return Vec2(0, 1)

    def __add__(self, other):
        return Vec2(self.x + other.x, self.y + other.y)

    def __sub__(self, other):
        return Vec2(self.x - other.x, self.y - other.y)

    def __mul__(self, other):
        return Vec2(self.x * other.x, self.y * other.y)

    def __divmod__(self, other):
        return Vec2(self.x / other.x, self.y / other.y)

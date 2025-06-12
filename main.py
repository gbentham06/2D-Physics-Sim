import math

class vector:
    def __init__(self, x, y):
        self.x = x
        self.y = y
        self.scal = math.sqrt(x**2 + y**2)
    
    def __add__(self, other):
        return vector(self.x + other.x, self.y + other.y)
    
    def __str__(self):
        return f"{self.x, self.y}"

class ball:
    def __init__(self, position, velocity, radius=1, colour="blue", mass=1, bounce=0):
        self.position = vector(position[0],position[1])
        self.velocity = vector(velocity[0],velocity[1])
        self.radius = radius
        self.colour = colour
        self.mass = mass
        self.bounce = bounce

    def onfloor(self):
        return self.position.y - self.radius == 0
    
    def hittingfloor(self):
        return self.position.y - self.radius + self.velocity.y - 9.81 <= 0
            

    def update(self):
        
        if self.onfloor() or self.hittingfloor():
            self.velocity.y = abs(self.velocity.y) * self.bounce
            self.position.y = 0 + self.radius
        else:
            self.velocity.y -= 9.81
            

        self.position += self.velocity

obj = ball([0,100],[4,5], bounce=0)

for i in range(100):
    print(obj.position, obj.velocity)
    obj.update()




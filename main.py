import random as r
import numpy as np
import pygame as py
import time

py.init()
window = py.display.set_mode((1000,1000))
frame_time = 1/200


class ball:
    def __init__(self, pos, vel=[0,0], acc=[0,0], radius=10, mass=1, elastic=0.5, friction=0, color=(0, 0, 255)):
        self.pos = np.array(pos, dtype=float)
        self.vel = np.array(vel, dtype=float)
        self.acc = np.array(acc, dtype=float)
        self.radius = radius
        self.mass = mass
        self.elastic = elastic
        self.color = color

    def get_net_force(self):
        self.net_force = self.acc * self.mass
        self.net_force[1] -= 9.81


def update_physics(screen, objs, dt=0.01):
    for i,obj in enumerate(objs):
        obj.get_net_force()
        obj.acc = obj.net_force / obj.mass
        obj.vel += obj.acc * dt
        obj.pos += obj.vel * dt

                # Left or right wall
        if obj.pos[0] - obj.radius < 0:
            obj.pos[0] = obj.radius  # This is a "snap" or teleport
            obj.vel[0] *= -obj.elastic
        elif obj.pos[0] + obj.radius > 1000:
            obj.pos[0] = 1000 - obj.radius
            obj.vel[0] *= -obj.elastic

        # Top or bottom wall
        if obj.pos[1] - obj.radius < 0:
            obj.pos[1] = obj.radius
            obj.vel[1] *= -obj.elastic
        elif obj.pos[1] + obj.radius > 1000:
            obj.pos[1] = 1000 - obj.radius
            obj.vel[1] *= -obj.elastic

        # ball-collide
        if len(objs) > 1:
            for j,obj2 in enumerate(objs):

                dist_vec = obj.pos - obj2.pos
                dist = np.linalg.norm(dist_vec)

                if dist < obj.radius + obj2.radius:

                    overlap = obj.radius + obj2.radius - dist
                    
                    if dist == 0:
                        dist = 1e-6 # Avoid division by zero with a tiny offset
                        n = np.array([1, 0]) # Default to horizontal separation
                    else:
                        n = dist_vec / dist # Normalized collision normal
                        
                    obj.pos -= n * overlap * 0.5
                    obj2.pos += n * overlap * 0.5

                    # Relative velocity
                    v_rel = obj2.vel - obj.vel

                    # Impulse scalar with elasticity
                    j = -(1 + obj.elastic) * np.dot(v_rel, n) * (obj.mass * obj2.mass) / (obj.mass + obj2.mass)

                    # Apply impulse
                    obj.vel -= (j / obj.mass) * n
                    obj2.vel += (j / obj2.mass) * n
                    
        py.draw.circle(screen, obj.color, (obj.pos[0], 1000-obj.pos[1]), obj.radius, 20)
        
 

objects = [ball([r.randint(50, 950), r.randint(50, 950)], vel=[r.randint(50, 950),r.randint(50, 950)], elastic=1) for i in range(10)]

while True:
    for event in py.event.get():
        if event.type == py.QUIT:
            py.quit()

    window.fill('black')
    update_physics(window, objects, dt=frame_time)
    py.display.update()

    time.sleep(frame_time)

import random
import numpy as np

class ball:
    def __init__(self, pos, vel=[0,0], acc=[0,0], radius=1, mass=1, elastic=0, friction=0):
        self.pos = np.array(pos)
        self.velocity = np.array(vel)
        self.acc = np.array(acc)
        self.radius = radius
        self.mass = mass
        self.elastic = elastic

    def get_net_force(self):
        self.net_force = self.acc * self.mass
        self.net_force.y -= 9.81


def update_physics(objs, dt=1):
    for i,obj in enumerate(objs):
        obj.acc / obj.mass
        obj.vel += obj.acc * dt
        obj.pos += obj.vel * dt

        if obj.pos[0] - obj.radius < 0 or obj.pos[0] + obj.radius > 300:
            obj.vel[0] *= -obj.elastic # v-collide
        if obj.pos[1] - obj.radius < 0 or obj.pos[1] + obj.radius > 300:
            obj.vel[1] *= -obj.elastic # h-collide

        # ball-collide
        for j,obj2 in enumerate(objs):
            if j > i: # avoids redundancy and self colision
                if np.hypot(obj.pos[0] - obj2.pos[0], obj.pos[1] - obj2.pos[1]) < obj.radius + obj2.radius:
                    n = (obj2.pos - obj.pos) / np.hypot(obj2.pos[0] - obj.pos[0], obj2.pos[1] - obj.pos[1])
                    v_rel = obj2.vel - obj.vel
                    v_n = v_rel * n
                    impulse = -(1 + obj.elastic) * v_n
                    obj.vel -= (impulse/obj.mass) * n
                    obj2.vel = (impulse/obj2.mass) * n



objects = [ball([random.randint(50, 250), random.randint(50, 250)],
                velocity=[random.randint(50, 250), random.randint(50, 250)]),
            ball([random.randint(50, 250), random.randint(50, 250)],
                velocity=[random.randint(50, 250), random.randint(50, 250)]),
            ball([random.randint(50, 250), random.randint(50, 250)],
                velocity=[random.randint(50, 250), random.randint(50, 250)]),
                ]

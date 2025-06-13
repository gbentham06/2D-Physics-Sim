import random as r
import numpy as np

class ball:
    def __init__(self, pos, vel=[0,0], acc=[0,0], radius=1, mass=1, elastic=0, friction=0):
        self.pos = np.array(pos)
        self.vel = np.array(vel)
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

            # Left or right wall
	    if obj.pos[0] - obj.radius < 0:
            obj.pos[0] = obj.radius
            obj.vel[0] = -obj.vel[0] * obj.elastic
        elif obj.pos[0] + obj.radius > 1000:
            obj.pos[0] = 1000 - obj.radius
            obj.vel[0] = -obj.vel[0] * obj.elastic

        # Top or bottom wall
        if obj.pos[1] - obj.radius < 0:
            obj.pos[1] = obj.radius
            obj.vel[1] = -obj.vel[1] * obj.elastic
        elif obj.pos[1] + obj.radius > 1000:
            obj.pos[1] = 1000 - obj.radius
            obj.vel[1] = -obj.vel[1] * obj.elastic

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

rand = r.randint(50, 950)

objects = [ball([rand, rand], vel=[rand, rand])]

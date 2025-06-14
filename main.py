import random as r
import numpy as np
import pygame as py
import time

running = True
py.init()
window = py.display.set_mode((1000,1000))
frame_time = 0.01
playback_speed = 1
runtime = 0


class ball:
    def __init__(self, pos, vel=[0,0], radius=10, color=(0, 0, 255), elastic=0.7, friction=0.9, density=1):
        self.pos = np.array(pos, dtype=float)
        self.vel = np.array(vel, dtype=float)
        self.acc = np.array([0,0], dtype=float)
        self.radius = radius
        self.mass = np.pi * self.radius**2 * density
        self.color = color
        self.elastic = elastic
        self.friction = friction

def border_collisions(obj, width, height):

    # walls
    if obj.pos[0] < obj.radius:
        obj.pos[0] = obj.radius  
        obj.vel[0] *= -obj.elastic
        obj.vel[1] *= obj.friction
    elif obj.pos[0] > width - obj.radius:
        obj.pos[0] = width - obj.radius
        obj.vel[0] *= -obj.elastic
        obj.vel[1] *= obj.friction

    # Ceiling & Floor
    if obj.pos[1] < obj.radius:
        obj.pos[1] = obj.radius
        obj.vel[1] *= -obj.elastic
        obj.vel[0] *= obj.friction
    elif obj.pos[1] > height - obj.radius:
        obj.pos[1] = height - obj.radius
        obj.vel[1] *= -obj.elastic
        obj.vel[0] *= obj.friction
        
        # elegant stopping
        if np.linalg.norm(obj.vel) < 20:
            obj.vel = np.array([0.0, 0.0])
        
    
def ball_collisions(objects):
    for i in range(len(objects)):
        for j in range(i + 1, len(objects)): # removes redundancy
            obj1 = objects[i]
            obj2 = objects[j]

            dist_vec = obj1.pos - obj2.pos
            dist = np.linalg.norm(dist_vec)
            min_dist = obj1.radius + obj2.radius

            if dist < min_dist:
                overlap = min_dist - dist
                
                if dist == 0:
                    n = np.array([1, 0]) # Avoid division by zero
                else:
                    n = dist_vec / dist # normalised vector
                
                obj1.pos += n * overlap * 0.5
                obj2.pos -= n * overlap * 0.5
                v_rel = obj1.vel - obj2.vel
                
                # impulse scalar
                e = (obj1.elastic + obj2.elastic) / 2
                j = -(1 + e) * np.dot(v_rel, n)
                if obj1.mass + obj2.mass > 0: # Avoid division by zero
                    j /= (1 / obj1.mass) + (1 / obj2.mass)
                else:
                    j = 0
                
                obj1.vel += (j / obj1.mass) * n
                obj2.vel -= (j / obj2.mass) * n

                if np.linalg.norm(obj1.vel) < 20:
                    obj1.vel = np.array([0.0, 0.0])


def force(obj, drag_coef=0.5):
    gravity = np.array([0.0, 981.0]) * obj.mass

    drag_force = -drag_coef * obj.vel

    total_force = gravity + drag_force

    if obj.mass > 0:
        obj.acc = total_force / obj.mass


def update_physics(obj, dt):
    prev_acc = obj.acc
    obj.pos += obj.vel * dt + 0.5 * prev_acc * (dt**2)

    force(obj)
    nxt_acc = obj.acc
    obj.vel += 0.5 * (prev_acc + nxt_acc) * dt


def r_colour():
    return (r.randint(0,255), r.randint(0,255), r.randint(0,255))


objects = [ball([r.randint(50,950), r.randint(50,950)], vel=[r.randint(-950,950), r.randint(-950,950)], color=r_colour())
           for i in range(50)]

while running:
    for event in py.event.get():
        if event.type == py.QUIT:
            running = False
            py.quit()

    # physics
    for object in objects:
        update_physics(object, frame_time*playback_speed)
        border_collisions(object, 1000, 1000)
    ball_collisions(objects)

    # display
    window.fill('black')
    for object in objects:
        py.draw.circle(window, object.color, object.pos, object.radius)
    py.display.update()

    time.sleep(frame_time)
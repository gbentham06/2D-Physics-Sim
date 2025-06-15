import random as r
import numpy as np
import pygame as py

# setup
running = True
py.init()
window_width, window_height = 1200, 1000
window = py.display.set_mode((window_width, window_height))
sim_width = 1000
fps = 100
playback_speed = 1
runtime = 0
clock = py.time.Clock()
font = py.font.Font(None, 32)
small_font = py.font.Font(None, 24)
py.display.set_caption("2D Physics Sim")


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


class TextBox:
    def __init__(self, x, y, w, h, text=''):
        self.rect = py.Rect(x, y, w, h)
        self.color_inactive = py.Color('gray30')
        self.color_active = py.Color('purple3')
        self.color = self.color_inactive
        self.text = text
        self.active = False

    def handle_event(self, event):
        if event.type == py.MOUSEBUTTONDOWN:
            if self.rect.collidepoint(event.pos):
                self.active = not self.active
            else:
                self.active = False
            self.color = self.color_active if self.active else self.color_inactive
        if event.type == py.KEYDOWN:
            if self.active:
                if event.key == py.K_BACKSPACE:
                    self.text = self.text[:-1]
                elif (event.unicode.isdigit() or
                      (event.unicode == '.' and '.' not in self.text) or
                      (event.unicode == '-' and len(self.text) == 0)):
                    self.text += event.unicode

    def draw(self, screen):
        txt_surface = font.render(self.text, True, (255, 255, 255))
        screen.blit(txt_surface, (self.rect.x + 5, self.rect.y + 5))
        py.draw.rect(screen, self.color, self.rect, 2)

class CheckBox:
    def __init__(self, x, y, w, h):
        self.rect = py.Rect(x, y, w, h)
        self.color = py.Color('gray30')
        self.checked_color = py.Color('purple3')
        self.checked = False

    def handle_event(self, event):
        if event.type == py.MOUSEBUTTONDOWN:
            if self.rect.collidepoint(event.pos):
                self.checked = not self.checked

    def draw(self, screen):
        py.draw.rect(screen, self.color, self.rect, 2)
        if self.checked:
            py.draw.rect(screen, self.checked_color, self.rect.inflate(-8, -8))

    
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


# ball list
objects = []

# UI setup :(
param_box_x = sim_width + 10
y_pos = 50
spacing = 80
textbox_w = 120
checkbox_w = 24
checkbox_offset = textbox_w + 15

ui_elements = []
labels = ["Radius:", "Elasticity:", "Friction:", "Density:",
          "Velocity X:", "Velocity Y:",
          "Color Red:", "Color Green:", "Color Blue:"]
defaults = ['20', '0.7', '0.9', '1', '0', '0', '0', '150', '255']

for i, label in enumerate(labels):
    y = y_pos + spacing * i
    textbox = TextBox(param_box_x, y, textbox_w, 32, defaults[i])
    checkbox = CheckBox(param_box_x + checkbox_offset, y + 4, checkbox_w, checkbox_w)
    ui_elements.append({'label': label, 'textbox': textbox, 'checkbox': checkbox})

param_configs = [
    {'key': 'radius',   'random_func': r.uniform, 'random_args': (5, 60),    'converter': float},
    {'key': 'elastic',  'random_func': r.uniform, 'random_args': (0.1, 1.0),  'converter': float},
    {'key': 'friction', 'random_func': r.uniform, 'random_args': (0.7, 1.0),  'converter': float},
    {'key': 'density',  'random_func': r.uniform, 'random_args': (0.5, 5.0),  'converter': float},
    {'key': 'vx',       'random_func': r.uniform, 'random_args': (-500, 500), 'converter': float},
    {'key': 'vy',       'random_func': r.uniform, 'random_args': (-500, 500), 'converter': float},
    {'key': 'cr',       'random_func': r.randint, 'random_args': (0, 255),    'converter': lambda text: int(float(text))},
    {'key': 'cg',       'random_func': r.randint, 'random_args': (0, 255),    'converter': lambda text: int(float(text))},
    {'key': 'cb',       'random_func': r.randint, 'random_args': (0, 255),    'converter': lambda text: int(float(text))},
]


while running:
    # event handling
    for event in py.event.get():
        if event.type == py.QUIT:
            running = False
            py.quit()
            exit()

        for el in ui_elements:
            el['textbox'].handle_event(event)
            el['checkbox'].handle_event(event)

        if event.type == py.MOUSEBUTTONDOWN:
            location = py.mouse.get_pos()

            if location[0] < sim_width:
                if event.button == 1:  # Left click to add a ball

                    try:
                        params = {}
                        temp_params = {}

                        # process parameters based on config
                        for i, config in enumerate(param_configs):
                            ui_el = ui_elements[i]
                            key = config['key']

                            if ui_el['checkbox'].checked:
                                temp_params[key] = config['random_func'](*config['random_args'])
                            else:
                                temp_params[key] = config['converter'](ui_el['textbox'].text)

                        params['radius'] = temp_params['radius']
                        params['elastic'] = temp_params['elastic']
                        params['friction'] = temp_params['friction']
                        params['density'] = temp_params['density']
                        params['vel'] = np.array([temp_params['vx'], temp_params['vy']], dtype=float)
                        # fancy linebreaks i found for the inline IF statements :))))
                        cr = temp_params['cr'] if ui_elements[6]['checkbox'].checked \
                            else max(0, min(255,temp_params['cr']))
                        cg = temp_params['cg'] if ui_elements[7]['checkbox'].checked \
                            else max(0, min(255,temp_params['cg']))
                        cb = temp_params['cb'] if ui_elements[8]['checkbox'].checked \
                            else max(0, min(255,temp_params['cb']))
                        params['color'] = (cr, cg, cb)

                        # Create the ball
                        objects.append(ball(location, **params))
                    except (ValueError, TypeError) as e:
                        print(f"Invalid number in a parameter box: {e}. Ball not created.")

                if event.button == 3:  # Right-click to remove a ball
                    for obj in reversed(objects):
                        if np.linalg.norm(obj.pos - np.array(location)) < obj.radius:
                            objects.remove(obj)
                            break

                            # physics handling
    dt = clock.tick(fps) / 1000.0 * playback_speed
    dt = min(dt, 0.1) # lag handling
    # physics
    for object in objects:
        update_physics(object, dt)
        border_collisions(object, 1000, 1000)
    ball_collisions(objects)

    # display handling
    window.fill('black')
    py.draw.line(window, (50, 50, 50), (sim_width, 0), (sim_width, window_height), 2)

    for object in objects:
        py.draw.circle(window, object.color, object.pos, object.radius)

    # Draw UI panel
    random_label = small_font.render("Rand", True, (200, 200, 200))
    window.blit(random_label, (param_box_x + checkbox_offset - 5, y_pos - 30))

    for el in ui_elements:
        label_surface = font.render(el['label'], True, (255, 255, 255))
        window.blit(label_surface, (el['textbox'].rect.x, el['textbox'].rect.y - 30))
        el['textbox'].draw(window)
        el['checkbox'].draw(window)

    py.display.update()


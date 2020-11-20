import pygame as pygame
import sys
import math
from trajectory_algorithm import trajectory_algorithm


# distance between two 2D points
def dis(t1, t2):
    return math.hypot(t2[0] - t1[0], t2[1] - t1[1])


class TrajectoryGUI:
    def __init__(self, size, w_l, o_l, b_l, path):

        self.input_type = 0
        self.charging = 0
        self.charging_t = (0, 0)
        self.insert_w = len(w_l) - 1
        self.insert_b = len(b_l) - 1
        self.window_size = size
        self.w_l = w_l
        self.o_l = o_l
        self.b_l = b_l
        self.scaled_path = path
        self.scaling = float('+inf')
        if len(w_l) + len(o_l) + len(b_l) + len(path) > 0:
            self.resize()
        else:
            self.scaling = (self.window_size / 5000) * 0.95

        pygame.init()
        pygame.display.set_caption('simulation')
        self.screen = pygame.display.set_mode((self.window_size, self.window_size))

        self.background = pygame.image.load('bg.jpg').convert()

    def resize(self):
        p_x = []
        p_y = []
        for t in self.scaled_path:
            p_x.append(t[0])
            p_y.append(t[1])
        w_x = []
        w_y = []
        for t in self.w_l:
            w_x.append(t[0])
            w_y.append(t[1])
        o_x = []
        o_y = []
        for t in self.o_l:
            o_x.append(t[0][0])
            o_y.append(t[0][1])
        b_x = []
        b_y = []
        for t in self.b_l:
            b_x.append(t[0])
            b_y.append(t[1])
        if len(self.scaled_path) > 0:
            min_p_x = min(p_x)
            max_p_x = max(p_x)
            min_p_y = min(p_y)
            max_p_y = max(p_y)
        else:
            min_p_x = float('+inf')
            max_p_x = float('-inf')
            min_p_y = float('+inf')
            max_p_y = float('-inf')

        if len(self.w_l) > 0:
            min_w_x = min(w_x)
            max_w_x = max(w_x)
            min_w_y = min(w_y)
            max_w_y = max(w_y)
        else:
            min_w_x = float('+inf')
            max_w_x = float('-inf')
            min_w_y = float('+inf')
            max_w_y = float('-inf')

        if len(self.o_l) > 0:
            min_o_x = min(o_x)
            max_o_x = max(o_x)
            min_o_y = min(o_y)
            max_o_y = max(o_y)
        else:
            min_o_x = float('+inf')
            max_o_x = float('-inf')
            min_o_y = float('+inf')
            max_o_y = float('-inf')

        if len(self.b_l) > 0:
            min_b_x = min(b_x)
            max_b_x = max(b_x)
            min_b_y = min(b_y)
            max_b_y = max(b_y)
        else:
            min_b_x = float('+inf')
            max_b_x = float('-inf')
            min_b_y = float('+inf')
            max_b_y = float('-inf')
        min_x_value = min(min_p_x, min_w_x, min_o_x, min_b_x)
        max_x_value = max(max_p_x, max_w_x, max_o_x, max_b_x)
        min_y_value = min(min_p_y, min_w_y, min_o_y, min_b_y)
        max_y_value = max(max_p_y, max_w_y, max_o_y, max_b_y)
        if min_x_value == float('+inf'):
            min_x_value = 0
        if max_x_value == float('-inf'):
            max_x_value = 0
        if min_y_value == float('+inf'):
            min_y_value = 0
        if max_y_value == float('-inf'):
            max_y_value = 0
        new_map_size = max(2 * abs(min_x_value), 2 * abs(max_x_value), 2 * abs(min_y_value), 2 * abs(max_y_value))
        if new_map_size > 0:
            new_scaling = (self.window_size / new_map_size) * 0.95
            if new_scaling < self.scaling:
                self.scaling = new_scaling

    def compute_path(self):
        self.scaled_path = trajectory_algorithm(self.w_l, self.o_l, self.b_l)
        print("NEW TRAJECTORY")
        for i in self.scaled_path:
          print(i[0]/3.281, i[1]/3.281)

    def draw_resized_circle(self, c, x, y, r):
        pygame.draw.circle(self.screen, c, (x / self.window_size, y / self.window_size),
                           r / self.window_size)

    def draw_resized_waypoints(self, li, ins, c):
        for i in range(len(li)):
            if (ins - 1) % len(li) == i or ins % len(li) == i:
                size = 4500
            else:
                size = 3000
            pygame.draw.circle(self.screen, c, self.con(li[i]), size / self.window_size)

    def draw_path(self, path, fac, c):
        for i in range(len(path) - fac):
            pygame.draw.line(self.screen, c, self.con(path[i + fac]), self.con(path[i + fac - 1]), width=2)

    def update(self):
        self.resize()
        self.screen.blit(self.background, (0, 0))

        self.draw_resized_circle((0, 255, 255), 25000, 30000, 6000)
        self.draw_resized_circle((255, 255, 255), 25000, 60000, 6000)
        self.draw_resized_circle((255, 255, 0), 25000, 90000, 6000)

        self.draw_path(self.b_l, 0, (255, 255, 0))
        self.draw_resized_waypoints(self.b_l, self.insert_b, (255, 0, 255))

        for i in range(len(self.o_l)):
            pygame.draw.circle(self.screen, (255, 255, 255),
                               self.con(self.o_l[i][0]), self.o_l[i][1] * self.scaling)

        if self.charging == 1:
            pos = pygame.mouse.get_pos()
            pygame.draw.circle(self.screen, (255, 255, 255), self.charging_t, dis(self.charging_t, pos))

        if len(self.w_l) > 1:
            self.draw_path(self.scaled_path, 1, (255, 0, 0))
            for i in range(int(len(self.scaled_path))):
                pygame.draw.circle(self.screen, (0, 255, 0), self.con(self.scaled_path[i]), 3000 / self.window_size)

        self.draw_resized_waypoints(self.w_l, self.insert_w, (0, 255, 255))

    def inv_x(self, x):
        return (x - self.window_size / 2) / self.scaling

    def inv_y(self, y):
        return (self.window_size / 2 - y) / self.scaling

    def inv(self, t):
        return self.inv_x(t[0]), self.inv_y(t[1])

    def con_x(self, x):
        return self.window_size / 2 + x * self.scaling

    def con_y(self, y):
        return self.window_size / 2 - y * self.scaling

    def con(self, t):
        return self.con_x(t[0]), self.con_y(t[1])

    def events(self):
        while True:
            pygame.display.update()
            if self.charging == 1:
                self.update()
            for event in pygame.event.get():
                compute = False
                if event.type == pygame.MOUSEBUTTONDOWN:
                    pos = pygame.mouse.get_pos()

                    if dis(pos, (25000 / self.window_size, 30000 / self.window_size)) < 6000 / self.window_size:
                        if pygame.mouse.get_pressed()[0]:
                            self.input_type = 0
                        else:
                            self.w_l.clear()
                            compute = True

                    elif dis(pos, (25000 / self.window_size, 60000 / self.window_size)) < 6000 / self.window_size:
                        if pygame.mouse.get_pressed()[0]:
                            self.input_type = 1
                        else:
                            self.o_l.clear()
                            compute = True

                    elif dis(pos, (25000 / self.window_size, 90000 / self.window_size)) < 6000 / self.window_size:
                        if pygame.mouse.get_pressed()[0]:
                            self.input_type = 2
                        else:
                            self.b_l.clear()
                            compute = True

                    else:
                        contact = False

                        p = 0
                        while p < len(self.o_l):
                            if (dis(pos, self.con(self.o_l[p][0])) <
                                    self.o_l[p][1] * self.scaling):
                                contact = True
                                if pygame.mouse.get_pressed()[0]:
                                    p += 1
                                else:
                                    del self.o_l[p]
                                    compute = True
                            else:
                                p += 1

                        p = 0
                        while p < len(self.b_l):
                            if (dis(pos, self.con(self.b_l[p])) <
                                    3000 / self.window_size):
                                contact = True
                                if pygame.mouse.get_pressed()[0]:
                                    p += 1
                                    self.insert_b = p - 1
                                    self.update()
                                else:
                                    del self.b_l[p]
                                    compute = True
                            else:
                                p += 1

                        p = 0
                        while p < len(self.w_l):
                            if (dis(pos, self.con(self.w_l[p])) <
                                    3000 / self.window_size):
                                contact = True
                                if pygame.mouse.get_pressed()[0]:
                                    p += 1
                                    self.insert_w = p - 1
                                    self.update()
                                else:
                                    del self.w_l[p]
                                    compute = True
                            else:
                                p += 1

                        if not contact and pygame.mouse.get_pressed()[0]:
                            if self.input_type == 0:
                                self.w_l.insert(self.insert_w, self.inv(pos))
                                compute = True
                                if len(self.w_l) == 2:
                                    self.insert_w = 0
                            elif self.input_type == 1:
                                self.charging = 1
                                self.charging_t = pos
                            elif self.input_type == 2:
                                self.b_l.insert(self.insert_b, self.inv(pos))
                                if len(self.w_l) > 1:
                                    self.compute_path()
                                self.update()

                if event.type == pygame.MOUSEBUTTONUP:
                    pos = pygame.mouse.get_pos()
                    if self.charging == 1:
                        self.charging = 0
                        self.o_l.append((self.inv(self.charging_t), dis(self.inv(self.charging_t), self.inv(pos))))
                        compute = True
                if compute:
                    if len(self.w_l) > 1:
                        self.compute_path()
                    self.update()
                if event.type == pygame.QUIT:
                    pygame.quit()
                    sys.exit()

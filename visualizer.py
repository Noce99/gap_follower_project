import os
os.environ['PYGAME_HIDE_SUPPORT_PROMPT'] = "hide pygame message"
import pygame
import sys
import math
import numpy as np
import utils

CAR_SIZE = 5

class Visualizer:
    
    CAR_LONG_SIZE = CAR_SIZE
    CAR_SHORT_SIZE = CAR_LONG_SIZE // 2
    STREET_WIDTH = 60
    RIGHT_BORDER_SIZE = 300

    def __init__(self, vehicle, track, track_name, waypoints, get_lidar, get_control_input=None):
        self.vehicle = vehicle
        self.waypoints = waypoints
        self.next_waypoint = 1
        self.passed_frame_from_waypoint = 0
        self.get_lidar = get_lidar
        self.get_control_input = get_control_input
        self.track_name = track_name
        self.track = track
        self.yellow_track = np.zeros((self.track.shape[0], self.track.shape[1], 3))
        for i in range(self.track.shape[0]):
            for j in range(self.track.shape[1]):
                if self.track[i, j] == 1:
                    self.yellow_track[i, j, 0] = 255 
                    self.yellow_track[i, j, 1] = 188
                    self.yellow_track[i, j, 2] = 0    
        
        self.passed_frame = 0
        
        # PyGame Initialization
        pygame.init()
        
        # info = pygame.display.Info()
        self.window_size = track.shape[0]# min(info.current_h, info.current_w)
        
        self.window = pygame.display.set_mode((self.window_size+self.RIGHT_BORDER_SIZE, self.window_size), display=0)
        pygame.display.set_caption(f"Autonomous Driving Project - Track {self.track_name}")
        
        self.font = pygame.font.Font("UbuntuMono-R.ttf", 32)

        self.FPS = 30
        self.clock = pygame.time.Clock()
         
    def draw_frame(self):
        # First of all let's clean the screen
        self.window.fill((0, 0, 0))
        
        # There I draw the map
        self.window.blit(pygame.surfarray.make_surface(self.yellow_track), (0, 0))
        
        # There I draw the waypoints
        for i, waypoint in enumerate(self.waypoints):
            if i == self.next_waypoint:
                color = (255, 100, 0)
            else:
                color = (0, 255, 0)
            if i == 0:
                radius = 10
            else:
                radius = 4
            pygame.draw.circle(self.window, color, (waypoint[0], waypoint[1]), radius)
            
        distance_to_next_point = math.sqrt((self.vehicle.position[0] - self.waypoints[self.next_waypoint][0])**2
                                         + (self.vehicle.position[1] - self.waypoints[self.next_waypoint][1])**2)
        self.passed_frame_from_waypoint += 1
        if distance_to_next_point < (self.STREET_WIDTH * 1.1):
            if self.next_waypoint == 0:
                print(f"Race finished in {self.passed_frame} frames!")
                print(f"Lidar Cost = {utils.calculate_lidar_cost():.2f}")
                print(f"Evaluation = {self.passed_frame/10000 * utils.calculate_lidar_cost():.2f}")
                quit_event = pygame.event.Event(pygame.QUIT)
                pygame.event.post(quit_event)
            self.passed_frame_from_waypoint = 0
            self.next_waypoint += 1
            if self.next_waypoint == len(self.waypoints):
                self.next_waypoint = 0
        
        # There I draw the vehicle
        pygame.draw.polygon(self.window, (255, 0, 0), self.get_point_arrow_from_vehicle(self.vehicle))
        
        # There I draw the lidar points
        lidar_points = self.get_lidar(
                self.vehicle.position[0],
                self.vehicle.position[1],
                self.vehicle.orientation)
        if lidar_points is not None:
            for x, y, distance, _ in lidar_points:
                 pygame.draw.circle(self.window, (255, 0, 0), (x, y), 2)
        else:
            print("Impossible to get Lidar Points!")
            if self.track[int(self.vehicle.position[0]), int(self.vehicle.position[1])] == 0:
                print("You are outside from the track!")
            quit_event = pygame.event.Event(pygame.QUIT)
            pygame.event.post(quit_event)
            
        # There I call the function to get input from the AI
        if self.get_control_input is not None and lidar_points is not None:
            distances_lidar_points = [d for _, _, d, _ in lidar_points]
            thetas_lidar_points = [theta for _, _, _, theta in lidar_points]
            position_lidar_points = [(x, y) for x, y, _, _ in lidar_points]
            gap, choosen_direction, control_the_vehicle = self.get_control_input(distances_lidar_points)
            
            # There I draw the gap
            if gap is not None:
                start_index = gap[0]
                final_index = start_index + gap[1]
                pygame.draw.line(self.window,
                        (0, 125, 0),
                        (self.vehicle.position[0], self.vehicle.position[1]),
                        (lidar_points[start_index][0], lidar_points[start_index][1]))
                pygame.draw.line(self.window,
                        (0, 125, 0),
                        (self.vehicle.position[0], self.vehicle.position[1]),
                        (lidar_points[final_index][0], lidar_points[final_index][1]))
            
            
            # There I draw the choosen direction
            acceleration = None
            steering = None
            
            if choosen_direction is not None:
                pygame.draw.line(self.window,
                        (0, 0, 190),
                        (self.vehicle.position[0], self.vehicle.position[1]),
                        (lidar_points[choosen_direction][0], lidar_points[choosen_direction][1]))
            
                acceleration, steering = control_the_vehicle(float(self.vehicle.orientation), float(self.vehicle.steering_angle), float(self.vehicle.speed), float(lidar_points[choosen_direction][3]))
            else:
                print("choosen_direction is NONE!")
                print("I cannot continue!")
                quit_event = pygame.event.Event(pygame.QUIT)
                pygame.event.post(quit_event)
                
            if acceleration is None and steering is None:
                self.vehicle.do_nothing()
            else:
                if acceleration is True:
                    self.vehicle.accelerate()
                elif acceleration is False:
                    self.vehicle.decelerate()
                if steering is True:
                    self.vehicle.stearing_left()
                elif steering is False:
                    self.vehicle.stearing_right()
            self.vehicle.integrate()
                
        # There I draw the steering angle in a graphical represetation
        steering_graphic_rect = pygame.Rect(self.window_size, 0, self.RIGHT_BORDER_SIZE, self.RIGHT_BORDER_SIZE)
        pygame.draw.arc(self.window, (255, 0, 0), steering_graphic_rect, 0, math.pi)
        rendered_line = self.font.render("Steering Wheels", True, (0, 255, 0), (0, 0, 128))
        steering_rect = rendered_line.get_rect()
        steering_rect.center = (
                int(self.window_size+self.RIGHT_BORDER_SIZE - 0.5 * (self.RIGHT_BORDER_SIZE - steering_rect.width) - 0.5 * steering_rect.width),
                int(self.RIGHT_BORDER_SIZE/2 + 0.5 * steering_rect.height)
        )
        self.window.blit(rendered_line, steering_rect)
        pygame.draw.line(self.window,
                        (0, 255, 0),
                        (self.window_size+self.RIGHT_BORDER_SIZE//2, self.RIGHT_BORDER_SIZE//2),
                        (self.window_size+self.RIGHT_BORDER_SIZE//2+math.sin(self.vehicle.steering_angle)*self.RIGHT_BORDER_SIZE//2,
                         self.RIGHT_BORDER_SIZE//2-math.cos(self.vehicle.steering_angle)*self.RIGHT_BORDER_SIZE//2)
                        )
        
        # There I draw the speed in a graphical represetation
        speed_graphic_rect = pygame.Rect(self.window_size, self.RIGHT_BORDER_SIZE//2+steering_rect.height*2, self.RIGHT_BORDER_SIZE, self.RIGHT_BORDER_SIZE)
        pygame.draw.arc(self.window, (255, 0, 0), speed_graphic_rect, 0, math.pi)
        rendered_line = self.font.render("Speed", True, (0, 255, 0), (0, 0, 128))
        speed_rect = rendered_line.get_rect()
        speed_rect.center = (
                int(self.window_size+self.RIGHT_BORDER_SIZE - 0.5 * (self.RIGHT_BORDER_SIZE - speed_rect.width) - 0.5 * speed_rect.width),
                int(self.RIGHT_BORDER_SIZE + steering_rect.height*2 + 0.5 * speed_rect.height)
        )
        self.window.blit(rendered_line, speed_rect)
        pygame.draw.line(self.window,
                        (0, 255, 0),
                        (self.window_size+self.RIGHT_BORDER_SIZE//2, self.RIGHT_BORDER_SIZE//2+steering_rect.height*2+self.RIGHT_BORDER_SIZE//2),
                        (self.window_size+self.RIGHT_BORDER_SIZE//2+math.sin(self.vehicle.speed/self.vehicle.max_speed*math.pi-math.pi/2)*self.RIGHT_BORDER_SIZE//2,
                         self.RIGHT_BORDER_SIZE//2+steering_rect.height*2+self.RIGHT_BORDER_SIZE//2-math.cos(self.vehicle.speed/self.vehicle.max_speed*math.pi-math.pi/2)*self.RIGHT_BORDER_SIZE//2)
                        )
        
        # There I draw some log
        lines = [
                    f"ESC for skipping track",
                    f"",
                    f"Steering Angle = {self.vehicle.steering_angle:.2f}", 
                    f"Speed = {self.vehicle.speed:.2f}",
                    f"Passed Frames = {self.passed_frame}",
                ]
        self.print_lines_bottom_right_corner(lines)
                
        # Final Update of the View
        pygame.display.update()
        self.clock.tick(self.FPS)
       
    def print_lines_bottom_right_corner(self, lines):
        for i, line in enumerate(lines):
            rendered_line = self.font.render(line, True, (0, 255, 0), (0, 0, 128))
            rect = rendered_line.get_rect()
            rect.center = (
                    int(self.window_size+self.RIGHT_BORDER_SIZE - 0.5 * rect.width),
                    int(self.window_size - 0.5 * rect.height - rect.height*i)
            )
            self.window.blit(rendered_line, rect)
    
    def get_point_arrow_from_vehicle(self, vehicle):
        f = [5*self.CAR_LONG_SIZE//2., 0.]
        b_lf = [-5*self.CAR_LONG_SIZE//2., 5*self.CAR_SHORT_SIZE//2]
        b_rg = [-5*self.CAR_LONG_SIZE//2., -5*self.CAR_SHORT_SIZE//2]
        
        front = [0, 0]
        back_lf = [0, 0]
        back_rg = [0, 0]
        
        front[0] = math.cos(vehicle.orientation)*f[0] - math.sin(vehicle.orientation)*f[1]
        front[1] = math.sin(vehicle.orientation)*f[0] + math.cos(vehicle.orientation)*f[1]
        back_lf[0] = math.cos(vehicle.orientation)*b_lf[0] - math.sin(vehicle.orientation)*b_lf[1]
        back_lf[1] = math.sin(vehicle.orientation)*b_lf[0] + math.cos(vehicle.orientation)*b_lf[1]
        back_rg[0] = math.cos(vehicle.orientation)*b_rg[0] - math.sin(vehicle.orientation)*b_rg[1]
        back_rg[1] = math.sin(vehicle.orientation)*b_rg[0] + math.cos(vehicle.orientation)*b_rg[1]

        front[0] += vehicle.position[0]
        front[1] += vehicle.position[1]
        back_lf[0] += vehicle.position[0]
        back_lf[1] += vehicle.position[1]
        back_rg[0] += vehicle.position[0]
        back_rg[1] += vehicle.position[1]
        return [front, back_lf, back_rg]

    def start(self):
        while True:
            events = pygame.event.get()
            for event in events:
                if event.type == pygame.QUIT:
                    print(f"Closing Track {self.track_name}")
                    pygame.quit()
                    return None
                elif event.type == pygame.KEYUP:
                    if event.key == 119:  # W
                        self.vehicle.accelerate()
                        break
                    elif event.key == 100:  # D
                        self.vehicle.stearing_right()
                        break
                    elif event.key == 115:  # S
                        self.vehicle.decelerate()
                        break
                    elif event.key == 97:  # A
                        self.vehicle.stearing_left()
                        break
                    elif event.key == 27:  # ESC
                        quit_event = pygame.event.Event(pygame.QUIT)
                        pygame.event.post(quit_event)
                    else:
                        print(f"Uknown Key: {event.key}")
                elif event.type == pygame.MOUSEBUTTONUP:
                    print(event.pos)
            else:
                self.vehicle.do_nothing()
            self.draw_frame()
            self.passed_frame += 1
            
            # If you run out of time I quit
            if self.passed_frame >= 10000:
                quit_event = pygame.event.Event(pygame.QUIT)
                pygame.event.post(quit_event)
            

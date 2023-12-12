import numpy as np
import random

class Lidar:
    RAY_STEP = 0.1
    PERCENTAGES_OF_NOISED_RAY = [0.0, 0.05, 0.10, 0.20]
    MAX_ERRORS_IN_M = [0, 10, 20, 30]

    def __init__(self, max_distance, number_of_rays, angle, quantity_of_noised_ray, error_size, track):
        self.max_distance = max_distance
        self.number_of_rays = number_of_rays
        self.track = track
        
        self.dtheta = angle / self.number_of_rays
        self.percentage_of_noised_ray = self.PERCENTAGES_OF_NOISED_RAY[quantity_of_noised_ray]
        self.max_error_in_m = self.MAX_ERRORS_IN_M[error_size]

    def get_lidar(self, x, y, orientation):
        self.angles = [orientation+self.dtheta*i for i in range(-self.number_of_rays//2, self.number_of_rays//2)]
        if self.track[int(x), int(y)] != 1:
            # print("Asked Lidar of a point outside from the track!")
            return None
        lidar_points = []
        for theta in self.angles:
            lidar_points.append(self.__lidar_ray(theta, x, y))
            
        # Let's put some noise
        number_of_noised_ray = int(self.percentage_of_noised_ray * self.number_of_rays)
        for _ in range(number_of_noised_ray):
            choosed_index = random.randrange(0, len(lidar_points))
            error_distance = random.random()*self.max_error_in_m
            error_sign = 1
            if random.random() < 0.5:
                error_sign = -1
            error_distance *= error_sign
            x_star, y_star, distance, theta = lidar_points[choosed_index]
            new_x_star = x_star + error_distance*np.cos(theta)
            new_y_star = y_star + error_distance*np.sin(theta)
            lidar_points[choosed_index] = \
            (
            new_x_star,
            new_y_star,
            np.sqrt((x-new_x_star)**2+(y-new_y_star)**2)/self.max_distance,
            theta
            ) 
        
        
        
        return lidar_points
    
    def __lidar_ray(self, theta, x, y):
        dx = self.RAY_STEP*np.cos(theta)
        dy = self.RAY_STEP*np.sin(theta)
        i = 0
        max_i = self.max_distance / self.RAY_STEP
        while True:
            i += 1
            x_star = x + i * dx
            y_star = y + i * dy
            if i >= max_i:
                break
            if self.track[int(x_star), int(y_star)] != 1:
                break
        return (x_star, y_star, np.sqrt((x-x_star)**2+(y-y_star)**2)/self.max_distance, theta)        


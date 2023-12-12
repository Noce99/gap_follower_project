import math

CAR_SIZE = 2
DT = 0.05

class Vehicle():
    d_STEEARING = math.pi / 50
    max_STEERING = math.pi / 4
    min_STEERING = - math.pi / 4
    max_speed = 100

    ACELERATION = 10
    L = CAR_SIZE

    def __init__(self, starting_position=[0, 0], starting_orientation=0):
        self.position = starting_position # A list of two elements (x, y coordinates) [m]
        self.orientation = starting_orientation # A single real number indicating the clockwise rotation of the vehicle axis respect to north [rad]

        self.steering_angle = 0 # The angle between the front-wheel plane and the plane passing from the vehicle axis and perpendicular to the floor [rad]

        self.speed = 0 # Simply the speed [m/s]
        self.angular_speed = 0 # The angular speed around the main vertical axis [rad/s]

        self.speed_input = None
        self.steering_input = None
        self.aceleration = 0 # Simply the aceleration [m/s^2]

    
    def after_integrate(func):
        def call_and_integrate(self):
            func(self)
            self.__integrate(DT)
        return call_and_integrate
    
    def stearing_right(self):
        self.steering_angle += self.d_STEEARING
        if self.steering_angle > self.max_STEERING:
            self.steering_angle = self.max_STEERING

    def stearing_left(self):
        self.steering_angle -= self.d_STEEARING
        if self.steering_angle < self.min_STEERING:
            self.steering_angle = self.min_STEERING

    def accelerate(self):
        self.aceleration = self.ACELERATION

    def decelerate(self):
        self.aceleration = -self.ACELERATION

    
    def do_nothing(self):
        pass

    def integrate(self, dt=DT):
        self.speed += self.aceleration*dt
        if self.speed > self.max_speed:
            self.speed = self.max_speed
        self.angular_speed = self.speed*math.tan(self.steering_angle)/self.L

        self.orientation += self.angular_speed*dt
        if self.orientation > 2*math.pi:
            self.orientation -= 2*math.pi
        elif self.orientation < 0:
            self.orientation += 2*math.pi
        self.position[0] += self.speed*math.cos(self.orientation)*dt
        self.position[1] += self.speed*math.sin(self.orientation)*dt
        self.aceleration = 0


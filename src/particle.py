from math import *
import random

class Particle:

    # --------
    # init: 
    #    creates robot and initializes location/orientation 
    #

    def __init__(self, world_size = 30.0):
        self.x = random.uniform(-world_size, world_size) # initial x position
        self.y = random.uniform(-world_size, world_size) # initial y position
        self.orientation = random.random() * 2.0 * pi # initial orientation
        self.bearing_noise  = 0.05 # initialize bearing noise to zero
        self.steering_noise = 0.1 # initialize steering noise to zero
        self.distance_noise = 1.0 # initialize distance noise to zero

    # --------
    # set: 
    #    sets a robot coordinate
    #

    def set(self, new_x, new_y, new_orientation):

        if new_orientation < 0 or new_orientation >= 2 * pi:
            raise ValueError('Orientation must be in [0..2pi]')
        self.x = float(new_x)
        self.y = float(new_y)
        self.orientation = float(new_orientation)

    # --------
    # set_noise: 
    #    sets the noise parameters
    #
    def set_noise(self, new_b_noise, new_s_noise, new_d_noise):
        # makes it possible to change the noise parameters
        # this is often useful in particle filters
        self.bearing_noise  = float(new_b_noise)
        self.steering_noise = float(new_s_noise)
        self.distance_noise = float(new_d_noise)

    # --------
    # measurement_prob
    #    computes the probability of a measurement
    #  

    def measurement_prob(self, pose_est): 
        error_bearing = sqrt((pose_est[0] - self.x) ** 2 + (pose_est[1] - self.y) ** 2)        

        # update Gaussian
        error = (exp(- (error_bearing ** 2) / (self.bearing_noise ** 2) / 2.0) /  
                    sqrt(2.0 * pi * (self.bearing_noise ** 2)))

        return error
    
    def __repr__(self): #allows us to print particle attributes.
        return '[x=%.6s y=%.6s orient=%.6s]' % (str(self.x), str(self.y), 
                                                str(self.orientation))
    
    ############# ONLY ADD/MODIFY CODE BELOW HERE ###################
       
    # --------
    # move: 
    #   
    
    # copy your code from the previous exercise
    # and modify it so that it simulates motion noise
    # according to the noise parameters
    #           self.steering_noise
    #           self.distance_noise
    
    def move(self, linear_velocity, angular_velocity, tal = 5):
        # Adicionando ruído à velocidade linear e angular
        linear_velocity = random.gauss(linear_velocity, self.distance_noise)
        angular_velocity = random.gauss(angular_velocity, self.steering_noise)

        # Se a velocidade angular for muito pequena, podemos tratar como movimento em linha reta
        if abs(angular_velocity) < 1e-6:
            # Movimento em linha reta
            new_x = self.x + linear_velocity * cos(self.orientation) * tal
            new_y = self.y + linear_velocity * sin(self.orientation) * tal
            new_orientation = self.orientation
        else:
            # Movimento em curva
            radius = linear_velocity / angular_velocity
            new_orientation = (self.orientation + angular_velocity * tal) % (2.0 * pi)
            new_x = self.x + radius * (sin(new_orientation) - sin(self.orientation))
            new_y = self.y - radius * (cos(new_orientation) - cos(self.orientation))

        # Criando um novo resultado como uma nova partícula
        result = Particle()
        result.set(new_x, new_y, new_orientation)
        
        return result

    
    ############## ONLY ADD/MODIFY CODE ABOVE HERE ####################
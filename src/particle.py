from math import *
import random

class Particle:

    # --------
    # init: 
    #    creates robot and initializes location/orientacao 
    #

    def __init__(self, world_size = 30.0):
        self.x = random.uniform(-world_size, world_size) # initial x position
        self.y = random.uniform(-world_size, world_size) # initial y position
        self.orientacao = random.random() * 2.0 * pi # initial orientacao
        self.ruido_frente  = 0.05 # initialize bearing noise to zero
        self.ruido_virar = 0.087 # initialize steering noise to zero
        self.erro_pose_est = 0.8 #posicao estimada do robo com valor aleatorio de 0.8m representando o erro da posicao real
        self.sigma_atualizacao = 1.5
        self.sigma_translacao = 0.2
        self.w = 0.0

    # --------
    # set: 
    #    sets a robot coordinate
    #

    def set(self, new_x, new_y, new_orientacao):

        if new_orientacao < 0 or new_orientacao >= 2 * pi:
            raise ValueError('orientacao must be in [0..2pi]')
        self.x = float(new_x)
        self.y = float(new_y)
        self.orientacao = float(new_orientacao)

    # --------
    # set_noise: 
    #    sets the noise parameters
    #
    def set_noise(self, new_frente_noise, new_virar_noise):
        # makes it possible to change the noise parameters
        # this is often useful in particle filters
        self.ruido_frente  = float(new_frente_noise)
        self.ruido_virar = float(new_virar_noise)

    # --------
    # measurement_prob
    #    computes the probability of a measurement
    #  
    def gaussian(self, mu, sigma, x):
        return exp(-((mu-x)**2)/(sigma**2) / 2) / sqrt(2*pi*(sigma**2))
        
    def measurement_prob(self, pose_est):         
        
        dist = sqrt((pose_est[0] - self.x) ** 2 + (pose_est[1] - self.y) ** 2)     
        
        # update Gaussian
        prob = self.gaussian(0, self.sigma_atualizacao, dist)
        #print(prob)
        return prob
    
    def __repr__(self): #allows us to print particle attributes.
        return '[x=%.6s y=%.6s orient=%.6s]' % (str(self.x), str(self.y), 
                                                str(self.orientacao))
    
    ############# ONLY ADD/MODIFY CODE BELOW HERE ###################
       
    # --------
    # move: 
    #   
    
    # copy your code from the previous exercise
    # and modify it so that it simulates motion noise
    # according to the noise parameters
    #           self.ruido_virar
    #           self.erro_pose_est
    
    def move(self, dist, comando):
        # Adicionando ruído à velocidade linear e angular   
        # Robo indo para frente
        dist = random.gauss(dist, self.sigma_translacao) 

        #andar para frente
        if comando == 1:
            new_orientacao = self.orientacao
        #se acao = virar para esquerda e se movimentar para frente
        elif comando == 2:
            new_orientacao = (self.orientacao + pi/2 + self.ruido_virar) % (2 * pi)
        #se acao = virar para direita e se movimentar para frente
        elif comando == 3:
            new_orientacao = (self.orientacao - pi/2 + self.ruido_virar) % (2 * pi)   
        elif comando == 4:
            new_orientacao = (self.orientacao + pi + self.ruido_virar) % (2 * pi)  

        new_x = self.x + dist * cos(new_orientacao)
        new_y = self.y + dist * sin(new_orientacao)        

        # Criando um novo resultado como uma nova partícula
        result = Particle()
        result.set(new_x, new_y, new_orientacao)
        
        return result

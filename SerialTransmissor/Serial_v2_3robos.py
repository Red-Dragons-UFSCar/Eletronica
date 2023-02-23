 # Importação de bibliotecas
from time import sleep
import serial
import time
import pygame
from pygame.locals import *
import matplotlib.pyplot as plt


# Instanciamento do objeto serial de comunicação
ser = serial.Serial()
ser.baudrate = 115200
ser.port = 'COM8' #'/dev/ttyACM0' # Conferir a porta USB que será utilizada
ser.open()
dados = []
tempos = []

# Mensagem a ser enviada - Padrão 1
# Senha - Direções Robô 0 - Velocidade 1 e 2 Robô 0 - Direções Robô 1 - Velocidade 1 e 2 Robô 1 - Direções Robô 2 - Velocidade 1 e 2 Robô 2
#Rd = bytearray([110,0,1,0,1,37,0,0,1,0,1,0,0,0,1,0,1,0,0])


dirMot1_Robo1 = 0b01000000
dirMot2_Robo1 = 0b00100000
dirMot1_Robo2 = 0b00001000
dirMot2_Robo2 = 0b00000010

direcao1 = dirMot1_Robo1 + dirMot2_Robo1 + dirMot1_Robo2 + dirMot2_Robo2

dirMot1_Robo3 = 0b01000000
dirMot2_Robo3 = 0b00010000

direcao2 = dirMot1_Robo3 + dirMot2_Robo3

Rd = bytearray([111, direcao1, direcao2, 40, 50, 60, 70,80, 90, 112])

t1 = time.time()

pygame.init()
win = pygame.display.set_mode((1000, 1000))
pygame.display.set_caption("Moving rectangle")

while (1):

    for event in pygame.event.get():
        if event.type == pygame.QUIT:
                  run = False

    keys = pygame.key.get_pressed()

    t2 = time.time()
    print(t2-t1)

    # ROBO 2:       SETAS
    if keys[pygame.K_RIGHT]:
        print("RIGHT Robo SETAS")

        dirMot1_Robo2 = 0b00000100
        dirMot2_Robo2 = 0b00000010
        v1 = 10
        v2 = 10

    elif keys[pygame.K_LEFT]:
        print("LEFT Robo SETAS")

        dirMot1_Robo2 = 0b00001000
        dirMot2_Robo2 = 0b00000001
        v1 = 10
        v2 = 10
    elif keys[pygame.K_UP]:
        print("UP Robo SETAS")

        dirMot1_Robo2 = 0b00001000
        dirMot2_Robo2 = 0b00000010
        v1 = 25
        v2 = 25
    elif keys[pygame.K_DOWN]:
        print("DOWN Robo SETAS")

        dirMot1_Robo2 = 0b00000100
        dirMot2_Robo2 = 0b00000001
        v1 = 25
        v2 = 25
    else:

        dirMot1_Robo2 = 0b00001100
        dirMot2_Robo2 = 0b00000011
        v1 = 0
        v2 = 0

    # ROB0 3:     WASD
    if keys[pygame.K_w]:
        print("UP Robo WASD")
        dirMot1_Robo3 = 0b10000000
        dirMot2_Robo3 = 0b00100000
        v3 = 30
        v4 = 30


    elif keys[pygame.K_s]:
        #print("DOWN Robo WASD")
        dirMot1_Robo3 = 0b01000000
        dirMot2_Robo3 = 0b00010000
        v3 = 15
        v4 = 15

    elif keys[pygame.K_d]:
        #print("LEFT Robo WASD")
        dirMot1_Robo3 = 0b01000000
        dirMot2_Robo3 = 0b00100000
        v3 = 5
        v4 = 5


    elif keys[pygame.K_a]:
            #print("RIGHT Robo WASD")
            dirMot1_Robo3 = 0b10000000
            dirMot2_Robo3 = 0b00010000
            v3 = 5
            v4 = 5


    else:
        dirMot1_Robo3 = 0b11000000
        dirMot2_Robo3 = 0b00110000
        v3 = 0
        v4 = 0

    # ROBO 1:    IJKL
    if keys[pygame.K_i]:
        print("UP Robo IJKL")
        dirMot1_Robo1 = 0b10000000 # Direito
        dirMot2_Robo1 = 0b00100000 # Esquerdo
        va = 25
        vb = 25


    elif keys[pygame.K_k]:
        print("DOWN Robo IJKL")
        dirMot1_Robo1 = 0b01000000 # Esquerda
        dirMot2_Robo1 = 0b00100000 # Direita
        va = 25
        vb = 25

    elif keys[pygame.K_j]:
        print("LEFT Robo IJKL")
        dirMot1_Robo1 = 0b10000000 # Esquerda
        dirMot2_Robo1 = 0b00100000 # Direita
        va = 15
        vb = 15


    elif keys[pygame.K_l]:
        print("RIGHT Robo IJKL")
        dirMot1_Robo1 = 0b01000000
        dirMot2_Robo1 = 0b00010000
        va = 15
        vb = 15
    elif keys[pygame.K_p]:
        print("RIGHT Robo IJKL")
        break


    else:
        dirMot1_Robo1 = 0b11000000
        dirMot2_Robo1 = 0b00110000
        va = 0
        vb = 0

    # if t2-t1 < 5:
    #     dirMot1_Robo1 = 0b01000000
    #     dirMot2_Robo1 = 0b00100000
    #     dirMot1_Robo2 = 0b00001000
    #     dirMot2_Robo2 = 0b00000010
    #     dirMot1_Robo3 = 0b01000000
    #     dirMot2_Robo3 = 0b00010000
    #     v1 = 5
    #     v2 = 5
    #     v3 = 10
    #     v4 = 10
    # elif t2-t1 < 10:
    #     dirMot1_Robo1 = 0b01000000
    #     dirMot2_Robo1 = 0b00100000
    #     dirMot1_Robo2 = 0b00001000
    #     dirMot2_Robo2 = 0b00000010
    #     dirMot1_Robo3 = 0b01000000
    #     dirMot2_Robo3 = 0b00010000
    #     v1 = 10
    #     v2 = 10
    #     v3 = 20
    #     v4 = 20
    # else:
    # 	t1 = t2
    direcao1 = dirMot1_Robo1 + dirMot2_Robo1 + dirMot1_Robo2 + dirMot2_Robo2
    direcao2 = dirMot1_Robo3 + dirMot2_Robo3
    Rd = bytearray([111, direcao1, direcao2, va, vb, v1, v2, v3, v4, 112])

    Start = time.time()
    ser.write(Rd)
    #print(ord(ser.read(1)))
    end = time.time()
    dt = Start -end
    #print("A mensagem foi enviada e lida em:{}".format(dt))
    v1_a = ord(ser.read(1))
    v1_b = ord(ser.read(1))
    v2_a = ord(ser.read(1))
    v2_b = ord(ser.read(1)) 
    v3_a = ord(ser.read(1))
    v3_b = ord(ser.read(1))  



    print("V1_a : {}".format(v1_a))
    print("V1_b : {}".format(v1_b))
    print("V2_a : {}".format(v2_a))
    print("V2_b : {}".format(v2_b))
    print("V3_a : {}".format(v3_a))
    print("V3_b : {}".format(v3_b))



    #print(int(direcao1))

    # Delay para manter o código em 60fps
    if (dt < 1/60):
        #print("Delay...")
        sleep(1/60-dt)
    

ser.close()


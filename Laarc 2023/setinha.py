import pygame
from pygame.locals import *
import time
import serial 

ser = serial.Serial()
ser.baudrate = 115200
#ser.port = 'COM5' # Mudar aqui dependendo da COM do transmissor
ser.port = '/dev/ttyUSB0' # Mudar aqui dependendo da COM do transmissor

pygame.init()
win = pygame.display.set_mode((300, 300))
pygame.display.set_caption("Moving rectangle")
ser.open()

# Definição das variáveis de cada robô
#Robo 1
dirMot1_Robo1 = 0b10000000
dirMot2_Robo1 = 0b00100000

#Robo 2
dirMot1_Robo2 = 0b00001000
dirMot2_Robo2 = 0b00000010

#Robo 3
dirMot1_Robo3 = 0b10000000
dirMot2_Robo3 = 0b00100000

flagSendVelocityZero = False

# Função para definir a direção dos motores
def verifyDirection(v):
    if v > 0:
        direction = 0b01
    elif v < 0:
        direction = 0b10
    else:
        direction = 0b11
    return direction

t1 = time.time()

while (1):

    for event in pygame.event.get():
        if event.type == pygame.QUIT:
                  run = False

    keys = pygame.key.get_pressed()
    send1 = True
    send2 = True

    # ROBO 2:       SETAS
    if keys[pygame.K_RIGHT]:
        print("RIGHT Robo SETAS")
        vl_2 = -10
        vr_2 = 10
    elif keys[pygame.K_LEFT]:
        print("LEFT Robo SETAS")
        vl_2 = 10
        vr_2 = -10
    elif keys[pygame.K_UP]:
        print("UP Robo SETAS")
        vl_2 = 50
        vr_2 = 50
    elif keys[pygame.K_DOWN]:
        print("DOWN Robo SETAS")
        vl_2 = -50
        vr_2 = -50
    else:
        vl_2 = 0
        vr_2 = 0
        
    # ROB0 3:     WASD
    if keys[pygame.K_w]:
        print("UP Robo WASD")
        vl_3 = 70
        vr_3 = 70
    elif keys[pygame.K_s]:
        print("DOWN Robo WASD")
        vl_3 = -30
        vr_3 = -30
    elif keys[pygame.K_a]:
        print("LEFT Robo WASD")
        vl_3 = 10
        vr_3 = -10
    elif keys[pygame.K_d]:
        print("RIGHT Robo WASD")
        vl_3 = -10
        vr_3 = 10
    else:
        vl_3 = 0
        vr_3 = 0

    # ROBO 1:    IJKL
    if keys[pygame.K_i]:
        print("UP Robo IJKL")
        vl_1 = 30
        vr_1 = 30
    elif keys[pygame.K_k]:
        print("DOWN Robo IJKL")
        vl_1 = -30
        vr_1 = -30
    elif keys[pygame.K_j]:
        print("LEFT Robo IJKL")
        vl_1 = 10
        vr_1 = -10
    elif keys[pygame.K_l]:
        print("RIGHT Robo IJKL")
        vl_1 = -10
        vr_1 = 10
    else:
        vl_1 = 0
        vr_1 = 0

    # Direções de cada motor
    dirMot1_Robo1 = verifyDirection(vl_1)
    dirMot2_Robo1 = verifyDirection(vr_1)
    dirMot1_Robo2 = verifyDirection(vl_2)
    dirMot2_Robo2 = verifyDirection(vr_2)
    dirMot1_Robo3 = verifyDirection(vl_3)
    dirMot2_Robo3 = verifyDirection(vr_3)

    # Palavra de Bytes de direção
    direcao1 = dirMot1_Robo1<<6 | dirMot2_Robo1<<4 | dirMot1_Robo2<<2 | dirMot2_Robo2
    direcao2 = dirMot1_Robo3<<6 | dirMot2_Robo3<<4

    # Vetor de mensagem para a eletrônica
    print("Palavra: ", [111, direcao1, direcao2, vl_1, vr_1, vl_2, vr_2, vl_3, vr_3, 113])
    Rd = bytearray([111, direcao1, direcao2, abs(vl_1), abs(vr_1), abs(vl_2), abs(vr_2), abs(vl_3), abs(vr_3), 113])
    
    # Escrita na porta serial
    ser.write(Rd)
    # Delay para manter o código em 60fps
    time.sleep(1/60)

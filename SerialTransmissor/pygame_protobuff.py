import pygame
from pygame.locals import *
import time
from vss_communication import StrategyControl

pygame.init()
win = pygame.display.set_mode((300, 300))
pygame.display.set_caption("Moving rectangle")

t1 = time.time()

controle = StrategyControl()

while (1):

    for event in pygame.event.get():
        if event.type == pygame.QUIT:
                  run = False

    keys = pygame.key.get_pressed()

    # ROBO 2:       SETAS
    if keys[pygame.K_RIGHT]:
        print("RIGHT Robo SETAS")
        vl_1 = -10
        vr_2 = 10
    elif keys[pygame.K_LEFT]:
        print("LEFT Robo SETAS")
        vl_1 = 10
        vr_2 = -10
    elif keys[pygame.K_UP]:
        print("UP Robo SETAS")
        vl_1 = 30
        vr_2 = 30
    elif keys[pygame.K_DOWN]:
        print("DOWN Robo SETAS")
        vl_1 = -30
        vr_2 = -30
    else:
        vl_1 = 0
        vr_2 = 0

    # ROB0 3:     WASD
    if keys[pygame.K_w]:
        print("UP Robo WASD")
        v3 = 30
        v4 = 30
    elif keys[pygame.K_s]:
        print("DOWN Robo WASD")
        v3 = -30
        v4 = -30
    elif keys[pygame.K_a]:
        print("LEFT Robo WASD")
        v3 = 10
        v4 = -10
    elif keys[pygame.K_d]:
        print("RIGHT Robo WASD")
        v3 = -10
        v4 = 10
    else:
        v3 = 0
        v4 = 0

    # ROBO 1:    IJKL
    if keys[pygame.K_i]:
        print("UP Robo IJKL")
        va = 30
        vb = 30
    elif keys[pygame.K_k]:
        print("DOWN Robo IJKL")
        va = -30
        vb = -30
    elif keys[pygame.K_j]:
        print("LEFT Robo IJKL")
        va = 10
        vb = -10
    elif keys[pygame.K_l]:
        print("RIGHT Robo IJKL")
        va = -10
        vb = 10
    else:
        va = 0
        vb = 0

    controle.send_mensage(1, True, vl_1, vr_2)
    controle.send_mensage(2, True, v3, v4)
    controle.send_mensage(0, True, va, vb)

    # Delay para manter o código em 60fps
    time.sleep(1/60)
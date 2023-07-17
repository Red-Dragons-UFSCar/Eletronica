# Importação de bibliotecas
from time import sleep
import serial
import time

# Instanciamento do objeto serial de comunicação
ser = serial.Serial()
ser.baudrate = 115200
ser.port = '/dev/ttyACM0' # Conferir a porta USB que será utilizada
ser.open()

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

while (1):

    t2 = time.time()
    print(t2-t1)
    if t2-t1 < 5:
        dirMot1_Robo1 = 0b01000000
        dirMot2_Robo1 = 0b00100000
        dirMot1_Robo2 = 0b00001000
        dirMot2_Robo2 = 0b00000010
        dirMot1_Robo3 = 0b01000000
        dirMot2_Robo3 = 0b00010000
        v1 = 5
        v2 = 5
        v3 = 10
        v4 = 10
    elif t2-t1 < 10:
        dirMot1_Robo1 = 0b01000000
        dirMot2_Robo1 = 0b00100000
        dirMot1_Robo2 = 0b00001000
        dirMot2_Robo2 = 0b00000010
        dirMot1_Robo3 = 0b01000000
        dirMot2_Robo3 = 0b00010000
        v1 = 10
        v2 = 10
        v3 = 20
        v4 = 20
    else:
    	t1 = t2
    
    direcao1 = dirMot1_Robo1 + dirMot2_Robo1 + dirMot1_Robo2 + dirMot2_Robo2
    direcao2 = dirMot1_Robo3 + dirMot2_Robo3
    Rd = bytearray([111, direcao1, direcao2, v1, v2, v1, v2, v3, v4, 112])

    Start = time.time()
    ser.write(Rd)
    #print(ord(ser.read(1)))
    end = time.time()
    dt = Start -end
    print("A mensagem foi enviada e lida em:{}".format(dt))

    print(int(direcao1))

    # Delay para manter o código em 60fps
    if (dt < 1/60):
        print("Delay...")
        sleep(1/60-dt)

ser.close()

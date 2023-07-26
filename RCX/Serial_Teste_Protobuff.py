import serial
import time
from vss_communication import Actuator

#Inicialização da biblioteca e definição do baudrate do serial
ser = serial.Serial()
ser.baudrate = 115200

#ser.port = 'COM5' 
ser.port = '/dev/ttyUSB0' # Conferir a porta USB que será utilizada

ser.open()

# Instância do objeto actuator da comunicação
eletronica = Actuator(logger=False)

# Definição das variáveis de cada robô
#Robo 1
dirMot1_Robo1 = 0b10000000
dirMot2_Robo1 = 0b00100000
v1 = 0
v2 = 0

#Robo 2
v3 = 0
v4 = 0
dirMot1_Robo2 = 0b00001000
dirMot2_Robo2 = 0b00000010

#Robo 3
dirMot1_Robo3 = 0b10000000
dirMot2_Robo3 = 0b00100000
v5 = 0
v6 = 0

t1 = time.time()
t2 = time.time()
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

while True:
    # Recebe as informações do controle e estrategia
    eletronica.update()
    data, errorCode = eletronica.get_data()

    # Leitura de velocidades
    if data["robot_id"]==0:
        v1 = int(data["wheel_left"])
        v2 = int(data["wheel_right"])
    elif data["robot_id"]==1:
        v3 = int(data["wheel_left"])
        v4 = int(data["wheel_right"])
    elif data["robot_id"]==2:
        v5 = int(data["wheel_left"])
        v6 = int(data["wheel_right"])

    #print("Erro: ", errorCode)
    t2 = time.time()
    if errorCode != 0:
        if t2-t1 > 1:
            v1, v2, v3, v4, v5, v6 = 0,0,0,0,0,0
            print("Zerei")
    else:
        print("FPS: ", 1/(t2-t1))
        t1 = time.time()

    # Direções de cada motor
    dirMot1_Robo1 = verifyDirection(v1)
    dirMot2_Robo1 = verifyDirection(v2)
    dirMot1_Robo2 = verifyDirection(v3)
    dirMot2_Robo2 = verifyDirection(v4)
    dirMot1_Robo3 = verifyDirection(v5)
    dirMot2_Robo3 = verifyDirection(v6)

    # Palavra de Bytes de direção
    direcao1 = dirMot1_Robo1<<6 | dirMot2_Robo1<<4 | dirMot1_Robo2<<2 | dirMot2_Robo2
    direcao2 = dirMot1_Robo3<<6 | dirMot2_Robo3<<4

    # Vetor de mensagem para a eletrônica
    if errorCode == 0: print("Palavra: ", [111, direcao1, direcao2, v1, v2, v3, v4, v5, v6, 113])
    Rd = bytearray([111, direcao1, direcao2, abs(v1), abs(v2), abs(v3), abs(v4), abs(v5), abs(v6), 113])
    
    # Escrita na porta serial
    ser.write(Rd)

    # Leitura dos valores de bateria
    senha1 = ord(ser.read())
    bateria_1 = ord(ser.read())
    bateria_2 = ord(ser.read())
    bateria_3 = ord(ser.read())
    b_4 = ord(ser.read())
    if(b_4 == 113 and senha1 == 111):
        bats = [bateria_1,bateria_2,bateria_3]
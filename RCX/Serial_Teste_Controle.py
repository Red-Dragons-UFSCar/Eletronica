import serial
import time
import pandas as pd

#Inicialização da biblioteca e definição do baudrate do serial
ser = serial.Serial()
ser.baudrate = 115200

ser.port = 'COM5' #'/dev/ttyACM0' # Conferir a porta USB que será utilizada



#Robo 3
dirMot1_Robo3 = 0b10000000
dirMot2_Robo3 = 0b00100000
#Robo 2
dirMot1_Robo1 = 0b10000000
dirMot2_Robo1 = 0b00100000
#Robo 1
dirMot1_Robo2 = 0b00001000
dirMot2_Robo2 = 0b00000010
#velocidades robo 1
v1 = 10
v2 = 10

v3 = 30
v4 = 30

v5 = 20
v6 = 20

#configuração direção das rodas
direcao1 = dirMot1_Robo1 + dirMot2_Robo1 + dirMot1_Robo2 + dirMot2_Robo2
direcao2 = dirMot1_Robo3 + dirMot2_Robo3

#montagem da mensagem
Rd = bytearray([111, direcao1, direcao2, v1, v2, v3, v4, v5, v6, 113])


while(True):
    print("digite o proximo comando desejado \n1 - Testar o controle \n2 - Alterar constantes de controle \n3 - Pausar")
    opc  = int(input())
    if opc == 1:
        ser.open()
        t = 0
        start = time.time()
        while t<=3:
            senha1 = 0
            b_4 = 0
            Rd = bytearray([111, 170, 160, v1, v2, v3, v4, v5, v6, 113])
            ser.write(Rd)
            end = time.time()
            t = end - start
        while t<=5 and t>3:
            Rd = bytearray([111, 0, 0, 0, 0, 0, 0, 0, 0, 113])
            ser.write(Rd)
            end = time.time()
            t = end -start
        ser.close()
    elif opc == 2:
        print("antes de prosseguir digite as novas constantes no arquivo txt \nDigite 1 para prosseguir apos digitar")
        opc2 = int(input())
        ser.open()
        if opc2 == 1:
            t = 0
            a=[0,0,0,0,0,0,0,0]
            with open('controle.txt') as f:
                contador = 0
                for linhas in f:
                    if contador != 7:
                        a[contador] = int(linhas)
                        contador +=1 
            t = 0
            start = time.time()
            while t<= 2:
                Rd = bytearray([112, a[0], a[1], a[2], a[3], a[4], a[5], a[6], a[7], 113])
                ser.write(Rd)
                senha1 = ord(ser.read())
                bateria_1 = ord(ser.read())
                bateria_2 = ord(ser.read())
                bateria_3 = ord(ser.read())
                b_4 = ord(ser.read())
                if(b_4 == 113 and senha1 == 111):
                    print("mensagem validada")
                    bats = [bateria_1,bateria_2,bateria_3]
                end = time.time()
                t =end - start
            Rd = bytearray([111, 0, 0, v1, v2, v3, v4, v5, v6, 113])
            ser.write(Rd)
        ser.close()
    elif opc ==3:
        break   



            
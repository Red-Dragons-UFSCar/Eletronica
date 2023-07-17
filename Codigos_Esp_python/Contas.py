dirMot1_Robo3 = 0b10000000
dirMot2_Robo3 = 0b00100000
#Robo 2
dirMot1_Robo1 = 0b10000000
dirMot2_Robo1 = 0b00100000
#Robo 1
dirMot1_Robo2 = 0b00001000
dirMot2_Robo2 = 0b00000010

direcao1 = dirMot1_Robo1 + dirMot2_Robo1 + dirMot1_Robo2 + dirMot2_Robo2
direcao2 = dirMot1_Robo3 + dirMot2_Robo3

print(direcao1)
print(direcao2)
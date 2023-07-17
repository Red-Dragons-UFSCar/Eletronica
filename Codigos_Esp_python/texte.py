a=[0,0,0,0,0,0,0]
with open('controle.txt', 'r') as f:
    contador = 0
    for linhas in f:
        if contador != 7:
            a[contador] = int(linhas)
            contador +=1 
print(a)
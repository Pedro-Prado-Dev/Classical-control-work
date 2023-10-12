import numpy as np
import control as cnt
import matplotlib.pyplot as plt
from scipy.io import loadmat

mat=loadmat('TransferFunction6.mat')
# print(mat)
#Variáveis
degrau = mat.get('degrau')
saida=mat.get('saida')
t = mat.get('t')

amplitude_degrau = max(degrau)
amplitude_saida = max(saida)
#considerando uma função de transferencia em malha aberta FT=k/(tau*s+1)
k=amplitude_saida/amplitude_degrau
k = k[0]
tau= 14.95
Theta = 5.98 # atraso de propagação
#parâmetros do controlador kp+kp/(Ti*s)+kp*Td*s

# CHR 1 
kp=(0.6*tau)/(k*Theta)
Ti=tau
Td=0.5*Theta

kp = kp/2
Ti = Ti*1.05
Td = Td/8

# Integral do erro
kp_IE=1/((Theta/tau)+0.2)
Ti_IE=(0.3*(Theta/tau)+1.2)/((Theta/tau)+0.08)
Td_IE=(1/(90*(Theta/tau)))

kp_IE = kp_IE/k
Ti_IE = Ti_IE*Theta
Td_IE = Td_IE*Theta

kp_IE = kp_IE/2.15
Ti_IE = Ti_IE*0.992
Td_IE = Td_IE/0.3

# Função para criar o PID
def criarPID(kp,Ti,Td,Hs):
    # Controlador proporcional
    numkp = np. array ([kp])
    denkp = np. array ([1])
    #integral
    numki = np. array ([kp])
    denki = np. array ([Ti,0])
    #derivativo
    numkd = np. array ([kp*Td,0])
    denkd = np. array ([1])
    #Construindo o controlador PID
    Hkp = cnt.tf(numkp , denkp)
    Hki=cnt.tf(numki , denki)
    Hkd=cnt.tf(numkd , denkd)
    Hctrl1 = cnt.parallel (Hkp , Hki)
    Hctrl = cnt.parallel (Hctrl1 , Hkd)
    Hdel = cnt.series (Hs , Hctrl)
    Hcl1 = cnt.feedback(Hdel, 1)
    Hcl1 = Hcl1*k
    
    return Hcl1

print(kp)
print(Ti)
print(Td)

# Função de transferência da planta
num = np. array ([k])
den = np. array ([tau , 1])
H = cnt.tf(num , den)
n_pade = 20
( num_pade , den_pade ) = cnt.pade ( Theta , n_pade )
H_pade = cnt.tf( num_pade , den_pade )
Hs = cnt.series (H , H_pade)

# Pega as informações digitadas pelo usuário
kp_USU = float(input('Entre com o valor de Kp: '))
Ti_USU = float(input('Entre com o valor de Ti: '))
Td_USU = float(input('Entre com o valor de Td: '))
setpoint = float(input('Entre com o setpoint: '))

# Cria os PIDs
Hcl_CHR = criarPID(kp,Ti,Td,Hs) # CHR
Hcl_IE = criarPID(kp_IE,Ti_IE,Td_IE,Hs) # Integral de erro
Hcl_USU = criarPID(kp_USU,Ti_USU,Td_USU,Hs) # Usuário

# Calcula a resposta ao impulso
(t , y_CHR ) = cnt.step_response ( Hcl_CHR, t)
(t , y_IE ) = cnt.step_response ( Hcl_IE, t)
(t , y_USU ) = cnt.step_response ( Hcl_USU, t)
y_CHR = y_CHR*amplitude_degrau
y_IE = y_IE*amplitude_degrau
y_USU = y_USU*amplitude_degrau

# Plot
plt.subplot(2,1,1)
plot1=plt.plot(t.T, saida, label='Saída')
plot2=plt.plot(t.T, degrau,label='degrau de entrada')
plot3=plt.plot(t.T, y_CHR, label='CHR')
plot4=plt.plot(t.T, y_IE, label='Integral do erro' )

plt.xlabel ( ' t [ s ] ')
plt.ylabel('Amplitude')
plt.legend(loc="upper left")
plt.title('Controle PID')
plt.grid ()

saida_usuario = (saida/amplitude_saida)*setpoint

plt.subplot(2,1,2)
plot1=plt.plot(t.T, saida_usuario, label='Saída')
plot2=plt.plot(t.T, degrau,label='degrau de entrada')
plot4=plt.plot(t.T, y_USU, label='Usuário' )

plt.xlabel ( ' t [ s ] ')
plt.ylabel('Amplitude')
plt.legend(loc="upper left")
plt.title('Controle PID')
plt.grid ()

plt.show()
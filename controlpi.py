import numpy as np
import control as cnt
import matplotlib.pyplot as plt
from scipy.io import loadmat

mat=loadmat('TransferFunction6.mat')
# print(mat)
#Variáveis
degrau = mat.get('degrau')
saida=mat.get('saida')
t1 = mat.get('t')

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

#Calcular esses três -> copiar em um novo arquivo, dps copiar até hs, depois pega o plot e trocar hcl por hs
#no (t, y ) da para calcular multiplicando hs com degrau
print(kp)
print(Ti)
print(Td)
#escrevendo a função de transferência da planta
num = np. array ([k])
den = np. array ([tau , 1])
H = cnt.tf(num , den)
n_pade = 20
( num_pade , den_pade ) = cnt.pade ( Theta , n_pade )
H_pade = cnt.tf( num_pade , den_pade )
Hs = cnt.series (H , H_pade)

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

# Controlador proporcional kp+kp/(Ti*s)+kp*Td*s
numkp = np. array ([kp_IE])
denkp = np. array ([1])
#integral
numki = np. array ([kp_IE])
denki = np. array ([Ti_IE,0])
#derivativo
numkd = np. array ([kp_IE*Td_IE,0])
denkd = np. array ([1])
#Construindo o controlador PID
Hkp = cnt.tf(numkp , denkp)
Hki=cnt.tf(numki , denki)
Hkd=cnt.tf(numkd , denkd)
Hctrl1 = cnt.parallel (Hkp , Hki)
Hctrl = cnt.parallel (Hctrl1 , Hkd)
Hdel = cnt.series (Hs , Hctrl)
Hcl2 = cnt.feedback(Hdel, 1)
Hcl2 = Hcl2*k

(t1 , y1 ) = cnt.step_response ( Hcl1, t1)
(t1 , y2 ) = cnt.step_response ( Hcl2, t1)
y1 = y1*amplitude_degrau
y2 = y2*amplitude_degrau


plot1=plt.plot(t1.T,saida, label='Saída')
plot2=plt.plot(t1.T,degrau,label='degrau de entrada')
plot3=plt.plot (t1 , y1 ,label='CHR')
plot4=plt.plot (t1 , y2,label='Integral do erro' )
plt.xlabel ( ' t [ s ] ')
plt.ylabel('Amplitude')
plt.legend(loc="upper left")
plt.title('Controle PID')

plt.grid ()
plt.show()
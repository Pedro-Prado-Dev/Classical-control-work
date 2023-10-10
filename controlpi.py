import numpy as np
import control as cnt
import matplotlib.pyplot as plt
#considerando uma função de transferencia em malha aberta FT=k/(tau*s+1)
k=4.3
tau= 14.95
Theta = 5.98 # atraso de propagação
#parâmetros do controlador kp+kp/(Ti*s)+kp*Td*s
kp=(0.6*tau)/(k*Theta)
Ti=tau
Td=0.5*Theta
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

t = np . linspace (0 , 40 , 100)
(t , y ) = cnt.step_response ( Hs, t )
plt.plot (t , y )
plt.xlabel ( ' t [ s ] ')
plt.ylabel('Amplitude')
plt.title('Controle PID')

plt.grid ()
plt.show()
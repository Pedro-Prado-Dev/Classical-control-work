import numpy as np
import control as cnt
import matplotlib.pyplot as plt
from scipy.io import loadmat

mat=loadmat('TransferFunction6.mat')

#Variáveis
degrau = mat.get('degrau')
saida=mat.get('saida')
saida = saida.flatten()
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

# Ajuste fino no CHR 1
kp_ajuste = kp/2
Ti_ajuste = Ti*1.05
Td_ajuste = Td/8

# Integral do erro
kp_IE=1/((Theta/tau)+0.2)
Ti_IE=(0.3*(Theta/tau)+1.2)/((Theta/tau)+0.08)
Td_IE=(1/(90*(Theta/tau)))

kp_IE = kp_IE/k
Ti_IE = Ti_IE*Theta
Td_IE = Td_IE*Theta

# Ajuste fino na Integral de erro
kp_IE_ajuste = kp_IE/2.15
Ti_IE_ajuste = Ti_IE*0.992
Td_IE_ajuste = Td_IE/0.3

# Função para criar o PID
def criarPID(kp,Ti,Td,Hs,feedback=0):
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
    Hcl1 = Hdel
    # Aplica o feedback
    if feedback == 1:
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

# kp_USU = 0.21
# Ti_USU = 15
# Td_USU = 0.37
# setpoint = 21

# Cria os PIDs sem feedback
Hcl_CHR = criarPID(kp,Ti,Td,Hs, 0) # CHR
Hcl_IE = criarPID(kp_IE,Ti_IE,Td_IE,Hs, 0) # Integral de erro
Hcl_USU = criarPID(kp_USU,Ti_USU,Td_USU,Hs, 0) # Usuário

# Cria os PIDs com feedback
Hcl_CHR_f = criarPID(kp,Ti,Td,Hs, 1) # CHR
Hcl_CHR_f_ajuste = criarPID(kp_ajuste,Ti_ajuste,Td_ajuste,Hs, 1) # CHR com ajuste fino
Hcl_IE_f = criarPID(kp_IE,Ti_IE,Td_IE,Hs, 1) # Integral de erro
Hcl_IE_f_ajuste = criarPID(kp_IE_ajuste,Ti_IE_ajuste,Td_IE_ajuste,Hs, 1) # Integral de erro com ajuste fino
Hcl_USU_f = criarPID(kp_USU,Ti_USU,Td_USU,Hs, 1) # Usuário

# Calcula a resposta ao impulso para os PIDs sem feedack
(t , y_CHR ) = cnt.step_response ( Hcl_CHR, t)
(t , y_IE ) = cnt.step_response ( Hcl_IE, t)
(t , y_USU ) = cnt.step_response ( Hcl_USU, t)
y_CHR = y_CHR*amplitude_degrau
y_IE = y_IE*amplitude_degrau
y_USU = y_USU*amplitude_degrau

# Calcula a resposta ao impulso para os PIDs com feedack
(t , y_CHR_f ) = cnt.step_response ( Hcl_CHR_f, t)
(t , y_CHR_f_ajuste ) = cnt.step_response ( Hcl_CHR_f_ajuste, t)
(t , y_IE_f ) = cnt.step_response ( Hcl_IE_f, t)
(t , y_IE_f_ajuste ) = cnt.step_response ( Hcl_IE_f_ajuste, t)
(t , y_USU_f ) = cnt.step_response ( Hcl_USU_f, t)
y_CHR_f = y_CHR_f*amplitude_degrau
y_CHR_f_ajuste = y_CHR_f_ajuste*amplitude_degrau
y_IE_f = y_IE_f*amplitude_degrau
y_IE_f_ajuste = y_IE_f_ajuste*amplitude_degrau
y_USU_f = y_USU_f*amplitude_degrau

# Calcula os vetores de erro em malha aberta
erro_CHR = y_CHR - saida
erro_IE = y_IE - saida

# Calcula os vetores de erro em malha fechada
erro_CHR_f = y_CHR_f - saida
erro_IE_f = y_IE_f - saida

# Plot Controle PID em malha aberta
plot1=plt.plot(t.T, saida, label='Saída')
plot2=plt.plot(t.T, degrau,label='degrau de entrada')
plot3=plt.plot(t.T, y_CHR, label='CHR')
plot4=plt.plot(t.T, y_IE, label='Integral do erro' )

plt.xlabel ( ' t [ s ] ')
plt.ylabel('Amplitude')
plt.legend(loc="upper left")
plt.title('Controle PID em malha aberta')
plt.grid ()

plt.show()

# Plot Controle PID em malha fechada
plot1=plt.plot(t.T, saida, label='Saída')
plot2=plt.plot(t.T, degrau,label='degrau de entrada')
plot3=plt.plot(t.T, y_CHR_f, label='CHR')
plot4=plt.plot(t.T, y_IE_f, label='Integral do erro' )

plt.xlabel ( ' t [ s ] ')
plt.ylabel('Amplitude')
plt.legend(loc="upper left")
plt.title('Controle PID em malha fechada')
plt.grid ()

plt.show()

# Plot Controle PID em malha fechada com ajuste fino
plot1=plt.plot(t.T, saida, label='Saída')
plot2=plt.plot(t.T, degrau,label='degrau de entrada')
plot3=plt.plot(t.T, y_CHR_f_ajuste, label='CHR')
plot4=plt.plot(t.T, y_IE_f_ajuste, label='Integral do erro' )

plt.xlabel ( ' t [ s ] ')
plt.ylabel('Amplitude')
plt.legend(loc="upper left")
plt.title('Controle PID em malha fechada com ajuste fino')
plt.grid ()

plt.show()

#Plot Controle PID com dados do usuário
saida_usuario = (saida/amplitude_saida)*setpoint

plot1=plt.plot(t.T, saida_usuario, label='Saída')
plot2=plt.plot(t.T, degrau,label='degrau de entrada')
plot3=plt.plot(t.T, y_USU, label='PID usuário em malha aberta' )
plot4=plt.plot(t.T, y_USU_f, label='PID usuário em malha fechada' )

plt.xlabel ( ' t [ s ] ')
plt.ylabel('Amplitude')
plt.legend(loc="upper left")
plt.title('Controle PID com dados do usuário')
plt.grid ()

plt.show()

#Plot dos Erros
plot1=plt.plot(t.T, erro_CHR, label='Erro CHR malha aberta')
plot2=plt.plot(t.T, erro_CHR_f,label='Erro CHR malha fechada')
plot3=plt.plot(t.T, erro_IE, label='Erro Integral de erro malha aberta' )
plot4=plt.plot(t.T, erro_IE_f, label='Erro Integral de erro malha fechada' )

plt.xlabel ( ' t [ s ] ')
plt.ylabel('Amplitude')
plt.legend(loc="upper left")
plt.title('Erros')
plt.grid ()

plt.show()
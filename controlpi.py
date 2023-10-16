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
kp_ajuste = kp*0.8
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
kp_IE_ajuste = kp_IE/1.2
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
    return Hcl1

print(kp_IE)
print(Ti_IE)
print(Td_IE)

# Função de transferência da planta
num = np. array ([k])
den = np. array ([tau , 1])
H = cnt.tf(num , den)
n_pade = 20
( num_pade , den_pade ) = cnt.pade ( Theta , n_pade )
H_pade = cnt.tf( num_pade , den_pade )
Hs = cnt.series (H , H_pade)
Hs_f = cnt.feedback(Hs, 1)

# Pega as informações digitadas pelo usuário
kp_USU = float(input('Entre com o valor de Kp: '))
Ti_USU = float(input('Entre com o valor de Ti: '))
Td_USU = float(input('Entre com o valor de Td: '))
setpoint = float(input('Entre com o setpoint: '))

# kp_USU = 0.21
# Ti_USU = 15
# Td_USU = 0.37
# setpoint = 6

# Cria os PIDs com feedback
Hcl_CHR_f = criarPID(kp,Ti,Td,Hs, 1) # CHR
Hcl_CHR_f_ajuste = criarPID(kp_ajuste,Ti_ajuste,Td_ajuste,Hs, 1) # CHR com ajuste fino
Hcl_IE_f = criarPID(kp_IE,Ti_IE,Td_IE,Hs, 1) # Integral de erro
Hcl_IE_f_ajuste = criarPID(kp_IE_ajuste,Ti_IE_ajuste,Td_IE_ajuste,Hs, 1) # Integral de erro com ajuste fino
Hcl_USU_f = criarPID(kp_USU,Ti_USU,Td_USU,Hs, 1) # Usuário

# Calcula a resposta ao impulso para os PIDs com feedack
(t , y_CHR_f ) = cnt.step_response ( Hcl_CHR_f, t)
(t , y_CHR_f_ajuste ) = cnt.step_response ( Hcl_CHR_f_ajuste, t)
(t , y_IE_f ) = cnt.step_response ( Hcl_IE_f, t)
(t , y_IE_f_ajuste ) = cnt.step_response ( Hcl_IE_f_ajuste, t)
(t , y_USU_f ) = cnt.step_response ( Hcl_USU_f, t)
(t , y_esti ) = cnt.step_response ( Hs, t)
(t , y_esti_f ) = cnt.step_response ( Hs_f, t)
y_CHR_f = y_CHR_f*amplitude_degrau
y_CHR_f_ajuste = y_CHR_f_ajuste*amplitude_degrau
y_IE_f = y_IE_f*amplitude_degrau
y_IE_f_ajuste = y_IE_f_ajuste*amplitude_degrau
y_USU_f = y_USU_f*amplitude_degrau
y_esti = y_esti*amplitude_degrau
y_esti_f = y_esti_f*amplitude_degrau

# Calcula os vetores de erro em malha aberta
erro = y_esti[len(y_esti)-1] - amplitude_degrau

# Calcula os vetores de erro em malha fechada
erro_f = y_esti_f[len(y_esti_f)-1] - amplitude_degrau

print(erro)
print(erro_f)

# Plot Controle PID em malha aberta e fechada
plot1=plt.plot(t.T, saida, label='Saída')
plot2=plt.plot(t.T, degrau,label='degrau de entrada')
plot3=plt.plot(t.T, y_esti, label='Malha Aberta')

plt.xlabel ( ' t [ s ] ')
plt.ylabel('Amplitude')
plt.legend(loc="upper left")
plt.title('Saída da planta')
plt.grid ()

plt.show()

# Plot Controle PID em malha aberta e fechada
plot1=plt.plot(t.T, saida, label='Saída')
plot2=plt.plot(t.T, degrau,label='degrau de entrada')
plot3=plt.plot(t.T, y_esti, label='Malha Aberta')
plot4=plt.plot(t.T, y_esti_f, label='Malha fechada' )

plt.xlabel ( ' t [ s ] ')
plt.ylabel('Amplitude')
plt.legend(loc="upper left")
plt.title('Saída da planta em malha aberta e fechada')
plt.grid ()

plt.show()

# Plot Controle PID CHR
plot1=plt.plot(t.T, saida, label='Saída')
plot2=plt.plot(t.T, degrau,label='degrau de entrada')
plot3=plt.plot(t.T, y_CHR_f, label='CHR')

plt.xlabel ( ' t [ s ] ')
plt.ylabel('Amplitude')
plt.legend(loc="upper left")
plt.title('Controle PID CHR')
plt.grid ()

plt.show()

# Plot Controle PID CHR ajustado
plot1=plt.plot(t.T, saida, label='Saída')
plot2=plt.plot(t.T, degrau,label='degrau de entrada')
plot3=plt.plot(t.T, y_CHR_f_ajuste, label='CHR')

plt.xlabel ( ' t [ s ] ')
plt.ylabel('Amplitude')
plt.legend(loc="upper left")
plt.title('Controle PID CHR ajustado')
plt.grid ()

plt.show()

# Plot Controle PID Integral do erro
plot1=plt.plot(t.T, saida, label='Saída')
plot2=plt.plot(t.T, degrau,label='degrau de entrada')
plot4=plt.plot(t.T, y_IE_f, label='Integral do erro' )

plt.xlabel ( ' t [ s ] ')
plt.ylabel('Amplitude')
plt.legend(loc="upper left")
plt.title('Controle PID Integral de erro')
plt.grid ()

plt.show()

# Plot Controle PID Integral do erro ajustado
plot1=plt.plot(t.T, saida, label='Saída')
plot2=plt.plot(t.T, degrau,label='degrau de entrada')
plot4=plt.plot(t.T, y_IE_f_ajuste, label='Integral do erro' )

plt.xlabel ( ' t [ s ] ')
plt.ylabel('Amplitude')
plt.legend(loc="upper left")
plt.title('Controle PID Integral de erro ajustado')
plt.grid ()

plt.show()

# Plot Comparação CHR e Integral do erro
plot1=plt.plot(t.T, saida, label='Saída')
plot2=plt.plot(t.T, degrau,label='degrau de entrada')
plot4=plt.plot(t.T, y_IE_f_ajuste, label='Integral do erro' )
plot4=plt.plot(t.T, y_CHR_f_ajuste, label='CHR' )

plt.xlabel ( ' t [ s ] ')
plt.ylabel('Amplitude')
plt.legend(loc="upper left")
plt.title('Comparação PID CHR e Integral de erro ajustados')
plt.grid ()

plt.show()

#Plot Controle PID com dados do usuário
degrau_usuario = (degrau/amplitude_degrau)*setpoint

plot2=plt.plot(t.T, degrau_usuario,label='degrau de entrada')
plot4=plt.plot(t.T, y_USU_f, label='PID usuário em malha fechada' )

plt.xlabel ( ' t [ s ] ')
plt.ylabel('Amplitude')
plt.legend(loc="lower right")
plt.title('Controle PID com dados do usuário')
plt.grid ()

plt.show()
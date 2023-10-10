from scipy.io import loadmat
import numpy as np
import matplotlib.pyplot as plt
mat=loadmat('TransferFunction6.mat')
print(mat)
#Variáveis
degrau = mat.get('degrau')
saida=mat.get('saida')
t1 = mat.get('t')


plot1=plt.plot(t1.T,saida, label='Saída')
plot2=plt.plot(t1.T,degrau,label='degrau de entrada')

plt.xlabel ( ' t [ s ] ')
plt.ylabel('Amplitude')
plt.legend(loc="upper left")

plt.grid ()
plt.show()

#deltaY=20.92 t1=10.97    t2=20.94    t=1.5(t2-t1)=14,95    0=t2-t=5,98      k =4,30
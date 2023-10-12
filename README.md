# Trabalho Controle Clássico

O método CHR proposto por [Chien, Hrones e Reswick, 1952] propõe dois critérios de desempenho:

- A resposta mais rápida do sistema sem sobrevalor.
- A resposta mais rápida do sistema com 20% de sobrevalor.
- As sintonias são obtidas tanto para o problema servo (mudança de valor do setpoint) como para o problema regulatório (perturbação de carga com setpoint constante).

O método da Integral do Erro-IAE 

1 - Carregue a função de transferência da planta destinada ao seu grupo. Para carregar os dados no MATLAB, execute os seguintes comandos:

- load('TransferFunction1.mat'). Isso resultará na exibição das variáveis de saída, do grau e do tempo (t). Existem 17 arquivos, considere "TransferFunctionX", com X representando o número do grupo de acordo com o arquivo do Excel, como exemplificado acima para o grupo 1.

2 - Escolha o método de identificação da planta e, com isso, encontre os valores de k, θ e τ.

3 - Plote a resposta original em relação à estimada na mesma figura e verifique se a aproximação foi satisfatória.

4 - Apresente os valores de erro da planta em malha aberta e fechada, e faça comentários sobre os resultados.

5 - Nesta etapa, você deve comparar um dos métodos tradicionais mencionados acima com os métodos de sintonia de Cohen e Coon para Curva de Reação e o método da Integral do Erro.

6 - Realize o ajuste fino, se necessário, e comente o que foi feito e qual o reflexo desse ajuste na resposta do sistema.

7 - Ao comparar os métodos, você identificou alguma desvantagem no método tradicional? Caso sim, o novo método resolveu o problema? Explique!

8 - Crie uma interface que permita que o usuário entre com os dados e os parâmetros do controlador PID e do Setpoint.

Para as bibliotecas usadas:

```python
pip install requirements.txt
```

Considerando uma função de transferencia em malha aberta FT=k/(tau*s+1):
```python
k=amplitude_saida/amplitude_degrau
k = k[0]
tau= 14.95
Theta = 5.98 # atraso de propagação
```

Parâmetros do controlador kp+kp/(Ti*s)+kp*Td*s
CHR 1
```python
kp=(0.6*tau)/(k*Theta)
Ti=tau
Td=0.5*Theta

kp = kp/2
Ti = Ti*1.05
Td = Td/8
```

Integral do erro
```python
kp_IE=1/((Theta/tau)+0.2)
Ti_IE=(0.3*(Theta/tau)+1.2)/((Theta/tau)+0.08)
Td_IE=(1/(90*(Theta/tau)))

kp_IE = kp_IE/k
Ti_IE = Ti_IE*Theta
Td_IE = Td_IE*Theta

kp_IE = kp_IE/2.15
Ti_IE = Ti_IE*0.992
Td_IE = Td_IE/0.3
```

Controle Proporcional
```python
numkp = np. array ([kp])
denkp = np. array ([1])
```
Integral
```python
numki = np. array ([kp])
denki = np. array ([Ti,0])
```
Derivativo
```python
numkd = np. array ([kp*Td,0])
denkd = np. array ([1])
```
Construindo o controlador PID
```python
Hkp = cnt.tf(numkp , denkp)
Hki=cnt.tf(numki , denki)
Hkd=cnt.tf(numkd , denkd)
Hctrl1 = cnt.parallel (Hkp , Hki)
Hctrl = cnt.parallel (Hctrl1 , Hkd)
Hdel = cnt.series (Hs , Hctrl)
Hcl1 = cnt.feedback(Hdel, 1)
Hcl1 = Hcl1*k
```

Controlador proporcional kp+kp/(Ti*s)+kp*Td*s
```python
numkp = np. array ([kp])
denkp = np. array ([1])
```
Integral
```python
numki = np. array ([kp_IE])
denki = np. array ([Ti_IE,0])
```
Derivativo
```python
numkd = np. array ([kp_IE*Td_IE,0])
denkd = np. array ([1])
```
Construindo o controlador PID
```python
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
```

Plot dos gráficos
```python
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
```

Pedro Henrique do Prado Paiva 1685 GEC

Guilherme Tomazoli de Carvalho 1664 GEC

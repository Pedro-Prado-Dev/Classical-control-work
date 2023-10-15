# Trabalho Controle Clássico

O método CHR proposto por [Chien, Hrones e Reswick, 1952] propõe dois critérios de desempenho:

- A resposta mais rápida do sistema sem sobrevalor.
- A resposta mais rápida do sistema com 20% de sobrevalor.
- As sintonias são obtidas tanto para o problema servo (mudança de valor do setpoint) como para o problema regulatório (perturbação de carga com setpoint constante).

O método da Integral do Erro-IAE 

1 - Carregue a função de transferência da planta destinada ao seu grupo. Para carregar os dados no MATLAB, execute os seguintes comandos:

- load('TransferFunction1.mat'). Isso resultará na exibição das variáveis de saída, do grau e do tempo (t). Existem 17 arquivos, considere "TransferFunctionX", com X representando o número do grupo de acordo com o arquivo do Excel, como exemplificado acima para o grupo 1.

2 - Escolha o método de identificação da planta e, com isso, encontre os valores de k, θ e τ.

```python
      amplitude_saida = 20,922
      amplitude_degrau = 6
      k = amplitude_saida/amplitude_degrau = 3,487
      t1 = 10,97
      t2 = 20,94
      t = 1.5(t2-t1)=14,95
      0 = t2-t=5,98
```


![saida](https://github.com/Pedro-Prado-Dev/Classical-control-work/assets/110736909/bfbf0ba1-3078-4cb6-9aed-02ff51dab200)

3 - Plote a resposta original em relação à estimada na mesma figura e verifique se a aproximação foi satisfatória.

![Resposta_sem_ajuste](https://github.com/Pedro-Prado-Dev/Classical-control-work/assets/100048797/1488255c-dc15-4307-beea-4bca34eb898a)


De um modo geral, pode-se dizer que a aproximação não está satisfatória, pois o controlador está com um tempo de subida bem menor do que o tempo de subida esperado. Contudo, um estudo mais aprofundado do ambiente real em que o sistema seria aplicado seria necessário para uma conclusão mais precisa, já que dependendo da aplicação a diferença entre os tempos de subida pode não ser tão relevante.

4 - Apresente os valores de erro da planta em malha aberta e fechada, e faça comentários sobre os resultados.

![Figure_(3)](https://github.com/Pedro-Prado-Dev/Classical-control-work/assets/100048797/0b6b328f-4db2-4619-b1cb-2e6b63a80b29)

Como é possível perceber com o gráfico acima, o erro é bem maior quando o sistema é de malha aberta, o que já era esperado, uma vez que o sistema não recebe um feedback do impacto que a sua atuação está causando no ambiente. Vale notar ainda, que no sistema em malha aberta a resposta divergiu, o que inviabiliza a sua aplicação em ambientes reais sem um ajuste fino prévio.


Já o sistema em malha fechada apresentou erros bem menores quando comparado com o sistema em malha aberta, e esses erros estão principalmente localizados nos segundos iniciais do funcionamento do sistema. Ao contrário do outro sistema, esse convergiu com um tempo similar ao esperado e assim pode ser aplicado em um ambiente real com alguns pequenos ajustes

5 - Nesta etapa, você deve comparar um dos métodos tradicionais mencionados acima com os métodos de sintonia de Cohen e Coon para Curva de Reação e o método da Integral do Erro.

Metodo clássico: CHR 1: O método CHR é baseado no trabalho de CHIEN; HRONES; RESWICK (1952). O método CHR é baseado em dois critérios: a resposta mais rápida sem sobressinal; e a resposta mais rápida possível com 20% de sobressinal. O método em questão considera tanto a sintonia para o problema regulador como para o problema servo. Em todos os casos, considera-se o sistema se comportando como um sistema de primeira ordem com atraso, com ganho estático K, constante de tempo t e tempo morto. A determinação desses parâmetros é feita analisando a resposta do processo por meio de um experimento em malha aberta a uma entrada em degrau.

Metodo novo : Integral do erro: Este método, considera que a dinâmica do processo pode ser representada por um modelo de primeira ordem com ganho K, constante de tempo τ (tau) e tempo morto θ (teta). Em Lopez et al. (1967), é descrito um método que minimiza os índices (IAE ou ITAE) para um problema do tipo regulador (perturbação de carga). Foram considerados sistemas com fator de incontrolabilidade entre 0 e 1. Quanto maior a integral do erro, pior é a malha de controle em questão. 

6 - Realize o ajuste fino, se necessário, e comente o que foi feito e qual o reflexo desse ajuste na resposta do sistema.

R: CHR 1:
Kp - dividindo o Kp por 2, temos o maximo de pico reduzido e o tempo de subida é aumentado.
Ti - multiplicando o Ti por 1.05, temos o tempo de subida aumentado, o maximo pico diminuido, o tempo de acomodação reduzida e a eliminação do erro regime permanente.
Td - dividindo o Td por 8, temos o maximo pico aumentado e o tempo de acomodação aumentado.
Com esses ajustes o resultado obtido foi mais proximo do resultado esperado.

Integral do erro:
Kp - dividindo o Kp por 2.15, temos o maximo de pico reduzido e o tempo de subida é aumentado.
Ti - multiplicando o Ti por 0.992, temos o tempo de subida aumentado, o maximo pico diminuido, o tempo de acomodação reduzida e a eliminação do erro regime permanente.
Td - dividindo o Td por 0.3, temos o maximo pico aumentado e o tempo de acomodação aumentado.
Com esses ajustes o resultado obtido foi mais proximo do resultado esperado.

Com o ajuste fino  
![Resposta_com_ajuste](https://github.com/Pedro-Prado-Dev/Classical-control-work/assets/100048797/93abd8cb-9875-4843-9c99-4c4955ea42a9)

7 - Ao comparar os métodos, você identificou alguma desvantagem no método tradicional? Caso sim, o novo método resolveu o problema? Explique!

R: Sim, pois sem a relização do ajuste fino o metodo tradicional teve mais oscilações que o novo metodo.

8 - Crie uma interface que permita que o usuário entre com os dados e os parâmetros do controlador PID e do Setpoint.

Interface criada:
```python
kp_USU = float(input('Entre com o valor de Kp: '))
Ti_USU = float(input('Entre com o valor de Ti: '))
Td_USU = float(input('Entre com o valor de Td: '))
setpoint = float(input('Entre com o setpoint: '))
```

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

Helio Abreu Marques Rocha 1763 GEC

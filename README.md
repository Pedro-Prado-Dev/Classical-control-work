# Trabalho Controle Clássico

O método CHR proposto por [Chien, Hrones e Reswick, 1952] propõe dois critérios de desempenho:

- A resposta mais rápida do sistema sem sobrevalor.
- A resposta mais rápida do sistema com 20% de sobrevalor.
- As sintonias são obtidas tanto para o problema servo (mudança de valor do setpoint) como para o problema regulatório (perturbação de carga com setpoint constante).

Para as bibliotecas usadas:

```python
pip install requirements.txt
```

considerando uma função de transferencia em malha aberta FT=k/(tau*s+1):
```python
k=amplitude_saida/amplitude_degrau
k = k[0]
tau= 14.95
Theta = 5.98 # atraso de propagação
```

parâmetros do controlador kp+kp/(Ti*s)+kp*Td*s
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

Pedro Henrique do Prado Paiva 1685 GEC

Guilherme Tomazoli de Carvalho 1664 GEC

############################
## Universidade Federal de Alagoas
## Instituto de Computação
## ECOM062 - Robótica
## CONTROLE CINEMÁTICO SCARA
## CONTROLE COM CINEMÁTICA DIFERENCIAL
## by José Augusto
############################


import sim
import time
import numpy as np
import spatialmath as sm
from functools import reduce
import roboticstoolbox as rtb
import matplotlib.pyplot as plt


# Função para facilitar a alteração de posição das juntas
def move_joints(clientID, jointHandles, jointPositions):
    for (position, joint) in zip(jointPositions, jointHandles):
        sim.simxSetJointPosition(clientID, joint, position, sim.simx_opmode_oneshot)
        time.sleep(0.01)
    

# Função para verificar se houve algum erro
has_error = lambda z: reduce(lambda x, y: x and y,
                   map(lambda x: x != sim.simx_error_noerror, z))

# Definindo Robô com a biblioteca Robotics ToolBox
# e os parâmetros de Denavit-Hatenberg
def get_SCARA():
    return rtb.DHRobot([
        rtb.RevoluteDH(a=.475, d=.0, alpha=.0),
        rtb.RevoluteDH(a=.4, d=.0, alpha=np.pi),
        rtb.PrismaticDH(a=.0, theta=.0, alpha=.0, qlim=[.0, .1]),
        rtb.RevoluteDH(a=.0, d=.0, alpha=.0)
   ])
   
SCARA = get_SCARA()

# Tabela de Denavit-Hatenberg
print(SCARA)

# termina todas as conexões só pra garantir
sim.simxFinish(-1)
print('Program started')

clientID = sim.simxStart('127.0.0.1', 19997, True, True, 5000, 5)

if clientID != -1:
    print ('Connected to remote API server')
    
    sim.simxStartSimulation(clientID, sim.simx_opmode_oneshot_wait)

    sim.simxAddStatusbarMessage(clientID, 'Funcionando...', sim.simx_opmode_oneshot_wait)

    time.sleep(2)
    
    # Colentando os handles úteis
    errorr, robot = sim.simxGetObjectHandle(clientID, 'MTB', sim.simx_opmode_oneshot_wait)
    errord, dummy = sim.simxGetObjectHandle(clientID, 'Dummy', sim.simx_opmode_oneshot_wait)
    error1, axis1 = sim.simxGetObjectHandle(clientID,  'rev1', sim.simx_opmode_oneshot_wait)
    error2, axis2 = sim.simxGetObjectHandle(clientID,  'rev2', sim.simx_opmode_oneshot_wait)
    error3, axis3 = sim.simxGetObjectHandle(clientID,  'rev3', sim.simx_opmode_oneshot_wait)
    error7, edge  = sim.simxGetObjectHandle(clientID,  'edge', sim.simx_opmode_oneshot_wait)
    
    # Criando lista com as juntas
    joints = [axis1, axis2, axis3]
    
    # Ativando stream de dados
    # Cria stream de dados como recomendado
    _, D = sim.simxGetObjectPosition(clientID, dummy, -1, sim.simx_opmode_streaming)
    _ = sim.simxGetJointPosition(clientID, axis1, sim.simx_opmode_streaming)
    _ = sim.simxGetJointPosition(clientID, axis2, sim.simx_opmode_streaming)
    _ = sim.simxGetJointPosition(clientID, axis3, sim.simx_opmode_streaming)
    _, X = sim.simxGetObjectPosition(clientID, edge, -1, sim.simx_opmode_streaming)
    
    
    # Checando erros iniciais
    errors = [errorr, errord, error1, error2, error3, error7]
    
    if has_error(errors):
        print('Erro encontrado. Finalizando programa.')
        
        sim.simxGetPingTime(clientID)
        sim.simxFinish(clientID)
        
        print('Programa finalizado.')
        
        exit(0)
        
    # Cria stream de dados, como recomendado
    _, position_robot = sim.simxGetObjectPosition(clientID, robot, -1, sim.simx_opmode_streaming)
    _, orientation_robot = sim.simxGetObjectOrientation(clientID, robot, -1, sim.simx_opmode_streaming)
    _, position_dummy = sim.simxGetObjectPosition(clientID, dummy, -1, sim.simx_opmode_streaming)
    _, orientation_dummy = sim.simxGetObjectOrientation(clientID, dummy, -1, sim.simx_opmode_streaming)
    time.sleep(2.)
    
    # Colocando robô em Home
    move_joints(clientID, joints, [0, 0, 0])
    time.sleep(2.)
    
    # Passo para integração
    dt = 0.06
    
    # Dicionário que guardará os erros para plotar depois
    errors = {
        "axis1": [],
        "axis2": [],
        "axis3": [],
        "axis4": [],
    }
    
    # Loop principal
    while True:
        # Pega posições da ponta e do objeto dummy
        _, E = sim.simxGetObjectPosition(clientID, edge, -1, sim.simx_opmode_buffer)
        _, D = sim.simxGetObjectPosition(clientID, dummy, -1, sim.simx_opmode_buffer)
        time.sleep(0.01)

        dist = np.linalg.norm(np.array(E) - np.array(D))
        
        if dist < 0.035:
            print("Perto o suficiente. Parando o progama.")
            
            for (key, value) in errors.items():
                plt.clf()
                plt.plot(value[20:], color='r') # removendo começo turbulento
                plt.title("Error for " + key)
                plt.savefig(key)
            
            # Pausar a simulação e fechar a conexão
            sim.simxPauseSimulation(clientID,sim.simx_opmode_oneshot_wait)
            sim.simxAddStatusbarMessage(clientID, 'Program paused', sim.simx_opmode_blocking)
            sim.simxFinish(clientID)
            
            exit(0)
            
        _, A1 = sim.simxGetJointPosition(clientID, axis1, sim.simx_opmode_buffer)
        _, A2 = sim.simxGetJointPosition(clientID, axis2, sim.simx_opmode_buffer)
        _, A3 = sim.simxGetJointPosition(clientID, axis3, sim.simx_opmode_buffer)
        _, A4 = _, .0
        time.sleep(0.05)
        
        A = [A1, A2, A3, A4]
        
        ## Algoritmo Resolved-Rate
        # Calcula a jacobiana e a jacobiana inversa
        J  = SCARA.jacobe(A)
        _J = np.linalg.pinv(J)
        
        # cinemática direta para A
        _edge  = SCARA.fkine(A);
        # Pega a matriz de transformação homogênea
        _dummy = sm.SE3(D[0], D[1], D[2])
        # Calcula a velocidade
        V, _ = rtb.p_servo(_edge, _dummy, 1)
        # Calcula iteração
        _A = np.matmul(_J, V)
        A  = A + _A * dt
        
        # Move as juntas para a posição calculada na iteração atual
        move_joints(clientID, joints, A[:-1])
        
        # Guardando erros para os gráficos
        for (i, erro) in enumerate(_A):
            errors['axis{}'.format(i+1)].append(erro)
else:
    print('Failed connecting to remote API server')
print('Program ended')

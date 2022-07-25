## Como rodar a simulação
### Projeto de robô (escolher robô e controlar com cinemática diferencial)
### Projeto de controle cinemático SCARA

Ambos as atividades têm a mesma estrutura e rodam de maneira parecida, siga as instruções abaixo.
Os códigos devem parar sozinhos quando a ponta do robô chegar perto do `dummy`, e logo depois as imagens de erro serão geradas. Para ver cada atividade, basta abrir o diretorio correspondente.

No projeto de robô, foi utilizado o robô UR5 que já está disponível no próprio CoppeliaSim.
O projeto roda tanto em windows quanto em linux, pois contém tanto a `.dll` quanto a `.o`.

OBS.: As vezes o robô pode ficar travado, não consegui consertar essa parte, mas basta rodar de novo o projeto que irá funcionar.

Antes de rodar, instalar as dependências do python 3:
- `pip install spatialmath-python==1.0.0`
- `pip install numpy==1.22.1`
- `pip install roboticstoolbox-python==1.0.1`
- `pip install matplotlib==3.5.1`

Para rodar este projeto, siga as instruções:
1. Abrir o CoppeliaSim
2. Carregar o `projeto_robo.ttt` (ou `scara_scene.ttt`, para o trabalho de Controle Cinemático SCARA)
3. Dar play na simulação
4. Rodar `python projeto_robo.py` no terminal (ou `python controle_cinematico_scara.py`, para o trabalho de Controle Cinemático SCARA)

O programa terminará quando a ponta do robô, i.e., `edge` estiver próxima o suficiente do `dummy`.

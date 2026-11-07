# Modelos do CoppeliaSim

Esta pasta não versiona arquivos `.ttm` (ver `.gitignore`): o modelo do robô é do projeto 2025S2 (`lucianovk/IA368II`) e não é redistribuído aqui.

Para gerar `myRobot.ttm` localmente:

1. `scripts/setup.sh` baixa o repositório do Luciano em `src/deps/` (somente leitura, Art. II).
2. `python3 tools/sim_bench/bench.py --model-src <caminho>/pega_banana.ttt` exporta `/myRobot` para `.sim/myRobot.ttm`. O padrão do `--model-src` é um caminho local do autor; ajuste-o para onde está o `pega_banana.ttt` (no repositório do Luciano, `IA368_ws/src/ia368_pkg/yolo_detector/`).
3. Opcional: copie o arquivo para esta pasta. Ele fica ignorado pelo git.

O modelo contém `/myRobot/{leftMotor,rightMotor,kinect,bumper,odometry,battery,dockingSensor,proximitySensor,...}`.

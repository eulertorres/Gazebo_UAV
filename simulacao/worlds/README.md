# TestBench.world

Este diretório contém o cenário **TestBench.world**, pensado para ligar rapidamente o motor "MN801S-150KV-APC2015-SAMA-FW25" com o plugin de motor brushless e o plugin de hélice. Abaixo está um passo a passo para abrir o mundo no Gazebo (WSL) e controlar o motor a partir do "Codex" (modelo Simulink rodando no Windows).

## 1. Preparar o ambiente no WSL
1. Abra o terminal na WSL e carregue o ambiente `catkin` com os plugins já compilados.
2. (Opcional) Exporte explicitamente o `GAZEBO_PLUGIN_PATH` caso os plugins estejam fora do caminho padrão:  
   ```bash
   export GAZEBO_PLUGIN_PATH=~/catkin_ws/devel/lib:$GAZEBO_PLUGIN_PATH
   ```
3. Inicie o Gazebo com o novo mundo:  
   ```bash
   gazebo --verbose simulacao/worlds/TestBench.world
   ```
   O motor será carregado no centro da cena com o chão e o sol padrão.

## 2. Conferir a porta UDP usada pelo plugin
1. Abra o arquivo do modelo `MN801S-150KV-APC2015-SAMA-FW25` (em `simulacao/models/.../model.sdf`).
2. Dentro da tag `<plugin>` procure pelos parâmetros `fdm_port_in`, `listen_addr` e `channel` para cada motor.
3. O valor padrão é `listen_addr=0.0.0.0` e `fdm_port_in=9002`. Anote a porta porque ela será usada no Simulink (Codex).

## 3. Configurar o Codex (Simulink no Windows)
1. No MATLAB/Simulink, abra ou crie o modelo responsável por gerar os PWMs.
2. Adicione um bloco **UDP Send** configurado para:
   - **Remote address**: IP da WSL (ver seção 4).
   - **Remote port**: porta `fdm_port_in` anotada anteriormente.
   - **Data type**: `single` (32 bits).
   - **Input vector size**: número de canais que deseja enviar (respeitando os `channel` definidos no SDF).
3. Converta seus sinais de PWM (em µs) para `single` e agrupe em um vetor na ordem dos canais.
4. Defina o *Sample time* do UDP Send (por exemplo, 0.01 s para 100 Hz).

## 4. Garantir a comunicação Windows ↔ WSL
1. No Windows, descubra o IP da WSL com `wsl hostname -I` ou `ipconfig`.
2. Caso o firewall bloqueie a porta UDP, libere a porta (ex.: 9002) tanto no Windows quanto na WSL.
3. Faça um teste rápido:  
   - No WSL rode `nc -lu 9002`.
   - No Windows envie dados com `PowerShell> echo hi | nc.exe -u <ip_da_wsl> 9002`.
   - Interrompa o teste com `Ctrl+C` em ambos.

## 5. Verificar no Gazebo
1. Com o Gazebo rodando, execute o Codex/Simulink e envie um PWM constante (ex.: 1500 µs).
2. Observe os logs do terminal do Gazebo: ao habilitar `debug` no SDF do motor, o plugin imprime os valores recebidos.
3. Se estiver usando ROS, abra outro terminal WSL e monitore `rostopic echo /motor_cmd` para confirmar a chegada dos PWMs.

## 6. Ajustar e salvar suas configurações
1. Para mudar o ponto inicial do motor, edite o bloco `<pose>` dentro do `<include>` do motor no `TestBench.world`.
2. Para adicionar mais motores, replique o `<include>` com nomes diferentes e ajuste os `channel` no SDF correspondente.
3. Guarde o modelo Simulink em um projeto separado e documente os canais/portas utilizados para repetir o ensaio facilmente.

Seguindo estes passos você consegue abrir o `TestBench.world` no Gazebo (WSL) e controlar o motor em tempo real usando o Codex (Simulink no Windows).

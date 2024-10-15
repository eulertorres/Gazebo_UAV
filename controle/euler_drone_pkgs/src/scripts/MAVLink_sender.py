import rospy
from pymavlink import mavutil
from std_msgs.msg import Float32
import time

# Função para conexão com MAVLink com tentativas de reconexão
def connect_to_mavlink():
    master = None
    while master is None:
        try:
            # Conecta ao ArduPilot via TCP na porta 5762 ou 5763 (ajuste conforme necessário)
            master = mavutil.mavlink_connection('tcp:127.0.0.1:5762')  # Ou use 5763 se preferir

            # Espera a conexão ser estabelecida (heartbeat)
            master.wait_heartbeat()
            print("Conexão estabelecida com o ArduPilot via MAVLink")
        except ConnectionRefusedError:
            print("[Errno 111] Deu ruim ao conectar no MAVLink, tentando novamente em 5 segundos...")
            time.sleep(5)  # Aguarda 5 segundos antes de tentar novamente
    return master

# Função para atualizar parâmetros no ArduPilot via MAVLink
def set_param(master, param_id, param_value):
    """Função para enviar o comando de ajuste de parâmetro via MAVLink."""
    master.mav.param_set_send(
        master.target_system,
        master.target_component,
        param_id.encode('utf-8'),
        param_value,
        mavutil.mavlink.MAV_PARAM_TYPE_REAL32
    )

    # Aguarda a confirmação da alteração do parâmetro
    while True:
        message = master.recv_match(type='PARAM_VALUE', blocking=True).to_dict()
        if message['param_id'] == param_id:
            print(f"Parâmetro {param_id} atualizado para {message['param_value']}")
            break

# Callback do tópico ROS para /battery/voltage
def voltage_callback(data):
    global mavlink_master  # Usar a conexão global

    voltage = data.data  # Obtém a tensão do tópico
    print(f"Tensão recebida: {voltage} V")

    # Atualiza o parâmetro SIM_BATT_VOLTAGE via MAVLink
    set_param(mavlink_master, 'SIM_BATT_VOLTAGE', voltage)

# Inicialização do nó ROS e assinatura do tópico /battery/voltage
def ros_listener():
    rospy.init_node('battery_voltage_updater', anonymous=True)

    # Assina o tópico /battery/voltage e define o callback para atualizar a tensão
    rospy.Subscriber("/battery/voltage", Float32, voltage_callback)

    # Mantém o nó em execução até que seja interrompido
    rospy.spin()

if __name__ == '__main__':
    try:
        # Estabelece a conexão constante com o MAVLink
        mavlink_master = connect_to_mavlink()

        # Inicia o listener ROS para escutar o tópico /battery/voltage
        ros_listener()

    except rospy.ROSInterruptException:
        pass

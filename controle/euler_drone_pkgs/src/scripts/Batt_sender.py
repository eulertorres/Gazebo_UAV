#!/usr/bin/env python3
import rospy
from std_msgs.msg import Float32
from pymavlink import mavutil

class BatteryToArduPilot:
    def __init__(self):
        # Inicializa o nó ROS
        rospy.init_node('battery_to_ardupilot', anonymous=True)

        # Configurações do MAVLink
        self.master = mavutil.mavlink_connection('udpout:127.0.0.1:14551')  # Porta onde o ArduPilot está escutando
        self.system_id = 1
        self.component_id = 1

        # Variáveis para armazenar os valores de tensão e corrente
        self.voltage = None
        self.current = None

        # Inscreve-se nos tópicos ROS
        rospy.Subscriber('/battery/voltage', Float32, self.voltage_callback)
        rospy.Subscriber('/battery/current', Float32, self.current_callback)

        # Frequência de envio das mensagens (em Hz)
        self.rate = rospy.Rate(10)  # 10 Hz

        # Loop principal
        self.run()

    def voltage_callback(self, msg):
        self.voltage = msg.data

    def current_callback(self, msg):
        self.current = msg.data

    def run(self):
        while not rospy.is_shutdown():
            if self.voltage is not None and self.current is not None:
                # Envia a mensagem BATTERY_STATUS
                self.send_battery_status()
            else:
                rospy.logwarn_throttle(10, "Aguardando valores de tensão e corrente...")
            self.rate.sleep()

    def send_battery_status(self):
        # Configura os parâmetros da mensagem BATTERY_STATUS
        battery_id = 0
        battery_function = mavutil.mavlink.MAV_BATTERY_FUNCTION_ALL
        battery_type = mavutil.mavlink.MAV_BATTERY_TYPE_LIPO
        temperature = 32767  # Temperatura não medida

        # Tensões das células (em mV)
        voltages = [int(self.voltage * 1000)] + [65535] * 9  # Preenche as demais células com 65535

        current_battery = int(self.current * 100)  # Em centiampères

        current_consumed = -1  # Não utilizado
        energy_consumed = -1   # Não utilizado
        battery_remaining = -1  # Não utilizado

        # Cria a mensagem BATTERY_STATUS
        msg = self.master.mav.battery_status_encode(
            battery_id,
            battery_function,
            battery_type,
            temperature,
            voltages,
            current_battery,
            current_consumed,
            energy_consumed,
            battery_remaining,
            0,  # Time remaining
            0,  # Charge state
            [],  # Voltages_ext
            0,  # Mode
            0   # Fault_bitmask
        )

        # Envia a mensagem
        self.master.mav.send(msg)
        rospy.loginfo_throttle(1, f"Enviando BATTERY_STATUS: Tensão={self.voltage}V, Corrente={self.current}A")

if __name__ == '__main__':
    try:
        BatteryToArduPilot()
    except rospy.ROSInterruptException:
        pass

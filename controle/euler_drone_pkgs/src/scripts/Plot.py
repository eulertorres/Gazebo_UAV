#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import pandas as pd
from geopy.distance import geodesic
from geopy import Point
from mavros_msgs.msg import Waypoint, WaypointList, State
from mavros_msgs.srv import WaypointPush, WaypointPushRequest
from sensor_msgs.msg import NavSatFix

# Caminho para o arquivo CSV
CSV_FILE_PATH = "dados-simulador.csv"

# Altitude padrão (em metros) para os waypoints
DEFAULT_RELATIVE_ALTITUDE = 2.0  # Você pode alterar este valor conforme necessário

# Classe para gerenciar a missão de waypoints
class MissionPlanner:
    def __init__(self):
        rospy.init_node('mission_planner', anonymous=True)

        print("Inicializando o Mission Planner...")

        # Variáveis para armazenar a posição atual do drone
        self.current_latitude = None
        self.current_longitude = None
        # Não precisamos mais da altitude absoluta
        # self.current_altitude = None

        # Publisher para publicar a missão
        self.waypoint_publisher = rospy.Publisher('/mavros/mission/waypoints', WaypointList, queue_size=10)

        # Serviço para enviar waypoints
        print("Aguardando pelo serviço /mavros/mission/push...")
        rospy.wait_for_service('/mavros/mission/push')
        try:
            self.push_waypoints = rospy.ServiceProxy('/mavros/mission/push', WaypointPush)
            print("Serviço /mavros/mission/push disponível.")
        except rospy.ServiceException as e:
            print("Serviço de envio de waypoints falhou: %s" % e)

        # Subscriber para o estado do drone
        self.current_state = None
        rospy.Subscriber('/mavros/state', State, self.state_callback)
        print("Subscrito ao tópico /mavros/state.")

        # Subscriber para a posição global atual do drone
        rospy.Subscriber('/mavros/global_position/global', NavSatFix, self.global_position_callback)
        print("Subscrito ao tópico /mavros/global_position/global.")

        # Aguarda o estado do drone estar conectado
        while not rospy.is_shutdown() and not self.current_state:
            print("Aguardando informações do estado do drone...")
            rospy.sleep(0.1)

        # Aguarda até receber a posição atual do drone
        while not rospy.is_shutdown() and (self.current_latitude is None or self.current_longitude is None):
            print("Aguardando a posição atual do drone...")
            rospy.sleep(0.1)

        print("Mission Planner Inicializado")

    def state_callback(self, msg):
        self.current_state = msg
        # Adiciona print para verificar o estado atual
        print("Estado do drone atualizado: %s" % self.current_state.mode)

    def global_position_callback(self, msg):
        self.current_latitude = msg.latitude
        self.current_longitude = msg.longitude
        # Não usamos mais a altitude absoluta do GPS
        # self.current_altitude = msg.altitude
        # Adiciona print para verificar a posição atual
        print("Posição atual do drone atualizada: lat=%.6f, lon=%.6f" %
              (self.current_latitude, self.current_longitude))

    def read_csv(self, filepath):
        try:
            df = pd.read_csv(filepath, delimiter=';')
            print("Arquivo CSV '%s' carregado com sucesso." % filepath)
            print("Dados do CSV:\n%s" % df.head())
            return df
        except Exception as e:
            print("Falha ao ler o arquivo CSV '%s': %s" % (filepath, e))
            return None

    def relative_to_gps(self, x, y, home_lat, home_lon):
        """
        Converte coordenadas relativas (x, y) em metros para latitude e longitude.
        """
        home_point = Point(home_lat, home_lon)
        # Deslocamento no eixo Norte e Leste
        north_offset = y  # metros
        east_offset = x   # metros

        # Calcula a nova latitude deslocada para o norte
        new_lat = geodesic(meters=north_offset).destination(home_point, 0).latitude
        # Calcula a nova longitude deslocada para o leste
        new_lon = geodesic(meters=east_offset).destination(home_point, 90).longitude

        # Adiciona print para verificar as coordenadas convertidas
        print("Coordenadas relativas (x: %.2f, y: %.2f) convertidas para GPS (lat: %.6f, lon: %.6f)" %
              (x, y, new_lat, new_lon))

        return new_lat, new_lon

    def generate_waypoints(self, df):
        print("Gerando waypoints a partir dos dados do CSV...")
        waypoints = []

        # Utiliza a posição atual do drone como home
        HOME_LATITUDE = self.current_latitude
        HOME_LONGITUDE = self.current_longitude
        HOME_ALTITUDE = DEFAULT_RELATIVE_ALTITUDE  # Usa a altitude relativa padrão

        # Waypoint de home
        home_wp = Waypoint()
        home_wp.frame = 3  # MAV_FRAME_GLOBAL_RELATIVE_ALT
        home_wp.command = 16  # MAV_CMD_NAV_WAYPOINT
        home_wp.is_current = True
        home_wp.autocontinue = True
        home_wp.param1 = 0  # Hold time in decimal seconds
        home_wp.param2 = 0  # Acceptance radius in meters
        home_wp.param3 = 0  # Pass radius in meters
        home_wp.param4 = 0  # Yaw angle
        home_wp.x_lat = HOME_LATITUDE
        home_wp.y_long = HOME_LONGITUDE
        home_wp.z_alt = HOME_ALTITUDE
        waypoints.append(home_wp)

        print("Waypoint inicial (Home) adicionado: %s" % home_wp)

        for index, row in df.iterrows():
            x = row['x[m]']
            y = row['y[m]']
            operacao = row.get('operacao', '')
            angulo = row.get('ang.abs[deg]', 0)
            # Adiciona print para cada linha do CSV processada
            print("Processando linha %d: x=%.2f, y=%.2f, operacao=%s, angulo=%.2f" %
                  (index, x, y, operacao, angulo))

            # Converter coordenadas relativas para GPS usando a posição atual como home
            lat, lon = self.relative_to_gps(x, y, HOME_LATITUDE, HOME_LONGITUDE)

            wp = Waypoint()
            wp.frame = 3  # MAV_FRAME_GLOBAL_RELATIVE_ALT
            wp.command = 16  # MAV_CMD_NAV_WAYPOINT
            wp.is_current = False
            wp.autocontinue = True
            wp.param1 = 0  # Hold time in decimal seconds
            wp.param2 = 5  # Acceptance radius in meters
            wp.param3 = 0  # Pass radius in meters
            wp.param4 = angulo  # Yaw angle
            wp.x_lat = lat
            wp.y_long = lon
            wp.z_alt = HOME_ALTITUDE  # Usa a altitude relativa padrão
            waypoints.append(wp)

            print("Waypoint %d adicionado: %s" % (index + 1, wp))

        print("Total de %d waypoints gerados." % len(waypoints))

        # Retornar uma lista de waypoints
        return waypoints

    def send_mission(self, waypoints):
        print("Enviando missão com %d waypoints..." % len(waypoints))
        req = WaypointPushRequest()
        req.start_index = 0
        req.waypoints = waypoints

        try:
            print("Chamando o serviço /mavros/mission/push...")
            response = self.push_waypoints(req)
            print("Resposta do serviço: %s" % response)
            if response.success:
                print("Waypoints enviados com sucesso. Waypoints transferidos: %d" % response.wp_transfered)
            else:
                print("Falha ao enviar waypoints. Erro: %s" % response)
        except rospy.ServiceException as e:
            print("Erro ao chamar o serviço de envio de waypoints: %s" % e)

    def run(self):
        df = self.read_csv(CSV_FILE_PATH)
        if df is None:
            print("Não foi possível carregar os dados do CSV. Encerrando o nó.")
            return

        waypoints = self.generate_waypoints(df)
        self.send_mission(waypoints)

        print("Missão concluída. Encerrando o nó.")
        rospy.signal_shutdown("Missão Enviada")

if __name__ == '__main__':
    try:
        planner = MissionPlanner()
        planner.run()
    except rospy.ROSInterruptException:
        pass

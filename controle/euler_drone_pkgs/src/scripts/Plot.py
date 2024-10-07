#!/usr/bin/env python

import rospy
import pandas as pd
import os
import threading
import queue
import time
from mavros_msgs.msg import Altitude, GlobalPositionTarget, RCIn, RCOut
from sensor_msgs.msg import NavSatFix, Imu
from nav_msgs.msg import Odometry
from geometry_msgs.msg import AccelWithCovarianceStamped, WrenchStamped, TwistStamped, PoseStamped
from openpyxl import Workbook, load_workbook

# Variáveis globais para armazenar os dados
topics = [
    '/mavros/altitude',
    '/mavros/global_position/global',
    '/mavros/global_position/local',
    '/mavros/global_position/raw/gps_vel',
    '/mavros/imu/data',
    #'/mavros/local_position/accel',
    '/mavros/local_position/pose',
    '/mavros/local_position/velocity_body',
    '/mavros/rc/in',
    '/mavros/rc/out',
    '/vento/drag',
    '/vento/resultante',
    '/vento/velocidade'
]

# Filas para armazenar os dados a serem escritos
data_queues = {topic: queue.Queue() for topic in topics}

# Evento para sinalizar a parada do thread de escrita
stop_event = threading.Event()

# Mapeamento de nomes de colunas para cada tópico
column_names = {
    '/vento/drag': ['timestamp', 'force_x', 'force_y', 'force_z'],
    '/vento/velocidade': ['timestamp', 'force_x', 'force_y', 'force_z'],
    '/vento/resultante': ['timestamp', 'force_x', 'force_y', 'force_z'],
    '/mavros/altitude': ['timestamp', 'amsl'],
    '/mavros/global_position/global': ['timestamp', 'latitude', 'longitude', 'altitude'],
    '/mavros/global_position/local': ['timestamp', 'position_x', 'position_y', 'position_z'],
    '/mavros/global_position/raw/gps_vel': ['timestamp', 'vel_x', 'vel_y', 'vel_z'],
    '/mavros/imu/data': ['timestamp', 'accel_x', 'accel_y', 'accel_z'],
    #'/mavros/local_position/accel': ['timestamp', 'accel_x', 'accel_y', 'accel_z'],
    '/mavros/local_position/pose': ['timestamp', 'pose_x', 'pose_y', 'pose_z'],
    '/mavros/local_position/velocity_body': ['timestamp', 'vel_x', 'vel_y', 'vel_z'],
    # Adicione outros tópicos se desejar nomes de colunas específicos
}

# Funções de callback para leitura de tópicos
def altitude_callback(msg):
    timestamp = rospy.get_time()
    data_row = [timestamp, msg.amsl]
    data_queues['/mavros/altitude'].put(data_row)

def global_position_callback(msg):
    timestamp = rospy.get_time()
    data_row = [timestamp, msg.latitude, msg.longitude, msg.altitude]
    data_queues['/mavros/global_position/global'].put(data_row)

def local_position_callback(msg):
    timestamp = rospy.get_time()
    data_row = [
        timestamp,
        msg.pose.pose.position.x,
        msg.pose.pose.position.y,
        msg.pose.pose.position.z
    ]
    data_queues['/mavros/global_position/local'].put(data_row)

def gps_velocity_callback(msg):
    timestamp = rospy.get_time()
    data_row = [
        timestamp,
        msg.twist.linear.x,
        msg.twist.linear.y,
        msg.twist.linear.z
    ]
    data_queues['/mavros/global_position/raw/gps_vel'].put(data_row)

def imu_data_callback(msg):
    timestamp = rospy.get_time()
    data_row = [
        timestamp,
        msg.linear_acceleration.x,
        msg.linear_acceleration.y,
        msg.linear_acceleration.z
    ]
    data_queues['/mavros/imu/data'].put(data_row)

def accel_callback(msg):
    timestamp = rospy.get_time()
    data_row = [
        timestamp,
        msg.accel.accel.linear.x,
        msg.accel.accel.linear.y,
        msg.accel.accel.linear.z
    ]
    data_queues['/mavros/local_position/accel'].put(data_row)

def pose_callback(msg):
    timestamp = rospy.get_time()
    data_row = [
        timestamp,
        msg.pose.position.x,
        msg.pose.position.y,
        msg.pose.position.z
    ]
    data_queues['/mavros/local_position/pose'].put(data_row)

def velocity_body_callback(msg):
    timestamp = rospy.get_time()
    data_row = [
        timestamp,
        msg.twist.linear.x,
        msg.twist.linear.y,
        msg.twist.linear.z
    ]
    data_queues['/mavros/local_position/velocity_body'].put(data_row)

def rc_in_callback(msg):
    timestamp = rospy.get_time()
    data_row = [timestamp] + list(msg.channels)
    data_queues['/mavros/rc/in'].put(data_row)

def rc_out_callback(msg):
    timestamp = rospy.get_time()
    data_row = [timestamp] + list(msg.channels)
    data_queues['/mavros/rc/out'].put(data_row)

def vento_drag_callback(msg):
    timestamp = rospy.get_time()
    data_row = [timestamp, msg.wrench.force.x, msg.wrench.force.y, msg.wrench.force.z]
    data_queues['/vento/drag'].put(data_row)

def vento_velocidade_callback(msg):
    timestamp = rospy.get_time()
    data_row = [timestamp, msg.wrench.force.x, msg.wrench.force.y, msg.wrench.force.z]
    data_queues['/vento/velocidade'].put(data_row)

def vento_resultante_callback(msg):
    timestamp = rospy.get_time()
    data_row = [timestamp, msg.wrench.force.x, msg.wrench.force.y, msg.wrench.force.z]
    data_queues['/vento/resultante'].put(data_row)

# Função para inicializar o arquivo Excel com as planilhas
def initialize_workbook(filename):
    wb = Workbook()
    # Remove a planilha padrão
    default_sheet = wb.active
    wb.remove(default_sheet)
    # Cria as planilhas para cada tópico
    for topic in topics:
        sheet_name = topic.replace('/', '_')[:31]
        ws = wb.create_sheet(title=sheet_name)
        # Escreve os cabeçalhos se estiverem definidos
        if topic in column_names:
            ws.append(column_names[topic])
    wb.save(filename)

# Função do thread de escrita
def writer_thread_func(filename, stop_event):
    while not stop_event.is_set():
        time.sleep(1)  # Espera 1 segundo
        wb = load_workbook(filename)
        for topic, q in data_queues.items():
            sheet_name = topic.replace('/', '_')[:31]
            ws = wb[sheet_name]
            rows_to_write = []
            while not q.empty():
                data_row = q.get()
                rows_to_write.append(data_row)
            if rows_to_write:
                for data_row in rows_to_write:
                    ws.append(data_row)
        wb.save(filename)
    # Escreve quaisquer dados restantes antes de sair
    wb = load_workbook(filename)
    for topic, q in data_queues.items():
        sheet_name = topic.replace('/', '_')[:31]
        ws = wb[sheet_name]
        rows_to_write = []
        while not q.empty():
            data_row = q.get()
            rows_to_write.append(data_row)
        if rows_to_write:
            for data_row in rows_to_write:
                ws.append(data_row)
    wb.save(filename)

# Função para monitorar a entrada do teclado
def monitor_keyboard():
    while not rospy.is_shutdown():
        user_input = input("Digite 'q' para sair e salvar o arquivo: ")
        if user_input == 'q':
            filename = input("Digite o nome do arquivo para salvar: ")
            # Sinaliza o thread de escrita para parar
            stop_event.set()
            writer_thread.join()
            # Renomeia o arquivo temp.xlsx para o nome escolhido
            os.rename('temp.xlsx', f"{filename}.xlsx")
            print(f"Dados salvos no arquivo {filename}")
            # Chama a função para gerar o gráfico interativo
            #interactive_plot(filename)
            rospy.signal_shutdown("Usuário encerrou o programa")
            break

# Função para gerar o gráfico interativo
def interactive_plot(filename):
    # Lê o arquivo Excel
    xls = pd.ExcelFile(filename)
    sheet_names = xls.sheet_names

    # Mapeia as colunas disponíveis
    data_columns = {}
    for sheet_name in sheet_names:
        df = pd.read_excel(xls, sheet_name)
        columns = list(df.columns)
        if 'timestamp' in columns:
            columns.remove('timestamp')
        if columns:  # Só adiciona se houver colunas além do timestamp
            data_columns[sheet_name] = columns

    # Exibe as opções disponíveis
    print("\nColunas disponíveis para plotagem:")
    idx = 1
    options = []
    for sheet_name, columns in data_columns.items():
        for col in columns:
            options.append((sheet_name, col))
            print(f"{idx}. {sheet_name} - {col}")
            idx += 1

    # Verifica se há opções disponíveis
    if not options:
        print("Não há dados disponíveis para plotagem.")
        return

    # Solicita ao usuário que selecione uma opção
    while True:
        selection = input("Selecione uma coluna de dados para plotar (digite o número ou 's' para sair): ")
        if selection.lower() == 's':
            print("Encerrando a plotagem interativa.")
            return
        try:
            selection = int(selection)
            if 1 <= selection <= len(options):
                break
            else:
                print("Seleção inválida. Tente novamente.")
        except ValueError:
            print("Entrada inválida. Digite um número ou 's' para sair.")

    selected_sheet, selected_column = options[selection - 1]

    # Lê os dados selecionados
    df = pd.read_excel(filename, sheet_name=selected_sheet)

    # Verifica se 'timestamp' está presente
    if 'timestamp' in df.columns:
        x = df['timestamp']
    else:
        print("Não há timestamp nos dados selecionados.")
        return

    y = df[selected_column]

    # Plota os dados
    plt.figure()
    plt.plot(x, y)
    plt.xlabel('Timestamp')
    plt.ylabel(selected_column)
    plt.title(f"{selected_sheet} - {selected_column} vs Timestamp")
    plt.grid(True)
    plt.show()

# Função para inicializar o nó ROS e os subscritores
def listener():
    global writer_thread
    rospy.init_node('data_recorder', anonymous=True)

    # Inicializa o arquivo Excel
    initialize_workbook('temp.xlsx')

    # Inicia o thread de escrita
    writer_thread = threading.Thread(target=writer_thread_func, args=('temp.xlsx', stop_event))
    writer_thread.start()

    rospy.Subscriber("/mavros/global_position/global", NavSatFix, global_position_callback)
    rospy.Subscriber("/mavros/global_position/local", Odometry, local_position_callback)
    #rospy.Subscriber("/mavros/local_position/accel", AccelWithCovarianceStamped, accel_callback)
    rospy.Subscriber("/vento/drag", WrenchStamped, vento_drag_callback)
    rospy.Subscriber("/vento/resultante", WrenchStamped, vento_resultante_callback)
    rospy.Subscriber("/vento/velocidade", WrenchStamped, vento_velocidade_callback)
    rospy.Subscriber("/mavros/altitude", Altitude, altitude_callback)
    rospy.Subscriber("/mavros/global_position/raw/gps_vel", TwistStamped, gps_velocity_callback)
    rospy.Subscriber("/mavros/imu/data", Imu, imu_data_callback)
    rospy.Subscriber("/mavros/local_position/pose", PoseStamped, pose_callback)
    rospy.Subscriber("/mavros/local_position/velocity_body", TwistStamped, velocity_body_callback)
    rospy.Subscriber("/mavros/rc/in", RCIn, rc_in_callback)
    rospy.Subscriber("/mavros/rc/out", RCOut, rc_out_callback)

    # Thread para monitorar entrada do teclado
    keyboard_thread = threading.Thread(target=monitor_keyboard)
    keyboard_thread.start()

    # Define a taxa de amostragem
    rate = rospy.Rate(10)  # 10 Hz

    while not rospy.is_shutdown():
        rate.sleep()

if __name__ == '__main__':
    try:
        listener()
    except rospy.ROSInterruptException:
        pass

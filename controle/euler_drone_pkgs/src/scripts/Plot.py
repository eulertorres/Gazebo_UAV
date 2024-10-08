#!/usr/bin/env python

import rospy
import pandas as pd
import os
import threading
import time
import subprocess
import signal
import rosbag
import openpyxl
from openpyxl import Workbook
from openpyxl.chart import LineChart, Reference, Series
from mavros_msgs.msg import Altitude, GlobalPositionTarget, RCIn, RCOut
from sensor_msgs.msg import NavSatFix, Imu
from nav_msgs.msg import Odometry
from geometry_msgs.msg import AccelWithCovarianceStamped, WrenchStamped, TwistStamped, PoseStamped

# Lista de tópicos para gravar
topics = [
    '/mavros/altitude',
    '/mavros/global_position/global',
    '/mavros/global_position/local',
    '/mavros/global_position/raw/gps_vel',
    '/mavros/imu/data',
    '/mavros/local_position/accel',
    '/mavros/local_position/pose',
    '/mavros/local_position/velocity_body',
    '/mavros/rc/in',
    '/mavros/rc/out',
    '/vento/drag',
    '/vento/resultante',
    '/vento/velocidade'
]

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
    '/mavros/local_position/accel': ['timestamp', 'accel_x', 'accel_y', 'accel_z'],
    '/mavros/local_position/pose': ['timestamp', 'pose_x', 'pose_y', 'pose_z'],
    '/mavros/local_position/velocity_body': ['timestamp', 'vel_x', 'vel_y', 'vel_z'],
    # Outros tópicos podem ser adicionados aqui
}

# Variável global para controlar a gravação
rosbag_process = None

# Função para iniciar a gravação com rosbag
def start_rosbag_record():
    global rosbag_process
    # Cria o diretório logs se não existir
    if not os.path.exists('logs'):
        os.makedirs('logs')
    # Define o nome do arquivo de log com timestamp
    bag_filename = 'logs/dados_{}.bag'.format(int(time.time()))
    # Comando para gravar os tópicos especificados
    command = ['rosbag', 'record', '-O', bag_filename] + topics
    # Inicia o processo de gravação
    rosbag_process = subprocess.Popen(command)
    print("Gravação iniciada com rosbag. Arquivo: {}".format(bag_filename))
    return bag_filename

# Função para parar a gravação com rosbag
def stop_rosbag_record():
    global rosbag_process
    if rosbag_process:
        rosbag_process.send_signal(signal.SIGINT)
        rosbag_process.wait()
        print("Gravação com rosbag finalizada.")
        rosbag_process = None

# Função para monitorar a entrada do teclado
def monitor_keyboard(bag_filename):
    while not rospy.is_shutdown():
        user_input = input("Digite 'q' para parar a gravação e salvar os dados: ")
        if user_input == 'q':
            stop_rosbag_record()
            filename = input("Digite o nome do arquivo Excel para salvar os dados: ")
            # Verifica se o nome do arquivo termina com .xlsx, se não, adiciona a extensão
            if not filename.endswith('.xlsx'):
                filename += '.xlsx'
            # Processa o arquivo .bag e salva os dados no Excel
            process_bag_file_incremental(bag_filename, filename)
            rospy.signal_shutdown("Usuário encerrou o programa")
            break

# Função para processar o arquivo .bag de forma incremental
def process_bag_file_incremental(bag_filename, excel_filename):
    print("Processando o arquivo .bag e extraindo os dados...")
    bag = rosbag.Bag(bag_filename)

    # Cria um novo arquivo Excel
    writer = pd.ExcelWriter(excel_filename, engine='xlsxwriter')
    workbook = writer.book

    for topic in topics:
        print(f"Processando tópico: {topic}")
        # Inicializa um gerador para ler as mensagens do tópico
        msg_generator = bag.read_messages(topics=[topic])
        # Processa e escreve os dados em pequenos chunks para evitar uso excessivo de memória
        data_chunk = []
        chunk_size = 1000  # Ajuste o tamanho do chunk conforme necessário

        # Obter os nomes das colunas
        if topic in column_names:
            columns = column_names[topic]
        else:
            # Para os tópicos de RC, gerar colunas dinamicamente
            # Precisamos ler uma mensagem para determinar o número de canais
            try:
                _, first_msg, _ = next(bag.read_messages(topics=[topic]))
                num_channels = len(first_msg.channels)
                columns = ['timestamp'] + [f'channel_{i+1}' for i in range(num_channels)]
            except StopIteration:
                # Se não houver mensagens no tópico, continue para o próximo
                continue

        # Cria uma planilha para o tópico
        sheet_name = topic.replace('/', '_')[:31]
        worksheet = workbook.add_worksheet(sheet_name)
        writer.sheets[sheet_name] = worksheet

        # Escreve os cabeçalhos
        worksheet.write_row(0, 0, columns)
        row_index = 1  # Começa na segunda linha

        for _, msg, t in msg_generator:
            timestamp = t.to_sec()
            if topic == '/mavros/altitude':
                data_row = [timestamp, msg.amsl]
            elif topic == '/mavros/global_position/global':
                data_row = [timestamp, msg.latitude, msg.longitude, msg.altitude]
            elif topic == '/mavros/global_position/local':
                data_row = [
                    timestamp,
                    msg.pose.pose.position.x,
                    msg.pose.pose.position.y,
                    msg.pose.pose.position.z
                ]
            elif topic == '/mavros/global_position/raw/gps_vel':
                data_row = [
                    timestamp,
                    msg.twist.linear.x,
                    msg.twist.linear.y,
                    msg.twist.linear.z
                ]
            elif topic == '/mavros/imu/data':
                data_row = [
                    timestamp,
                    msg.linear_acceleration.x,
                    msg.linear_acceleration.y,
                    msg.linear_acceleration.z
                ]
            elif topic == '/mavros/local_position/accel':
                data_row = [
                    timestamp,
                    msg.accel.accel.linear.x,
                    msg.accel.accel.linear.y,
                    msg.accel.accel.linear.z
                ]
            elif topic == '/mavros/local_position/pose':
                data_row = [
                    timestamp,
                    msg.pose.position.x,
                    msg.pose.position.y,
                    msg.pose.position.z
                ]
            elif topic == '/mavros/local_position/velocity_body':
                data_row = [
                    timestamp,
                    msg.twist.linear.x,
                    msg.twist.linear.y,
                    msg.twist.linear.z
                ]
            elif topic == '/vento/drag' or topic == '/vento/velocidade' or topic == '/vento/resultante':
                data_row = [
                    timestamp,
                    msg.wrench.force.x,
                    msg.wrench.force.y,
                    msg.wrench.force.z
                ]
            elif topic == '/mavros/rc/in' or topic == '/mavros/rc/out':
                data_row = [timestamp] + list(msg.channels)
            else:
                continue  # Pula se o tópico não estiver mapeado

            data_chunk.append(data_row)

            # Se o chunk atingir o tamanho definido, escreve no arquivo e limpa o chunk
            if len(data_chunk) >= chunk_size:
                for data in data_chunk:
                    worksheet.write_row(row_index, 0, data)
                    row_index +=1
                data_chunk = []

        # Escreve quaisquer dados restantes
        if data_chunk:
            for data in data_chunk:
                worksheet.write_row(row_index, 0, data)
                row_index +=1

        # Após escrever os dados, podemos adicionar um gráfico simples
        # Verifica se há dados suficientes para criar um gráfico
        if row_index > 2:
            chart = workbook.add_chart({'type': 'line'})
            # O eixo X é o timestamp (coluna A)
            # O eixo Y é a segunda coluna (primeiro dado após o timestamp)
            chart.add_series({
                'categories': [sheet_name, 1, 0, row_index -1, 0],  # timestamp
                'values':     [sheet_name, 1, 1, row_index -1, 1],  # primeiro dado
                'name': columns[1],
            })
            chart.set_title({'name': f'{columns[1]} vs Time'})
            chart.set_x_axis({'name': 'Timestamp'})
            chart.set_y_axis({'name': columns[1]})
            # Insere o gráfico na planilha
            worksheet.insert_chart('G2', chart)

    writer.close()
    bag.close()
    print("Dados salvos no arquivo Excel: {}".format(excel_filename))

# Função principal
def main():
    rospy.init_node('data_recorder_rosbag', anonymous=True)

    # Inicia a gravação com rosbag
    bag_filename = start_rosbag_record()

    # Thread para monitorar entrada do teclado
    keyboard_thread = threading.Thread(target=monitor_keyboard, args=(bag_filename,))
    keyboard_thread.start()

    rospy.spin()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass

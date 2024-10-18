import sys
import os
import signal
from PyQt5.QtWidgets import (
    QApplication, QMainWindow, QWidget, QPushButton,
    QVBoxLayout, QHBoxLayout, QTextEdit, QTabWidget, QLineEdit,
    QLabel, QSplitter, QSizePolicy, QComboBox, QCheckBox, QMessageBox
)
from PyQt5.QtCore import QProcess, Qt
from PyQt5.QtGui import QFont, QIcon, QPixmap, QTextCursor

class TerminalTab(QWidget):
    def __init__(self, label, cmd, process_type, parent=None):
        super().__init__(parent)
        self.label = label
        self.cmd = cmd
        self.process_type = process_type
        self.process = QProcess(self)
        self.process.setProcessChannelMode(QProcess.MergedChannels)
        self.pid = None  # Armazenar o PID do processo

        # Layouts
        layout = QVBoxLayout()
        self.output = QTextEdit()
        self.output.setReadOnly(True)
        # Melhorar a aparência do terminal
        self.output.setStyleSheet("""
            background-color: #1e1e1e;
            color: #d4d4d4;
            font-family: Consolas, 'Courier New', monospace;
            font-size: 12px;
        """)
        font = QFont("Consolas")
        font.setStyleHint(QFont.Monospace)
        self.output.setFont(font)

        self.input_line = QLineEdit()
        self.input_line.returnPressed.connect(self.send_input)
        self.input_line.setStyleSheet("""
            background-color: #2d2d2d;
            color: #d4d4d4;
            border: none;
            padding: 5px;
            font-family: Consolas, 'Courier New', monospace;
            font-size: 12px;
        """)

        layout.addWidget(self.output)
        layout.addWidget(QLabel("Digite um comando e pressione Enter:"))
        layout.addWidget(self.input_line)
        self.setLayout(layout)

        # Conectar sinais
        self.process.readyRead.connect(self.update_output)
        self.process.finished.connect(self.process_finished)
        self.process.started.connect(self.process_started)

        # Iniciar o processo
        self.start_process()

    def start_process(self):
        self.process.start("bash", ["-c", self.cmd])

    def process_started(self):
        self.pid = self.process.processId()
        self.output.append(f"Processo iniciado com PID: {self.pid}")

    def send_input(self):
        text = self.input_line.text()
        self.input_line.clear()
        self.process.write((text + '\n').encode())

    def update_output(self):
        output = self.process.readAll().data().decode()
        self.output.append(output)
        # Auto-scroll para o final
        self.output.moveCursor(QTextCursor.End)

    def process_finished(self):
        self.output.append(f"\nProcesso '{self.label}' foi finalizado.")

class MainWindow(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("SIMULADOR RW")
        self.resize(1200, 800)
        self.initUI()
        # Definir o ícone do programa
        icon_path = os.path.join(os.path.dirname(__file__), 'utilidades/resources/gato.jpg')
        self.setWindowIcon(QIcon(icon_path))

    def initUI(self):
        # Aplicar tema claro com letras pretas
        self.setStyleSheet("""
            QMainWindow {
                background-color: #f0f0f0;
                color: #000000;
            }
            QLabel, QPushButton, QLineEdit, QComboBox, QCheckBox {
                color: #000000;
            }
            QPushButton {
                background-color: #dcdcdc;
                border: none;
                padding: 5px;
            }
            QPushButton:hover {
                background-color: #c0c0c0;
            }
            QPushButton:pressed {
                background-color: #a0a0a0;
            }
            QTabWidget::pane {
                border: 1px solid #444;
            }
            QTabBar::tab {
                background: #dcdcdc;
                padding: 5px;
            }
            QTabBar::tab:selected {
                background: #f0f0f0;
            }
        """)

        # Layouts
        main_layout = QVBoxLayout()
        top_layout = QHBoxLayout()
        left_layout = QVBoxLayout()
        output_layout = QVBoxLayout()

        # Adicionar título, versão e imagem
        title_label = QLabel("SIMULADOR RW")
        title_label.setStyleSheet("font-size: 24px; font-weight: bold;")
        title_label.setAlignment(Qt.AlignCenter)

        version_label = QLabel("Versão 1.0")
        version_label.setAlignment(Qt.AlignLeft)

        image_label = QLabel()
        image_path = os.path.join(os.path.dirname(__file__), 'utilidades/resources/gato.jpg')
        pixmap = QPixmap(image_path)
        image_label.setPixmap(pixmap.scaled(100, 100, Qt.KeepAspectRatio))

        title_container = QHBoxLayout()
        title_container.addWidget(version_label)
        title_container.addStretch()
        title_container.addWidget(title_label)
        title_container.addStretch()
        title_container.addWidget(image_label)

        # Dropdowns
        self.ambiente_combo = QComboBox()
        self.ambiente_combo.addItems(["pista_vazia"])  # Por enquanto, apenas 'pista_vazia'
        self.ambiente_label = QLabel("Selecione o Ambiente (Mundo):")

        self.aeronave_combo = QComboBox()
        self.aeronave_combo.addItems(["EASy", "T30", "T30_barra", "T30_estavel"])
        self.aeronave_label = QLabel("Selecione a Aeronave:")

        # Botões On/Off para Console e Mapa
        self.console_checkbox = QCheckBox("Console")
        self.console_checkbox.setChecked(True)
        self.mapa_checkbox = QCheckBox("Mapa")
        self.mapa_checkbox.setChecked(True)

        # Caixas de texto para parâmetros
        self.j_label = QLabel("Número de núcleos:")
        self.j_input = QLineEdit("8")
        self.lat_label = QLabel("Latitude:")
        self.lat_input = QLineEdit("-22.013852")
        self.lon_label = QLabel("Longitude:")
        self.lon_input = QLineEdit("-47.947333,858,0")

        # Adicionar widgets ao layout esquerdo
        left_layout.addWidget(self.ambiente_label)
        left_layout.addWidget(self.ambiente_combo)
        left_layout.addWidget(self.aeronave_label)
        left_layout.addWidget(self.aeronave_combo)
        left_layout.addWidget(self.console_checkbox)
        left_layout.addWidget(self.mapa_checkbox)
        left_layout.addWidget(self.j_label)
        left_layout.addWidget(self.j_input)
        left_layout.addWidget(self.lat_label)
        left_layout.addWidget(self.lat_input)
        left_layout.addWidget(self.lon_label)
        left_layout.addWidget(self.lon_input)

        # Botões e suas funções
        self.tabs = QTabWidget()
        self.process_tabs = {}

        # Estado dos botões
        self.buttons_state = {
            "Gazebo": False,
            "Ardupilot": False
        }

        actions = [
            ("Abrir ambiente Gazebo", self.start_gazebo, "gazebo"),
            ("Iniciar Ardupilot", self.start_ardupilot, "ardupilot"),
            ("Abrir MissionPlanner", self.start_mission_planner, "mission_planner"),
            ("Carregar Joystick", self.load_joystick, "joystick"),
            ("Plot Tempo Real", self.start_real_time_plot, "plot_tempo_real"),
            ("Script Avionica", self.start_avionics_script, "script_avionica"),
            ("Enviar Missão", self.send_mission, "enviar_missao"),
            ("Plot Excel", self.plot_excel, "plot_excel")
        ]

        self.buttons = {}

        for label, func, key in actions:
            btn_layout = QHBoxLayout()
            start_btn = QPushButton(label)
            start_btn.clicked.connect(func)
            start_btn.setEnabled(True)  # Ajustaremos a habilitação posteriormente

            stop_btn = QPushButton("Encerrar")
            stop_btn.setStyleSheet("background-color: red; color: black;")
            stop_btn.clicked.connect(lambda _, l=label: self.stop_process(l))

            self.buttons[key] = start_btn

            btn_layout.addWidget(start_btn)
            btn_layout.addWidget(stop_btn)
            left_layout.addLayout(btn_layout)

        # Botão para encerrar tudo e fechar o programa
        self.exit_btn = QPushButton("Encerrar Tudo e Sair")
        self.exit_btn.setStyleSheet("background-color: darkred; color: black;")
        self.exit_btn.clicked.connect(self.close_application)
        left_layout.addWidget(self.exit_btn)

        # Ajustar habilitação inicial dos botões
        self.buttons["script_avionica"].setEnabled(False)
        self.buttons["joystick"].setEnabled(False)
        self.buttons["enviar_missao"].setEnabled(False)

        # Área de saída (abas com terminais simulados)
        output_layout.addWidget(self.tabs)

        # Adicionar a aba de comandos do Ardupilot
        self.create_ardupilot_commands_tab()

        # Configuração do layout principal
        top_layout.addLayout(left_layout)
        top_layout.addLayout(output_layout)

        # Usar QSplitter para permitir redimensionamento
        splitter = QSplitter(Qt.Vertical)
        top_widget = QWidget()
        top_widget.setLayout(top_layout)
        splitter.addWidget(top_widget)

        main_layout.addLayout(title_container)
        main_layout.addWidget(splitter)

        container = QWidget()
        container.setLayout(main_layout)
        self.setCentralWidget(container)

    def create_ardupilot_commands_tab(self):
        self.ardupilot_commands_tab = QWidget()
        layout = QVBoxLayout()

        # Caixa de texto para o valor de X no Takeoff
        self.takeoff_label = QLabel("Altitude de Decolagem (m):")
        self.takeoff_input = QLineEdit("2")

        takeoff_layout = QHBoxLayout()
        takeoff_btn = QPushButton("Takeoff")
        takeoff_btn.clicked.connect(self.takeoff_command)
        takeoff_layout.addWidget(takeoff_btn)
        takeoff_layout.addWidget(self.takeoff_label)
        takeoff_layout.addWidget(self.takeoff_input)

        commands = [
            ("Carregar Joystick", self.load_joystick),
            ("Listar Missões", lambda: self.send_ardupilot_command('wp list')),
            ("Limpar Missões", lambda: self.send_ardupilot_command('wp clear')),
            ("Modo Guided", lambda: self.send_ardupilot_command('mode guided')),
            ("Modo Auto", lambda: self.send_ardupilot_command('mode auto')),
            ("RTL", lambda: self.send_ardupilot_command('rtl')),
            ("Loiter", lambda: self.send_ardupilot_command('mode loiter')),
            ("Stabilize", lambda: self.send_ardupilot_command('mode stabilize')),
            ("Arm Throttle", lambda: self.send_ardupilot_command('arm throttle')),
            # Adicione mais comandos conforme necessário
        ]

        for label, func in commands:
            btn = QPushButton(label)
            btn.clicked.connect(func)
            layout.addWidget(btn)

        layout.addLayout(takeoff_layout)

        self.ardupilot_commands_tab.setLayout(layout)
        self.tabs.addTab(self.ardupilot_commands_tab, "Comandos Ardupilot")
        self.tabs.setTabEnabled(self.tabs.indexOf(self.ardupilot_commands_tab), False)

    def takeoff_command(self):
        altitude = self.takeoff_input.text()
        self.send_ardupilot_command(f'takeoff {altitude}')

    def send_ardupilot_command(self, command):
        if 'Ardupilot' in self.process_tabs:
            tab_info = self.process_tabs['Ardupilot']
            tab = tab_info['tab']
            tab.process.write((command + '\n').encode())
        else:
            QMessageBox.warning(self, "Aviso", "Ardupilot não está em execução.")

    def start_gazebo(self):
        label = "Gazebo"
        ambiente = self.ambiente_combo.currentText()
        aeronave = self.aeronave_combo.currentText()
        cmd = f'roslaunch simulacao simulacao.launch model_name:={aeronave} world:={ambiente} 2>&1 | grep -v "DS: no mapping for sensor id"'
        self.start_process(label, cmd, process_type='gazebo')
        # Habilitar o botão Script Avionica
        self.buttons["script_avionica"].setEnabled(True)
        self.buttons_state["Gazebo"] = True

    def start_ardupilot(self):
        label = "Ardupilot"
        aeronave = self.aeronave_combo.currentText()
        home_dir = os.path.expanduser("~")
        cwd = os.path.join(home_dir, f"catkin_ws/src/Euler_Drone_Sim/simulacao/models/{aeronave}/Copter")
        # Parâmetros configuráveis
        console_flag = "--console" if self.console_checkbox.isChecked() else ""
        map_flag = "--map" if self.mapa_checkbox.isChecked() else ""
        j_value = self.j_input.text()
        lat_value = self.lat_input.text()
        lon_value = self.lon_input.text()
        cmd = f"cd {cwd} && sim_vehicle.py -f gazebo-iris --speedup 20 -j {j_value} -v ArduCopter {console_flag} {map_flag} -l {lat_value},{lon_value}"
        self.start_process(label, cmd, process_type='ardupilot')
        # Habilitar o botão Carregar Joystick e comandos do Ardupilot
        self.buttons["joystick"].setEnabled(True)
        self.tabs.setTabEnabled(self.tabs.indexOf(self.ardupilot_commands_tab), True)
        self.buttons_state["Ardupilot"] = True

    def load_joystick(self):
        self.send_ardupilot_command('module load joystick')
        # Habilitar o botão Enviar Missão
        self.buttons["enviar_missao"].setEnabled(True)

    def start_mission_planner(self):
        label = "MissionPlanner"
        home_dir = os.path.expanduser("~")
        cwd = os.path.join(home_dir, "Desktop/MissionPlanner")
        cmd = f"cd {cwd} && mono MissionPlanner.exe"
        self.start_process(label, cmd, process_type='mission_planner')

    def start_real_time_plot(self):
        label = "Plot Tempo Real"
        cmd = "rqt"
        self.start_process(label, cmd, process_type='script')

    def start_avionics_script(self):
        label = "Script Avionica"
        if not self.buttons_state.get("Gazebo", False):
            QMessageBox.warning(self, "Aviso", "Por favor, inicie o Gazebo primeiro.")
            return
        home_dir = os.path.expanduser("~")
        cwd = os.path.join(home_dir, "catkin_ws/src/Euler_Drone_Sim/controle/euler_drone_pkgs/src/scripts")
        cmd = f"cd {cwd} && python MAVLink_sender.py"
        self.start_process(label, cmd, process_type='script')

    def send_mission(self):
        label = "Enviar Missão"
        if not self.buttons_state.get("Ardupilot", False):
            QMessageBox.warning(self, "Aviso", "Por favor, inicie o Ardupilot primeiro.")
            return
        home_dir = os.path.expanduser("~")
        cwd = os.path.join(home_dir, "catkin_ws/src/Euler_Drone_Sim/controle/euler_drone_pkgs/src/scripts")
        cmd = f"cd {cwd} && python Mission_sender.py"
        self.start_process(label, cmd, process_type='script')

    def plot_excel(self):
        label = "Plot Excel"
        home_dir = os.path.expanduser("~")
        cwd = os.path.join(home_dir, "catkin_ws/src/Euler_Drone_Sim/controle/euler_drone_pkgs/src/scripts")
        cmd = f"cd {cwd} && python Plot.py"
        self.start_process(label, cmd, process_type='script')

    def start_process(self, label, cmd, process_type):
        if label in self.process_tabs:
            QMessageBox.information(self, "Processo em Execução", f"{label} já está em execução.")
            return
        tab = TerminalTab(label, cmd, process_type)
        self.tabs.addTab(tab, label)
        self.process_tabs[label] = {'tab': tab, 'process_type': process_type}

    def stop_process(self, label):
        if label in self.process_tabs:
            tab_info = self.process_tabs[label]
            tab = tab_info['tab']
            process_type = tab_info['process_type']
            # Remover a aba do terminal
            index = self.tabs.indexOf(tab)
            self.tabs.removeTab(index)
            del self.process_tabs[label]
            # Encerra o processo adequadamente
            if process_type == 'gazebo':
                # Encerra o processo Gazebo
                os.system('killall -9 gzserver')
                os.system('killall -9 gzclient')
                # Enviar SIGINT para o processo
                if tab.pid:
                    os.kill(tab.pid, signal.SIGINT)
            elif process_type == 'mission_planner':
                # Fechar o MissionPlanner
                tab.process.terminate()
            elif process_type == 'ardupilot':
                # Fechar o Ardupilot e seus terminais
                tab.process.terminate()
                #os.system('pkill -f sim_vehicle.py')
                #os.system('pkill -f mavproxy.py')
            elif process_type == 'script':
                # Enviar sinal Ctrl+C
                tab.process.terminate()
            else:
                tab.process.terminate()
            tab.process.waitForFinished()
            # Desabilitar a aba de comandos do Ardupilot se necessário
            if label == 'Ardupilot':
                self.tabs.setTabEnabled(self.tabs.indexOf(self.ardupilot_commands_tab), False)
                self.buttons["joystick"].setEnabled(False)
                self.buttons["enviar_missao"].setEnabled(False)
                self.buttons_state["Ardupilot"] = False
        else:
            QMessageBox.information(self, "Processo Não Encontrado", f"{label} não está em execução.")

    def close_application(self):
        # Encerrar todos os processos
        for label in list(self.process_tabs.keys()):
            self.stop_process(label)
        # Fechar o aplicativo
        self.close()

    def closeEvent(self, event):
        # Sobrescrever o evento de fechamento para encerrar os processos
        self.close_application()
        event.accept()

if __name__ == "__main__":
    app = QApplication(sys.argv)
    window = MainWindow()
    window.show()
    sys.exit(app.exec_())

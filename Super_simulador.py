import sys
import os
from PyQt5.QtWidgets import (
    QApplication, QMainWindow, QWidget, QPushButton,
    QVBoxLayout, QHBoxLayout, QTextEdit, QTabWidget, QLineEdit,
    QLabel, QSplitter, QSizePolicy, QComboBox, QCheckBox, QMessageBox
)
from PyQt5.QtCore import QProcess, Qt, QEvent
from PyQt5.QtGui import QFont, QIcon, QPixmap

class TerminalTab(QWidget):
    def __init__(self, label, cmd, parent=None):
        super().__init__(parent)
        self.label = label
        self.cmd = cmd
        self.process = QProcess(self)
        self.process.setProcessChannelMode(QProcess.MergedChannels)
        self.pid = None  # Armazenar o PID do processo

        # Layouts
        layout = QVBoxLayout()
        self.output = QTextEdit()
        self.output.setReadOnly(True)
        self.output.setStyleSheet("background-color: black; color: white;")
        font = QFont("Monospace")
        font.setStyleHint(QFont.TypeWriter)
        self.output.setFont(font)

        self.input_line = QLineEdit()
        self.input_line.returnPressed.connect(self.send_input)

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

    def process_finished(self):
        self.output.append(f"\nProcesso '{self.label}' foi finalizado.")

    def stop_process(self):
        if self.pid:
            confirm = QMessageBox.question(
                self, "Encerrar Processo",
                f"Deseja encerrar o processo '{self.label}' com PID {self.pid}?",
                QMessageBox.Yes | QMessageBox.No, QMessageBox.No
            )
            if confirm == QMessageBox.Yes:
                os.kill(self.pid, 9)  # Enviar sinal SIGKILL
                self.process.waitForFinished()
                self.pid = None
        else:
            self.process.terminate()
            self.process.waitForFinished()

class MainWindow(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("Simulador do RW")
        self.resize(1200, 800)
        self.initUI()
        # Definir o ícone do programa
        icon_path = os.path.join(os.path.dirname(__file__), 'utilidades/resources/gato.jpg')
        self.setWindowIcon(QIcon(icon_path))

    def initUI(self):
        # Aplicar estilo cinza escuro
        self.setStyleSheet("""
            QMainWindow {
                background-color: #2b2b2b;
                color: #ffffff;
            }
            QLabel, QPushButton, QLineEdit, QComboBox {
                color: #ffffff;
            }
            QPushButton {
                background-color: #3c3c3c;
                border: none;
                padding: 5px;
            }
            QPushButton:hover {
                background-color: #505050;
            }
            QPushButton:pressed {
                background-color: #2b2b2b;
            }
            QTabWidget::pane {
                border: 1px solid #444;
            }
            QTabBar::tab {
                background: #3c3c3c;
                padding: 5px;
            }
            QTabBar::tab:selected {
                background: #2b2b2b;
            }
        """)

        # Layouts
        main_layout = QVBoxLayout()
        top_layout = QHBoxLayout()
        left_layout = QVBoxLayout()
        output_layout = QVBoxLayout()

        # Adicionar título e imagem
        title_layout = QHBoxLayout()
        title_label = QLabel("Simulador do RW")
        title_label.setStyleSheet("font-size: 24px; font-weight: bold;")
        image_label = QLabel()
        image_path = os.path.join(os.path.dirname(__file__), 'utilidades/resources/gato.jpg')
        pixmap = QPixmap(image_path)
        image_label.setPixmap(pixmap.scaled(100, 100, Qt.KeepAspectRatio))
        title_layout.addWidget(title_label)
        title_layout.addStretch()
        title_layout.addWidget(image_label)

        # Dropdowns
        self.ambiente_combo = QComboBox()
        self.ambiente_combo.addItems(["EASy", "T30", "Pista vazia"])
        self.ambiente_label = QLabel("Selecione o Ambiente:")

        self.aeronave_combo = QComboBox()
        self.aeronave_combo.addItems(["EASy", "T30", "T30_barra", "T30_estavel"])
        self.aeronave_label = QLabel("Selecione a Aeronave:")

        # Botões On/Off para Console e Mapa
        self.console_checkbox = QCheckBox("Console")
        self.console_checkbox.setChecked(True)
        self.mapa_checkbox = QCheckBox("Mapa")
        self.mapa_checkbox.setChecked(True)

        # Caixas de texto para parâmetros
        self.j_label = QLabel("Valor de -j:")
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
            ("Script Avionica", self.start_avionics_script, "script_avionica"),
            ("Iniciar Ardupilot", self.start_ardupilot, "ardupilot"),
            ("Carregar Joystick", self.load_joystick, "joystick"),
            ("Enviar Missão", self.send_mission, "enviar_missao"),
            ("Abrir MissionPlanner", self.start_mission_planner, "mission_planner"),
            ("Plot Tempo Real", self.start_real_time_plot, "plot_tempo_real"),
            ("Plot Excel", self.plot_excel, "plot_excel")
        ]

        self.buttons = {}

        for label, func, key in actions:
            btn_layout = QHBoxLayout()
            start_btn = QPushButton(label)
            start_btn.clicked.connect(func)
            start_btn.setEnabled(True)  # Ajustaremos a habilitação posteriormente

            stop_btn = QPushButton("Encerrar")
            stop_btn.setStyleSheet("background-color: red; color: white;")
            stop_btn.clicked.connect(lambda _, l=label: self.stop_process(l))

            self.buttons[key] = start_btn

            btn_layout.addWidget(start_btn)
            btn_layout.addWidget(stop_btn)
            left_layout.addLayout(btn_layout)

        # Botão para encerrar tudo e fechar o programa
        self.exit_btn = QPushButton("Encerrar Tudo e Sair")
        self.exit_btn.setStyleSheet("background-color: darkred; color: white;")
        self.exit_btn.clicked.connect(self.close_application)
        left_layout.addWidget(self.exit_btn)

        # Ajustar habilitação inicial dos botões
        self.buttons["script_avionica"].setEnabled(False)
        self.buttons["joystick"].setEnabled(False)
        self.buttons["enviar_missao"].setEnabled(False)

        # Área de saída (abas com terminais simulados)
        output_layout.addWidget(self.tabs)

        # Configuração do layout principal
        top_layout.addLayout(left_layout)
        top_layout.addLayout(output_layout)

        # Usar QSplitter para permitir redimensionamento
        splitter = QSplitter(Qt.Vertical)
        top_widget = QWidget()
        top_widget.setLayout(top_layout)
        splitter.addWidget(top_widget)
        # Não precisamos adicionar a área de terminais ao splitter aqui

        main_layout.addLayout(title_layout)
        main_layout.addWidget(splitter)

        container = QWidget()
        container.setLayout(main_layout)
        self.setCentralWidget(container)

    def start_gazebo(self):
        label = "Gazebo"
        ambiente = self.ambiente_combo.currentText()
        if ambiente == "EASy":
            cmd = 'roslaunch simulacao EASy.launch 2>&1 | grep -v "DS: no mapping for sensor id"'
        elif ambiente == "T30":
            cmd = 'roslaunch simulacao T30.launch 2>&1 | grep -v "DS: no mapping for sensor id"'
        else:
            cmd = 'roslaunch simulacao simulacao.launch 2>&1 | grep -v "DS: no mapping for sensor id"'

        self.start_process(label, cmd)
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
        self.start_process(label, cmd)
        # Habilitar o botão Carregar Joystick
        self.buttons["joystick"].setEnabled(True)
        self.buttons_state["Ardupilot"] = True

    def load_joystick(self):
        label = "Carregar Joystick"
        cmd = "module load joystick"
        self.start_process(label, cmd)
        # Habilitar o botão Enviar Missão
        self.buttons["enviar_missao"].setEnabled(True)

    def start_mission_planner(self):
        label = "MissionPlanner"
        home_dir = os.path.expanduser("~")
        cwd = os.path.join(home_dir, "Desktop/MissionPlanner")
        cmd = f"cd {cwd} && mono MissionPlanner.exe"
        self.start_process(label, cmd)

    def start_real_time_plot(self):
        label = "Plot Tempo Real"
        home_dir = os.path.expanduser("~")
        cwd = os.path.join(home_dir, "catkin_ws/src/Euler_Drone_Sim/controle/euler_drone_pkgs/src/scripts")
        cmd = f"cd {cwd} && python Plot.py"
        self.start_process(label, cmd)

    def start_avionics_script(self):
        label = "Script Avionica"
        if not self.buttons_state.get("Gazebo", False):
            QMessageBox.warning(self, "Aviso", "Por favor, inicie o Gazebo primeiro.")
            return
        home_dir = os.path.expanduser("~")
        cwd = os.path.join(home_dir, "catkin_ws/src/Euler_Drone_Sim/controle/euler_drone_pkgs/src/scripts")
        cmd = f"cd {cwd} && python MAVLink_sender.py"
        self.start_process(label, cmd)

    def send_mission(self):
        label = "Enviar Missão"
        if not self.buttons_state.get("Ardupilot", False):
            QMessageBox.warning(self, "Aviso", "Por favor, inicie o Ardupilot primeiro.")
            return
        home_dir = os.path.expanduser("~")
        cwd = os.path.join(home_dir, "catkin_ws/src/Euler_Drone_Sim/controle/euler_drone_pkgs/src/scripts")
        cmd = f"cd {cwd} && python Mission_sender.py"
        self.start_process(label, cmd)

    def plot_excel(self):
        label = "Plot Excel"
        home_dir = os.path.expanduser("~")
        cwd = os.path.join(home_dir, "catkin_ws/src/Euler_Drone_Sim/controle/euler_drone_pkgs/src/scripts")
        cmd = f"cd {cwd} && python PlotExcel.py"
        self.start_process(label, cmd)

    def start_process(self, label, cmd):
        if label in self.process_tabs:
            QMessageBox.information(self, "Processo em Execução", f"{label} já está em execução.")
            return
        tab = TerminalTab(label, cmd)
        self.tabs.addTab(tab, label)
        self.process_tabs[label] = tab

    def stop_process(self, label):
        if label in self.process_tabs:
            tab = self.process_tabs[label]
            tab.stop_process()
            index = self.tabs.indexOf(tab)
            self.tabs.removeTab(index)
            del self.process_tabs[label]
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

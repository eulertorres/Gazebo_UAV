// euler_mot_plugin_v2.cpp

#include "euler_plugins/euler_mot_plugin_v2.h"

namespace gazebo {

GZ_REGISTER_MODEL_PLUGIN(EulerMotPluginV2)

// Implementação de EulerMotSocketPrivate

EulerMotSocketPrivate::EulerMotSocketPrivate() {
    fd = socket(AF_INET, SOCK_DGRAM, 0);
    #ifndef _WIN32
    fcntl(fd, F_SETFD, FD_CLOEXEC);
    #endif
}

EulerMotSocketPrivate::~EulerMotSocketPrivate() {
    if (fd != -1) {
        ::close(fd);
        fd = -1;
    }
}

bool EulerMotSocketPrivate::Bind(const char *_address, const uint16_t _port) {
    struct sockaddr_in sockaddr;
    MakeSockAddr(_address, _port, sockaddr);

    if (bind(fd, (struct sockaddr *)&sockaddr, sizeof(sockaddr)) != 0) {
        ::shutdown(fd, 0);  // Use ::shutdown para chamar a função do sistema
        ::close(fd);
        return false;
    }
    int one = 1;
    setsockopt(fd, SOL_SOCKET, SO_REUSEADDR, reinterpret_cast<const char *>(&one), sizeof(one));
    fcntl(fd, F_SETFL, fcntl(fd, F_GETFL, 0) | O_NONBLOCK);
    return true;
}

void EulerMotSocketPrivate::MakeSockAddr(const char *_address, const uint16_t _port, struct sockaddr_in &_sockaddr) {
    memset(&_sockaddr, 0, sizeof(_sockaddr));
    _sockaddr.sin_port = htons(_port);
    _sockaddr.sin_family = AF_INET;
    _sockaddr.sin_addr.s_addr = inet_addr(_address);
}

ssize_t EulerMotSocketPrivate::Recv(void *_buf, const size_t _size, uint32_t _timeoutMs) {
    fd_set fds;
    struct timeval tv;

    FD_ZERO(&fds);
    FD_SET(fd, &fds);

    tv.tv_sec = _timeoutMs / 1000;
    tv.tv_usec = (_timeoutMs % 1000) * 1000UL;

    if (select(fd + 1, &fds, NULL, NULL, &tv) != 1) {
        return -1;
    }

    return recv(fd, _buf, _size, 0);
}

// Construtor de MotorControl
MotorControl::MotorControl() :
    channel(0),
    cmd(0.0),
    multiplier(1.0),
    offset(0.0),
    pwm_min(1000),
    pwm_max(2000),
    KV_rpm(77),
    KV_rad(0.0),
    R(0.116),
    L(0.004),
    i0(0.250),
    n_esc(1.0),
    b(0.000198845),
    orientation("CCW"),
    wire_gauge_awg(10),
    Kp(0.85),
    Ki(0.085),
    Kd(0.0),
    Kt(0.0),
    Kbq(0.0),
    max_current(0.0),
    battery_voltage(0.0),
    current(0.0),
    torque(0.0),
    angular_velocity(0.0),
    rpm(0.0),
    rpm_des(0.0),
    is_delta(true),
    integral_error(0.0),
    prev_error(0.0),
    max_angular_velocity(0.0),
    debug(false)
{}

// Implementação de EulerMotPluginV2

EulerMotPluginV2::EulerMotPluginV2() : ModelPlugin(), nh_(nullptr) {}

EulerMotPluginV2::~EulerMotPluginV2() {
    if (nh_) {
        nh_->shutdown();
        delete nh_;
    }
}

void EulerMotPluginV2::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf) {
    // Armazena ponteiros para o modelo e o mundo
    model_ = _model;
    world_ = model_->GetWorld();

    // Inicializa o ROS se ainda não foi inicializado (se necessário)
    if (!ros::isInitialized()) {
        int argc = 0;
        char** argv = nullptr;
        ros::init(argc, argv, "gazebo_motor_plugin_v2", ros::init_options::NoSigintHandler);
    }

    nh_ = new ros::NodeHandle();

    // Inicializa os sockets
    if (!InitSockets(_sdf)) {
        gzerr << "[EulerMotPluginV2] Falha ao inicializar sockets.\n";
        return;
    }

    // Analisa o SDF para obter configurações dos motores
    ParseMotorSDF(_sdf);

    // Inicializa os publicadores ROS
    motor_state_pub_ = nh_->advertise<std_msgs::Float32MultiArray>("motor_state", 10);
    motor_cmd_pub_ = nh_->advertise<std_msgs::Float32MultiArray>("motor_cmd", 10);
    joint_state_pub_ = nh_->advertise<sensor_msgs::JointState>("joint_states", 10);

    // Conecta-se ao evento de atualização do Gazebo
    update_connection_ = event::Events::ConnectWorldUpdateBegin(
        std::bind(&EulerMotPluginV2::OnUpdate, this, std::placeholders::_1));

    last_update_time_ = world_->SimTime();
    last_publish_time_ = world_->SimTime();

    std::cout << "[EulerMotPluginV2] Plugin carregado com sucesso.\n";
}

bool EulerMotPluginV2::InitSockets(sdf::ElementPtr _sdf) {
    fdm_addr_ = _sdf->Get<std::string>("fdm_addr", "127.0.0.1").first;
    listen_addr_ = _sdf->Get<std::string>("listen_addr", "127.0.0.1").first;
	fdm_port_in_ = _sdf->Get<unsigned int>("fdm_port_in", 9002).first;
	fdm_port_out_ = _sdf->Get<unsigned int>("fdm_port_out", 9003).first;

    if (!socket_in_.Bind(listen_addr_.c_str(), fdm_port_in_)) {
        gzerr << "[EulerMotPluginV2] Falha ao vincular socket em " << listen_addr_ << ":" << fdm_port_in_ << "\n";
        return false;
    } else {
        gzdbg << "[EulerMotPluginV2] Socket vinculado com sucesso em " << listen_addr_ << ":" << fdm_port_in_ << "\n";
    }

    return true;
}

void EulerMotPluginV2::ParseMotorSDF(sdf::ElementPtr _sdf) {
    if (!_sdf->HasElement("motor")) {
        gzerr << "[EulerMotPluginV2] Nenhum elemento <motor> encontrado no SDF.\n";
        return;
    }

    sdf::ElementPtr motorSDF = _sdf->GetElement("motor");
    while (motorSDF) {
        MotorControl motor;

        if (motorSDF->HasAttribute("channel")) {
            motor.channel = motorSDF->Get<int>("channel");
        } else {
            motor.channel = motors_.size();
            gzwarn << "[EulerMotPluginV2] Canal não especificado, usando índice " << motor.channel << "\n";
        }

        if (motorSDF->HasElement("joint_name")) {
            motor.joint_name = motorSDF->Get<std::string>("joint_name");
        } else {
            gzerr << "[EulerMotPluginV2] joint_name não especificado para o motor no canal " << motor.channel << "\n";
            motorSDF = motorSDF->GetNextElement("motor");
            continue;
        }

        // Obtém o ponteiro para a articulação
        motor.joint = model_->GetJoint(motor.joint_name);
        if (!motor.joint) {
            gzerr << "[EulerMotPluginV2] Não foi possível encontrar a articulação: " << motor.joint_name << "\n";
            motorSDF = motorSDF->GetNextElement("motor");
            continue;
        }

        // Carrega outros parâmetros específicos do motor
        motor.pwm_min = motorSDF->Get<double>("PWM_min", 1000).first;
        motor.pwm_max = motorSDF->Get<double>("PWM_max", 2000).first;
        motor.KV_rpm = motorSDF->Get<double>("KV_rpm", 77).first;
        motor.R = motorSDF->Get<double>("R", 0.116).first;
        motor.L = motorSDF->Get<double>("L", 0.004).first;
        motor.i0 = motorSDF->Get<double>("i0", 0.250).first;
        motor.n_esc = motorSDF->Get<double>("n_esc", 1.0).first;
        motor.b = motorSDF->Get<double>("b", 0.000198845).first;
        motor.orientation = motorSDF->Get<std::string>("orientacao", "CCW").first;
        motor.wire_gauge_awg = motorSDF->Get<int>("AWG", 10).first;
        motor.Kp = motorSDF->Get<double>("Kp", 0.85).first;
        motor.Ki = motorSDF->Get<double>("Ki", 0.085).first;
        motor.Kd = motorSDF->Get<double>("Kd", 0.0).first;
        motor.is_delta = motorSDF->Get<bool>("delta", true).first;
        motor.debug = motorSDF->Get<bool>("debug", false).first;

        // Calcula constantes
        motor.max_current = GetMaxCurrentForAWG(motor.wire_gauge_awg);
        motor.KV_rpm = motor.KV_rpm * motor.n_esc;
        motor.KV_rad = motor.KV_rpm * M_PI / 30.0;  // Converte rpm/V para rad/s/V

        if (motor.is_delta) {
            motor.Kt = (1.0 / motor.KV_rad) * sqrt(3.0 / 2.0);
            motor.L = motor.L / 2.0;
            motor.R = motor.R / 2.0;
        } else {
            motor.Kt = 1.0 / (motor.KV_rad * sqrt(2.0));
            motor.L = motor.L * 3.0 / 2.0;
            motor.R = motor.R * 3.0 / 2.0;
        }

        motor.Kbq = motor.Kt;

        motors_.push_back(motor);

        motorSDF = motorSDF->GetNextElement("motor");
    }
}

void EulerMotPluginV2::OnUpdate(const common::UpdateInfo& _info) {
    common::Time current_time = world_->SimTime();
    double dt = (current_time - last_update_time_).Double();

    if (dt <= 0.0)
        return; // Previne divisão por zero ou passos de tempo negativos

    ReceiveMotorCommand();
    ApplyMotorForces(dt);

    last_update_time_ = current_time;
}

void EulerMotPluginV2::ReceiveMotorCommand() {
    ServoPacket pkt;
    ssize_t recvSize = socket_in_.Recv(&pkt, sizeof(ServoPacket), 1);

    if (recvSize == -1) {
        // Não recebeu um pacote
        // Você pode adicionar tratamento de erro aqui se desejar
        return;
    }

    ssize_t recvChannels = recvSize / sizeof(pkt.motorSpeed[0]);

    // Atualiza os comandos de cada motor com base nos dados recebidos
    for (auto& motor : motors_) {
        if (motor.channel < recvChannels) {
            // Atualiza o comando do motor
            motor.cmd = pkt.motorSpeed[motor.channel];

            // Se necessário, você pode aplicar multiplicador e offset
            motor.cmd = motor.multiplier * (motor.offset + motor.cmd);
        } else {
            gzerr << "[EulerMotPluginV2] Canal " << motor.channel << " não recebido.\n";
        }
    }
}

void EulerMotPluginV2::ApplyMotorForces(double dt) {

	std_msgs::Float32MultiArray motor_state_msg;
    std_msgs::Float32MultiArray motor_cmd_msg;
    sensor_msgs::JointState joint_state_msg;

    for (auto& motor : motors_) {
        // Atualiza as variáveis específicas do motor
        double pwm_value = motor.cmd;
        double duty = (pwm_value - motor.pwm_min) / (motor.pwm_max - motor.pwm_min);

        // Atualiza RPM para publicação
        motor.rpm = motor.angular_velocity * 60.0 / (2 * M_PI);

        if (duty < 0.01 || motor.battery_voltage <= 2.0) {
            motor.torque = 0.0;
            motor.current = motor.i0;
            motor.rpm_des = 0;
            continue;
        }

        // Definir a velocidade angular desejada
        if (motor.is_delta) {
            motor.max_angular_velocity = sqrt(3.0 / 2.0) * motor.battery_voltage / motor.Kbq;
        } else {
            motor.max_angular_velocity = sqrt(1.0 / 2.0) * motor.battery_voltage / motor.Kbq;
        }

        double desired_angular_velocity = duty * motor.max_angular_velocity * 0.9;

        // Obter velocidade angular atual
        if (motor.joint) {
            motor.angular_velocity = std::abs(motor.joint->GetVelocity(0));  // Pega a magnitude da velocidade angular
        } else {
            motor.angular_velocity = 0.0;
        }

        double error = desired_angular_velocity - motor.angular_velocity;

        // Controlador PID
        motor.integral_error += error * dt;
        double derivative = (error - motor.prev_error) / dt;
        double Vq = motor.Kp * error + motor.Ki * motor.integral_error + motor.Kd * derivative;
        motor.prev_error = error;

        // Limitar Vq
        if (Vq > motor.battery_voltage) {
            Vq = motor.battery_voltage;
        } else if (Vq < -motor.battery_voltage) {
            Vq = -motor.battery_voltage;
        }

        // Calcular dIq_dt no eixo q
        double dIq_dt = (Vq - motor.R * motor.current - motor.Kbq * motor.angular_velocity) / motor.L;

        // Atualizar corrente
        motor.current += dIq_dt * dt;

        // Limitar corrente
        if (motor.current > motor.max_current) {
            motor.current = motor.max_current;
        } else if (motor.current < -motor.max_current) {
            motor.current = -motor.max_current;
        }

        // Calcular torque
        motor.torque = motor.Kt * motor.current - motor.b * motor.angular_velocity;

        // Limitar torque se necessário
        double max_torque = motor.Kt * motor.max_current;
        if (motor.torque > max_torque) {
            motor.torque = max_torque;
        } else if (motor.torque < -max_torque) {
            motor.torque = -max_torque;
        }

        // Aplicar torque na junta
        if (motor.orientation == "CW") {
            motor.joint->SetForce(0, -motor.torque);
        } else {
            motor.joint->SetForce(0, motor.torque);
        }

        // Atualiza as mensagens
        motor_state_msg.data.push_back(motor.angular_velocity);
        motor_cmd_msg.data.push_back(motor.cmd);

        // Preencher a mensagem de joint_state
        joint_state_msg.name.push_back(motor.joint_name);
        joint_state_msg.position.push_back(motor.joint->Position(0));
        joint_state_msg.velocity.push_back(motor.joint->GetVelocity(0));
        joint_state_msg.effort.push_back(motor.torque);

    }

	// Publica as mensagens
	motor_state_pub_.publish(motor_state_msg);
	motor_cmd_pub_.publish(motor_cmd_msg);
	joint_state_pub_.publish(joint_state_msg);

}

double EulerMotPluginV2::GetMaxCurrentForAWG(int awg) {
    // Tabela aproximada de corrente máxima para cada AWG
    std::map<int, double> awg_max_current = {
        {10, 55.0},
        {12, 41.0},
        {14, 32.0},
        {16, 22.0},
        {18, 16.0},
        {20, 11.0}
        // Adicione mais tamanhos conforme necessário
    };

    if (awg_max_current.find(awg) != awg_max_current.end()) {
        return awg_max_current[awg];
    } else {
        // Valor conservador padrão
        return 41.0;
    }
}

}  // namespace gazebo

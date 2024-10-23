// euler_mot_plugin.cpp

#include "euler_plugins/euler_mot_plugin.h"

namespace gazebo {

GZ_REGISTER_MODEL_PLUGIN(EulerMotPlugin)

// Construtor e destrutor
EulerMotPlugin::EulerMotPlugin()
    : ModelPlugin(),
      nh_(nullptr),
      pwm_value_(0),
      battery_voltage_(0.0),
      current_(0.0),
      torque_(0.0),
      rpm_(0.0),
      angular_velocity_(0.0),
      is_delta_(true),
      debug_(false),  // Inicializa a flag de debug como false
      integral_error_(0.0),  // Inicializa o termo integral do PID
      prev_error_(0.0)       // Inicializa o erro anterior para o termo derivativo
{}

EulerMotPlugin::~EulerMotPlugin() {
    if (nh_) {
        nh_->shutdown();
        delete nh_;
    }
}

void EulerMotPlugin::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf) {
    // Armazena ponteiros para o modelo e o mundo
    model_ = _model;
    world_ = model_->GetWorld();

    // Inicializa o ROS se ainda não foi inicializado
    if (!ros::isInitialized()) {
        int argc = 0;
        char** argv = nullptr;
        ros::init(argc, argv, "gazebo_motor_plugin", ros::init_options::NoSigintHandler);
    }

    // NodeHandle do ROS
    nh_ = new ros::NodeHandle();

    // Lê parâmetros do SDF, incluindo se o motor é Delta ou Estrela
    if (_sdf->HasElement("delta"))
        is_delta_ = _sdf->Get<bool>("delta"); // Enrolamento: Delta ou Estrela

    if (_sdf->HasElement("numero_motor"))
        motor_number_ = _sdf->Get<int>("numero_motor");
    else
        motor_number_ = 1;
    std::string motor_str = std::to_string(motor_number_);

    if (_sdf->HasElement("PWM_min"))
        pwm_min_ = _sdf->Get<double>("PWM_min");
    else
        pwm_min_ = 1000;

    if (_sdf->HasElement("PWM_max"))
        pwm_max_ = _sdf->Get<double>("PWM_max");
    else
        pwm_max_ = 2000;

    // Lê as constantes do motor do SDF
    if (_sdf->HasElement("KV_rpm"))
        KV_rpm_ = _sdf->Get<double>("KV_rpm");
    else
        KV_rpm_ = 77;  // Valor padrão alterado para 77

    // Calcula KV_rad_
    KV_rad_ = KV_rpm_ * M_PI / 30.0;  // Converte rpm/V para rad/s/V

    // Kt é o recíproco de KV_rad
    Kt_ = 1.0 / KV_rad_;
    Kbq_ = Kt_;             // Constante de força contraeletromotriz (V/(rad/s))

    if (_sdf->HasElement("R"))
        R_ = _sdf->Get<double>("R");
    else
        R_ = 0.116;

    if (_sdf->HasElement("L"))
        L_ = _sdf->Get<double>("L");
    else
        L_ = 0.004;

    L_ = L_ / 2;   // Configuração Delta

    if (_sdf->HasElement("i0"))
        i0_ = _sdf->Get<double>("i0");
    else
        i0_ = 0.250;

    if (_sdf->HasElement("n_esc"))
        n_esc_ = _sdf->Get<double>("n_esc");
    else
        n_esc_ = 1.0;

    if (_sdf->HasElement("b"))
        b_ = _sdf->Get<double>("b");
    else
        b_ = 0.000198845;  // Valor padrão positivo

    if (_sdf->HasElement("joint_name"))
        joint_name_ = _sdf->Get<std::string>("joint_name");
    else
        joint_name_ = "motor_joint";

    if (_sdf->HasElement("current_topic"))
        current_topic_ = _sdf->Get<std::string>("current_topic");
    else
        current_topic_ = "/battery" + motor_str;  // Nome personalizado do tópico

    if (_sdf->HasElement("torque_topic"))
        torque_topic_ = _sdf->Get<std::string>("torque_topic");
    else
        torque_topic_ = "/motor" + motor_str + "/torque";  // Nome personalizado do tópico

    if (_sdf->HasElement("rpm_topic"))
        rpm_topic_ = _sdf->Get<std::string>("rpm_topic");
    else
        rpm_topic_ = "/motor" + motor_str + "/rpm";  // Nome personalizado do tópico

    if (_sdf->HasElement("orientacao"))
        orientation_ = _sdf->Get<std::string>("orientacao");
    else
        orientation_ = "CCW";  // Padrão é CCW

    if (_sdf->HasElement("AWG"))
        wire_gauge_awg_ = _sdf->Get<int>("AWG");
    else
        wire_gauge_awg_ = 10;  // Valor padrão

    // Lê o parâmetro de debug do SDF
    if (_sdf->HasElement("debug"))
        debug_ = _sdf->Get<bool>("debug");
    else
        debug_ = false;
    pwm_timeout_ = 1.0;

    if (_sdf->HasElement("Kp"))
        Kp = _sdf->Get<double>("Kp");
    else
        Kp = 0.85;  // Valor padrão

    if (_sdf->HasElement("Ki"))
        Ki = _sdf->Get<double>("Ki");
    else
        Ki = 0.085;  // Valor padrão

    if (_sdf->HasElement("Kd"))
        Kd = _sdf->Get<double>("Kd");
    else
        Kd = 0.0046;  // Valor padrão

    // Obtém o ponteiro para a articulação
    joint_ = model_->GetJoint(joint_name_);
    if (!joint_) {
        gzerr << "[euler_mot_plugin] Não foi possível encontrar a articulação: " << joint_name_ << "\n";
        return;
    }

    // Publicadores para corrente e torque
    current_pub_ = nh_->advertise<std_msgs::Float32>(current_topic_, 1);
    torque_pub_ = nh_->advertise<std_msgs::Float32>(torque_topic_, 1);
    rpm_pub_ = nh_->advertise<std_msgs::Float32>(rpm_topic_, 1);  // Publicador para RPM

    // Inscreve-se nos tópicos de PWM e tensão da bateria
    pwm_sub_ = nh_->subscribe("/mavros/rc/out", 1, &EulerMotPlugin::PwmCallback, this);
    voltage_sub_ = nh_->subscribe("/battery/voltage", 1, &EulerMotPlugin::VoltageCallback, this);

    // Conecta-se ao evento de atualização do Gazebo
    update_connection_ = event::Events::ConnectWorldUpdateBegin(
        std::bind(&EulerMotPlugin::OnUpdate, this, std::placeholders::_1));

    // Calcular a corrente máxima com base no AWG
    max_current_ = GetMaxCurrentForAWG(wire_gauge_awg_);

    std::cout << "[euler_mot_plugin] Plugin do motor carregado com sucesso.\n";
    if (debug_) {
        std::cout << "[euler_mot_plugin] Modo debug ativado para o motor " << motor_number_ << ".\n";
    }
    std::cout.flush();
    last_pwm_received_time_ = world_->SimTime();  // Inicializa o tempo da última recepção de PWM
    last_publish_time_ = world_->SimTime();       // Inicializa o tempo da última publicação
    last_update_time_ = world_->SimTime();

	current_ = i0_;
}

void EulerMotPlugin::OnUpdate(const common::UpdateInfo& _info) {
    common::Time current_time = world_->SimTime();
    double dt = (current_time - last_update_time_).Double();

    if (dt <= 0.0)
        return; // Previne divisão por zero ou passos de tempo negativos

    calcula(dt);
    last_update_time_ = current_time;

    // Publica em uma taxa menor (por exemplo, a cada 0.1s)
    if ((current_time - last_publish_time_) >= 0.1) {
        PublishCurrent();
        PublishTorque();
        PublishRPM();
        last_publish_time_ = current_time;
    }
}

// Função para calcular corrente e torque
void EulerMotPlugin::calcula(double dt) {
    double duty = (pwm_value_ - pwm_min_) / (pwm_max_ - pwm_min_);

	// **Atualizar RPM para publicação**
    rpm_ = angular_velocity_ * 60.0 / (2 * M_PI);

    if (duty < 0.01 || battery_voltage_ <= 2.0) {
        torque_ = 0.0;
		current_ = i0_;
        return;
    }

    // **1. Definir a velocidade angular desejada**
    double max_angular_velocity = (sqrt(2.0 / 3.0) * battery_voltage_) / Kbq_;
    double desired_angular_velocity = duty * max_angular_velocity;

    // **2. Calcular o erro de velocidade**
    // Obter velocidade angular atual
    if (joint_) {
        angular_velocity_ = std::abs(joint_->GetVelocity(0));  // Pega a magnitude da velocidade angular
    } else {
        angular_velocity_ = 0.0;
    }

    double error = desired_angular_velocity - angular_velocity_;

    // **3. Implementar o controlador PID**
    // Ganhos do PID (ajuste conforme necessário)

    // Termo integral
    integral_error_ += error * dt;

    // Termo derivativo
    double derivative = (error - prev_error_) / dt;

    // Saída do controlador PID (V_q)
    double Vq = Kp * error + Ki * integral_error_ + Kd * derivative;

    // Atualizar o erro anterior
    prev_error_ = error;

    // **4. Aplicar limites a V_q**
    // V_q não pode exceder a tensão da bateria
    if (Vq > battery_voltage_) {
        Vq = battery_voltage_;
    } else if (Vq < -battery_voltage_) {
        Vq = -battery_voltage_;
    }

    // **5. Calcular dIq_dt no eixo q**
    double dIq_dt = (Vq - R_ * current_ - Kbq_ * angular_velocity_) / L_;

    // **6. Atualizar a corrente usando integração de Euler**
    current_ += dIq_dt * dt;

    // **7. Limitar a corrente para evitar valores não físicos**
    if (current_ > max_current_) {
        current_ = max_current_;
    } else if (current_ < -max_current_) {
        current_ = -max_current_;
    }

    // **8. Calcular o torque usando a equação do motor**
    torque_ = Kt_ * current_ - b_ * angular_velocity_;

    // **9. Limitar o torque se necessário**
    double max_torque = Kt_ * max_current_;
    if (torque_ > max_torque) {
        torque_ = max_torque;
    } else if (torque_ < -max_torque) {
        torque_ = -max_torque;
    }

    // **10. Aplicar o torque na junta**
    if (orientation_ == "CW") {
        joint_->SetForce(0, -torque_);
    } else {
        joint_->SetForce(0, torque_);
    }

    if (debug_) {
        PrintDebug(duty, Vq, dIq_dt, desired_angular_velocity, error);
    }
}

void EulerMotPlugin::PrintDebug(double duty, double Vq, double dIq_dt, double desired_angular_velocity, double error){
    std::cout << "[Motor " << motor_number_ << "] PWM Value: " << pwm_value_ << ", Duty Cycle: " << duty << std::endl;
    std::cout << "[Motor " << motor_number_ << "] Battery Voltage: " << battery_voltage_ << ", Vq: " << Vq << std::endl;
    std::cout << "[Motor " << motor_number_ << "] Desired Angular Velocity (rad/s): " << desired_angular_velocity << std::endl;
    std::cout << "[Motor " << motor_number_ << "] Actual Angular Velocity (rad/s): " << angular_velocity_ << std::endl;
    std::cout << "[Motor " << motor_number_ << "] Error: " << error << std::endl;
    std::cout << "[Motor " << motor_number_ << "] dIq_dt: " << dIq_dt << std::endl;
    std::cout << "[Motor " << motor_number_ << "] Current before limiting: " << current_ << std::endl;
    std::cout << "[Motor " << motor_number_ << "] Torque before limiting: " << torque_ << std::endl;
    if (orientation_ == "CW"){
        std::cout << "[Motor " << motor_number_ << "] Applying torque (CW): " << -torque_ << std::endl;
    } else{
        std::cout << "[Motor " << motor_number_ << "] Applying torque (CCW): " << torque_ << std::endl;
    }
}

// Publica a corrente calculada
void EulerMotPlugin::PublishCurrent() {
    std_msgs::Float32 current_msg;
    current_msg.data = current_;
    current_pub_.publish(current_msg);
}

// Publica o torque calculado
void EulerMotPlugin::PublishTorque() {
    std_msgs::Float32 torque_msg;
    torque_msg.data = torque_;
    torque_pub_.publish(torque_msg);
}

// Publica o RPM calculado
void EulerMotPlugin::PublishRPM() {
    std_msgs::Float32 rpm_msg;
    rpm_msg.data = rpm_;
    rpm_pub_.publish(rpm_msg);
}

void EulerMotPlugin::PwmCallback(const mavros_msgs::RCOut::ConstPtr& msg) {
    if (motor_number_ > 0 && motor_number_ <= msg->channels.size()) {
        pwm_value_ = msg->channels[motor_number_ - 1];
        last_pwm_received_time_ = world_->SimTime();  // Atualiza o tempo da última recepção
    }
}

void EulerMotPlugin::VoltageCallback(const std_msgs::Float32::ConstPtr& msg) {
    battery_voltage_ = msg->data;  // Atualiza Vbat com a tensão recebida
    //if (debug_) {
    //    std::cout << "[Motor " << motor_number_ << "] Tensão da bateria atualizada: " << battery_voltage_ << std::endl;
    //}
}

double EulerMotPlugin::GetMaxCurrentForAWG(int awg) {
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

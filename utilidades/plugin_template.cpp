// euler_mot_plugin.cpp

#include "euler_plugins/euler_mot_plugin.h"

namespace gazebo {

GZ_REGISTER_MODEL_PLUGIN(EulerMotPlugin)

// Construtor e destrutor
EulerMotPlugin::EulerMotPlugin()
    : ModelPlugin(),
      nh_(nullptr),
      pwm_value_(0) {}

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

    // Lê os parâmetros do SDF
    if (_sdf->HasElement("numero_motor"))
        motor_number_ = _sdf->Get<int>("numero_motor");
    else
        motor_number_ = 1;
    std::string motor_str = std::to_string(motor_number_);

    if (_sdf->HasElement("PWM_min"))
        pwm_min_ = _sdf->Get<double>("PWM_min");
    else
        pwm_min_ = 1100;

    if (_sdf->HasElement("PWM_max"))
        pwm_max_ = _sdf->Get<double>("PWM_max");
    else
        pwm_max_ = 1900;

    // Lê as constantes do motor do SDF
    if (_sdf->HasElement("KV_rpm"))
        KV_rpm_ = _sdf->Get<double>("KV_rpm");
    else
        KV_rpm_ = 77;  // Valor padrão alterado para 77

    // Calcula KV_rad_
    KV_rad_ = KV_rpm_ * M_PI / 30.0;  // Converte rpm/V para rad/s/V

    // Kt é o recíproco de KV_rad
    Kt_ = 1.0 / KV_rad_;

    if (_sdf->HasElement("R"))
        R_ = _sdf->Get<double>("R");
    else
        R_ = 0.116;

    if (_sdf->HasElement("L"))
        L_ = _sdf->Get<double>("L");
    else
        L_ = 0.004;

    if (_sdf->HasElement("i0"))
        i0_ = _sdf->Get<double>("i0");
    else
        i0_ = 1.0;

    if (_sdf->HasElement("n_esc"))
        n_esc_ = _sdf->Get<double>("n_esc");
    else
        n_esc_ = 1.0;

    // Lê as constantes da hélice do SDF
    if (_sdf->HasElement("Iz"))
        Iz_ = _sdf->Get<double>("Iz");
    else
        Iz_ = 2.0 * 5.23e-3; // Valor padrão

    if (_sdf->HasElement("a"))
        a_ = _sdf->Get<double>("a");
    else
        a_ = 0.00000153635;

    if (_sdf->HasElement("b"))
        b_ = _sdf->Get<double>("b");
    else
        b_ = -0.000198845;

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

    if (_sdf->HasElement("forceUpdateRate"))
        force_update_rate_ = _sdf->Get<double>("forceUpdateRate");
    else
        force_update_rate_ = 0.1;

    if (_sdf->HasElement("orientacao"))
        orientation_ = _sdf->Get<std::string>("orientacao");
    else
        orientation_ = "CCW";  // Padrão é CCW

    pwm_timeout_ = 1.0;

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

    pwm_value_ = 1100;
    // Inscreve-se nos tópicos de PWM e tensão da bateria
    pwm_sub_ = nh_->subscribe("/mavros/rc/out", 1, &EulerMotPlugin::PwmCallback, this);
    voltage_sub_ = nh_->subscribe("/battery/voltage", 1, &EulerMotPlugin::VoltageCallback, this);

    // Conecta-se ao evento de atualização do Gazebo
    update_connection_ = event::Events::ConnectWorldUpdateBegin(
        std::bind(&EulerMotPlugin::OnUpdate, this, std::placeholders::_1));

    std::cout << "[euler_mot_plugin] Plugin do motor carregado com sucesso.\n";
    std::cout.flush();
    last_pwm_received_time_ = world_->SimTime();  // Inicializa o tempo da última recepção de PWM
    last_update_time_ = world_->SimTime();
    last_integration_time_ = world_->SimTime();  // Inicializa o tempo da última integração
}

void EulerMotPlugin::OnUpdate(const common::UpdateInfo& _info) {
    common::Time current_time = world_->SimTime();

	// Calculo torque e corrente

    // Publica corrente e torque na taxa desejada
    if ((current_time - last_update_time_).Double() >= 0.1) {
        PublishCurrent();
        PublishTorque();
        PublishRPM();
        last_update_time_ = current_time;
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
    rpm_msg.data = rpm;
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
}

}  // namespace gazebo

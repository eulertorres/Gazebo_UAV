// euler_batt_plugin.cpp

#include "euler_plugins/euler_batt_plugin.h"

namespace gazebo {

GZ_REGISTER_MODEL_PLUGIN(EulerBattPlugin)

EulerBattPlugin::EulerBattPlugin() : ModelPlugin(), nh_(nullptr), current_draw_(0.0) {}

EulerBattPlugin::~EulerBattPlugin() {
    if (nh_) {
        nh_->shutdown();
        delete nh_;
    }
}

void EulerBattPlugin::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf) {
    // Armazena o ponteiro para o modelo
    model_ = _model;
    world_ = model_->GetWorld();

    // Inicializa o ROS se ainda não foi inicializado
    if (!ros::isInitialized()) {
        int argc = 0;
        char** argv = nullptr;
        ros::init(argc, argv, "gazebo_battery_plugin",
                  ros::init_options::NoSigintHandler);
    }

    // NodeHandle do ROS
    nh_ = new ros::NodeHandle();

    // Leitura dos parâmetros do SDF
    if (_sdf->HasElement("batteryType"))
        battery_type_ = _sdf->Get<std::string>("batteryType");
    else
        battery_type_ = "Default";

    if (_sdf->HasElement("S"))
        cells_in_series_ = _sdf->Get<int>("S");
    else
        cells_in_series_ = 14;

    if (_sdf->HasElement("P"))
        cells_in_parallel_ = _sdf->Get<int>("P");
    else
        cells_in_parallel_ = 4;

    if (_sdf->HasElement("nomeLink"))
        link_name_ = _sdf->Get<std::string>("nomeLink");
    else
        link_name_ = "battery_link";

    if (_sdf->HasElement("TopicoCorrente"))
        current_topic_ = _sdf->Get<std::string>("TopicoCorrente");
    else
        current_topic_ = "/battery/current";

    if (_sdf->HasElement("TopicoTensao"))
        voltage_pub_topic_ = _sdf->Get<std::string>("TopicoTensao");
    else
        voltage_pub_topic_ = "/battery/voltage";

    if (_sdf->HasElement("EstadoBattPubTopic"))
        battery_state_pub_topic_ = _sdf->Get<std::string>("EstadoBattPubTopic");
    else
        battery_state_pub_topic_ = "/battery/state";

    // Obter link da bateria
    link_ = model_->GetLink(link_name_);
    if (!link_) {
        gzerr << "[euler_batt_plugin] Não foi possível encontrar o link especificado: "
              << link_name_ << "\n";
        return;
    }

    // Configurar parâmetros da bateria com base no tipo
    ConfigureBatteryParameters();

    // Calcula a massa da bateria e atribui ao link
    CalculateAndSetBatteryMass();

    // Inscrever-se no tópico de corrente
    current_sub_ = nh_->subscribe(current_topic_, 1, &EulerBattPlugin::CurrentCallback, this);

    // Publicadores
    voltage_pub_ = nh_->advertise<std_msgs::Float32>(voltage_pub_topic_, 1);
    battery_state_pub_ = nh_->advertise<sensor_msgs::BatteryState>(battery_state_pub_topic_, 1);

    // Conectar ao evento de atualização do mundo do Gazebo
    update_connection_ = event::Events::ConnectWorldUpdateBegin(
        std::bind(&EulerBattPlugin::OnUpdate, this, std::placeholders::_1));

    last_update_time_ = world_->SimTime();

    ROS_INFO("[euler_batt_plugin] Plugin carregado com sucesso. Seguuuuura peão");
}

void EulerBattPlugin::OnUpdate(const common::UpdateInfo& _info) {
    common::Time current_time = world_->SimTime();
    double dt = (current_time - last_update_time_).Double();

    // Atualiza o estado de carga (SoC)
    UpdateBatteryState(dt);

    // Publica a tensão disponível
    PublishVoltage();

    // Publica o estado da bateria
    PublishBatteryState();

    last_update_time_ = current_time;
}

void EulerBattPlugin::ConfigureBatteryParameters() {
    // Exemplo de banco de dados de tipos de bateria
    battery_database_ = {
        {"Welion", {4.2, 35, 0.385}},   // {tensao nominal, capacidade (Ah), massa por célula (kg)}
        {"LiPo", {4.2, 2.2, 0.045}},
        {"LiIon", {4.2, 3.0, 0.048}}
    };

    // Verifica se o tipo de bateria está no banco de dados
    if (battery_database_.find(battery_type_) != battery_database_.end()) {
        BatteryInfo info = battery_database_[battery_type_];
        cell_voltage_ = info.nominal_voltage;
        cell_capacity_ = info.capacity;
        cell_mass_ = info.mass;
    } else {
        // Valores padrão
        gzerr << "[euler_batt_plugin] Tipo de bateria não encontrado. Usando LiPo.\n";
        cell_voltage_ = 4.2;
        cell_capacity_ = 2.0;
        cell_mass_ = 0.045;
    }

    // Calcula a tensão total e capacidade total
    total_voltage_ = cell_voltage_ * cells_in_series_;
    total_capacity_ = cell_capacity_ * cells_in_parallel_;  // Em Ah [se lascar Marcus]

    // Estado inicial de carga (100%)
    state_of_charge_ = 1.0;  // 100%
    available_voltage_ = total_voltage_;
}

void EulerBattPlugin::CalculateAndSetBatteryMass() {
    double total_mass = cell_mass_ * cells_in_series_ * cells_in_parallel_;

    // Obtém o ponteiro para o inercial atual do link
    physics::InertialPtr inertial = link_->GetInertial();

    // Define a massa diretamente
    inertial->SetMass(total_mass);

    // Não é necessário definir novamente o inercial no link
    // link_->SetInertial(inertial); // Pode ser omitido

    gzdbg << "[euler_batt_plugin] Massa da bateria definida para " << total_mass << " kg.\n";
}


void EulerBattPlugin::CurrentCallback(const std_msgs::Float32::ConstPtr& msg) {
    current_draw_ = msg->data;  // Corrente em Amperes
}

void EulerBattPlugin::UpdateBatteryState(double dt) {
    // Atualiza o estado de carga (SoC)
    double discharge = (current_draw_ * dt) / (3600.0 * total_capacity_);  // Normalizado pra segundos
    state_of_charge_ -= discharge;
    if (state_of_charge_ < 0.0) state_of_charge_ = 0.0;

    // Atualiza a tensão disponível (simplificação linear)
    available_voltage_ = total_voltage_ * state_of_charge_;
    double min_cell_voltage = 3.2;  // Tensão mínima por célula
    double min_total_voltage = min_cell_voltage * cells_in_series_;
    if (available_voltage_ < min_total_voltage)
        available_voltage_ = min_total_voltage;
}

void EulerBattPlugin::PublishVoltage() {
    std_msgs::Float32 voltage_msg;
    voltage_msg.data = available_voltage_;
    voltage_pub_.publish(voltage_msg);
}

void EulerBattPlugin::PublishBatteryState() {
    sensor_msgs::BatteryState battery_state_msg;
    battery_state_msg.voltage = available_voltage_;
    battery_state_msg.current = current_draw_;
    battery_state_msg.capacity = total_capacity_;
    battery_state_msg.percentage = state_of_charge_;
    battery_state_pub_.publish(battery_state_msg);
}

}  // namespace gazebo

// euler_batt_plugin.cpp

#include "euler_plugins/euler_batt_plugin.h"

namespace gazebo {

GZ_REGISTER_MODEL_PLUGIN(EulerBattPlugin)

// Construtor e destrutor
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
        ros::init(argc, argv, "gazebo_battery_plugin", ros::init_options::NoSigintHandler);
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
	// Definir o tópico de tensão (pode ser parametrizado via SDF)
	if (_sdf->HasElement("TopicoTensao"))
		voltage_pub_topic_ = _sdf->Get<std::string>("TopicoTensao");
	else
		voltage_pub_topic_ = "/battery/voltage";

    // Obter link da bateria
    link_ = model_->GetLink(link_name_);
    if (!link_) {
        gzerr << "[euler_batt_plugin] Não foi possível encontrar o link especificado: " << link_name_ << "\n";
        return;
    }

    // Configurar parâmetros da bateria com base no tipo
    ConfigureBatteryParameters();

    // Calcula a massa da bateria e atribui ao link
    CalculateAndSetBatteryMass();

    // Inscrever-se no tópico de corrente
    current_sub_ = nh_->subscribe(current_topic_, 1, &EulerBattPlugin::CurrentCallback, this);

    // Publisher para o tópico /mavros/battery
    battery_state_pub_ = nh_->advertise<mavros_msgs::BatteryStatus>("/mavros/battery", 1);
	sys_status_pub_ = nh_->advertise<mavros_msgs::SysStatus>("/mavros/sys_status", 1);

	// Inicializar o publisher para o tópico de tensão
	voltage_pub_ = nh_->advertise<std_msgs::Float32>(voltage_pub_topic_, 1);

    // Conectar ao evento de atualização do mundo do Gazebo
    update_connection_ = event::Events::ConnectWorldUpdateBegin(
        std::bind(&EulerBattPlugin::OnUpdate, this, std::placeholders::_1));

    std::cout << "[euler_batt_plugin] Plugin carregado com sucesso.\n";

    last_update_time_ = world_->SimTime();
}

void EulerBattPlugin::OnUpdate(const common::UpdateInfo& _info) {
    common::Time current_time = world_->SimTime();
    double dt = (current_time - last_update_time_).Double();

    // Publica o estado da bateria
    //PublishBatteryState();

    // Publica o status do sistema para o ArduPilot
    //PublishSysStatus();

    // Atualiza o parâmetro SIM_BATT_VOLTAGE a cada 10 Hz
    if (dt >= 10) {  // 10 Hz = 100 ms

		// Atualiza o estado de carga (SoC)
		UpdateBatteryState(dt);

		// Publica a tensão disponível
		PublishVoltage();

        SetSimBatteryVoltage(available_voltage_);  // Atualiza o parâmetro SIM_BATT_VOLTAGE com a tensão calculada
        last_update_time_ = current_time;
    }

    //last_update_time_ = current_time;
}

void EulerBattPlugin::ConfigureBatteryParameters() {
    // Exemplo de banco de dados de tipos de bateria
    battery_database_ = {
        {"Welion", {4.2, 35, 0.385}},   // {tensão nominal, capacidade (Ah), massa por célula (kg)}
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
    total_capacity_ = cell_capacity_ * cells_in_parallel_;

    // Estado inicial de carga (100%)
    state_of_charge_ = 0.8;  // 100%
    available_voltage_ = total_voltage_*0.8;
	current_draw_ = 20;	//Debug apenas, arrumar para corrente da avionica
}

void EulerBattPlugin::CalculateAndSetBatteryMass() {
    double total_mass = cell_mass_ * cells_in_series_ * cells_in_parallel_;

    // Obtém o ponteiro para o inercial atual do link
    physics::InertialPtr inertial = link_->GetInertial();

    // Define a massa diretamente
    inertial->SetMass(total_mass);

    gzdbg << "[euler_batt_plugin] Massa da bateria definida para " << total_mass << " kg.\n";
}

void EulerBattPlugin::CurrentCallback(const std_msgs::Float32::ConstPtr& msg) {
    current_draw_ = msg->data;  // Corrente em Amperes
    //current_draw_ = 50;  // Corrente em Amperes
}

void EulerBattPlugin::UpdateBatteryState(double dt) {
    // Atualiza o estado de carga (SoC)
    double discharge = (current_draw_ * dt) / (3600.0 * total_capacity_);  // Normalizado para segundos
    state_of_charge_ -= discharge;
    if (state_of_charge_ < 0.0) state_of_charge_ = 0.0;

    // Atualiza a tensão disponível (simplificação linear)
    available_voltage_ = total_voltage_ * state_of_charge_;
    double min_cell_voltage = 3.2;  // Tensão mínima por célula
    double min_total_voltage = min_cell_voltage * cells_in_series_;
    if (available_voltage_ < min_total_voltage)
        available_voltage_ = min_total_voltage;
}

void EulerBattPlugin::PublishBatteryState() {
    sensor_msgs::BatteryState battery_state_msg;
    battery_state_msg.header.stamp = ros::Time::now();
    battery_state_msg.voltage = available_voltage_;  // Tensão em volts
    battery_state_msg.current = current_draw_;       // Corrente em amperes
    battery_state_msg.charge = std::numeric_limits<float>::quiet_NaN();  // Desconhecido
    battery_state_msg.capacity = std::numeric_limits<float>::quiet_NaN();  // Desconhecido
    battery_state_msg.design_capacity = total_capacity_;  // Capacidade total em Ah
    battery_state_msg.percentage = state_of_charge_;      // Estado de carga (0.0 a 1.0)
    battery_state_msg.power_supply_status = sensor_msgs::BatteryState::POWER_SUPPLY_STATUS_DISCHARGING;
    battery_state_msg.power_supply_health = sensor_msgs::BatteryState::POWER_SUPPLY_HEALTH_GOOD;
    battery_state_msg.power_supply_technology = sensor_msgs::BatteryState::POWER_SUPPLY_TECHNOLOGY_LIPO;
    battery_state_msg.present = true;

    // Publica a mensagem no tópico /mavros/battery
    battery_state_pub_.publish(battery_state_msg);
}

void EulerBattPlugin::PublishVoltage() {
    std_msgs::Float32 voltage_msg;
    voltage_msg.data = available_voltage_;
    voltage_pub_.publish(voltage_msg);
}

void EulerBattPlugin::PublishSysStatus() {
    mavros_msgs::SysStatus sys_status_msg;
    sys_status_msg.header.stamp = ros::Time::now();
    sys_status_msg.voltage_battery = available_voltage_ * 1000;  // Em milivolts
    sys_status_msg.current_battery = current_draw_ * 100;        // Em centiamperes
    sys_status_msg.battery_remaining = state_of_charge_ * 100;   // Estado de carga em porcentagem

    // Publica a mensagem no tópico /mavros/sys_status
    sys_status_pub_.publish(sys_status_msg);
}

void EulerBattPlugin::SetSimBatteryVoltage(float voltage) {
    // Configura a mensagem para alterar o parâmetro SIM_BATT_VOLTAGE
    mavros_msgs::ParamSet param_set_msg;
    param_set_msg.request.param_id = "SIM_BATT_VOLTAGE";
    param_set_msg.request.value.real = voltage;

    // Envia a solicitação para alterar o parâmetro via serviço ROS
    ros::ServiceClient param_set_client = nh_->serviceClient<mavros_msgs::ParamSet>("/mavros/param/set");
    param_set_client.call(param_set_msg);

    if (param_set_msg.response.success) {
        ROS_INFO("Parâmetro SIM_BATT_VOLTAGE atualizado com sucesso: %f", voltage);
    } else {
        ROS_ERROR("Falha ao atualizar o parâmetro SIM_BATT_VOLTAGE");
    }
}


}  // namespace gazebo

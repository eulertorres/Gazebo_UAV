// euler_batt_plugin.cpp

#include "euler_plugins/euler_batt_plugin.h"

namespace gazebo {

GZ_REGISTER_MODEL_PLUGIN(EulerBattPlugin)

// Construtor e destrutor
EulerBattPlugin::EulerBattPlugin() : ModelPlugin(), nh_(nullptr), current_draw_(0.0), script_executed_(false) {}

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

	// Definir o tópico de corrente (pode ser parametrizado via SDF)
	if (_sdf->HasElement("TopicoCorrente"))
		current_pub_topic_ = _sdf->Get<std::string>("TopicoCorrente");
	else
		current_pub_topic_ = "/battery/Total_current";
	// Definir o tópico de tensão (pode ser parametrizado via SDF)
	if (_sdf->HasElement("TopicoTensao"))
		voltage_pub_topic_ = _sdf->Get<std::string>("TopicoTensao");
	else
		voltage_pub_topic_ = "/battery/voltage";

	if (_sdf->HasElement("numRotores"))
		numrotors_ = _sdf->Get<int>("numRotores");
	else
		numrotors_ = 4;

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


    // Busca os tópicos
    ros::master::V_TopicInfo master_topics;
    ros::master::getTopics(master_topics);

	// Inicializar o publisher para o tópico de tensão
	voltage_pub_ = nh_->advertise<std_msgs::Float32>(voltage_pub_topic_, 1);
	current_pub_ = nh_->advertise<std_msgs::Float32>(current_pub_topic_, 1);

    // Conectar ao evento de atualização do mundo do Gazebo
    update_connection_ = event::Events::ConnectWorldUpdateBegin(
        std::bind(&EulerBattPlugin::OnUpdate, this, std::placeholders::_1));

    // Inscrição em tópicos de corrente para cada motor
    for (int i = 1; i <= numrotors_; ++i) {
        std::string topic_name = "/battery/current/motor" + std::to_string(i);

        // Inscreve-se no tópico de corrente do motor
        ros::Subscriber sub = nh_->subscribe<std_msgs::Float32>(
            topic_name, 1, boost::bind(&EulerBattPlugin::CurrentComponentCallback, this, _1, topic_name));
        component_current_subs_.push_back(sub);
        component_currents_[topic_name] = 0.0;  // Inicializa a corrente para este motor
        std::cout << "[euler_batt_plugin] Inscrito no tópico: " << topic_name << "\n";
    }

    std::cout << "[euler_batt_plugin] Plugin carregado com sucesso.\n";
	std::cout.flush(); 
    last_update_time_ = world_->SimTime();
}

void EulerBattPlugin::OnUpdate(const common::UpdateInfo& _info) {
    common::Time current_time = world_->SimTime();
    double dt = (current_time - last_update_time_).Double();

    // Somar as correntes de todos os componentes que estão publicando
    double total_component_current = 0.0;
    for (const auto& component_current : component_currents_) {
        total_component_current += component_current.second;  // Soma as correntes de todos os componentes
    }

    // Adicionar a corrente da eletrônica ou outros componentes fixos (se houver)
    current_draw_ = total_component_current;

    // Atualiza o parâmetro SIM_BATT_VOLTAGE a cada 10 Hz
    if (dt >= 0.1) {  // Periodo

		// Atualiza o estado de carga (SoC)
		UpdateBatteryState(dt);

		// Publica a tensão disponível
		PublishVoltage();

        //SetSimBatteryVoltage(available_voltage_);  // Atualiza o parâmetro SIM_BATT_VOLTAGE com a tensão calculada
        last_update_time_ = current_time;
    }
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
    state_of_charge_ = 1;  // 100%
    available_voltage_ = total_voltage_;
	current_draw_ = 0;	//Debug apenas, arrumar para corrente da avionica
}

void EulerBattPlugin::CalculateAndSetBatteryMass() {
    double total_mass = cell_mass_ * cells_in_series_ * cells_in_parallel_;

    // Obtém o ponteiro para o inercial atual do link
    physics::InertialPtr inertial = link_->GetInertial();

    // Define a massa diretamente
    inertial->SetMass(total_mass);

    gzdbg << "[euler_batt_plugin] Massa da bateria definida para " << total_mass << " kg.\n";
	std::cout.flush(); 
}

void EulerBattPlugin::CurrentComponentCallback(const std_msgs::Float32::ConstPtr& msg, const std::string& topic_name) {
    // Atualiza a corrente consumida pelo componente (motor, IMU, etc.)
    component_currents_[topic_name] = msg->data;
}

void EulerBattPlugin::UpdateBatteryState(double dt) {
    // Atualiza o estado de carga (SoC)
    double discharge = (current_draw_ * dt) / (3600.0 * total_capacity_);  // Normalizado para segundos
    state_of_charge_ -= discharge*1.1;
    if (state_of_charge_ < 0.0) state_of_charge_ = 0.0;

    // Atualiza a tensão disponível (simplificação linear)
    available_voltage_ = total_voltage_ * state_of_charge_;
    double min_cell_voltage = 3.2;  // Tensão mínima por célula
    double min_total_voltage = min_cell_voltage * cells_in_series_;
    //if (available_voltage_ < min_total_voltage)
    //    available_voltage_ = min_total_voltage;
}


void EulerBattPlugin::PublishVoltage() {
    std_msgs::Float32 voltage_msg;
    std_msgs::Float32 current_msg;
    voltage_msg.data = available_voltage_;
    current_msg.data = current_draw_;
    voltage_pub_.publish(voltage_msg);
    current_pub_.publish(current_msg);

/*
    // Executa o script Python apenas uma vez, após a primeira publicação
    if (!script_executed_) {
        script_executed_ = true;

        // Obtenha o caminho do pacote
        std::string package_path = ros::package::getPath("euler_aerodinamica_v1");
        std::string script_path = package_path + "/src/scripts/MAVLink_sender.py";

        // Chama o script MAVLink_sender.py via system()
        std::string command = "python3 " + script_path;
        int ret = system(command.c_str());
        if (ret == -1) {
            gzerr << "Falha ao executar o script MAVLink_sender.py\n";
        } else {
            gzdbg << "Script MAVLink_sender.py executado com sucesso.\n";
        }
		std::cout.flush(); 
    }
*/

}


}  // namespace gazebo

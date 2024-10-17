// euler_batt_plugin.h

#ifndef EULER_BATT_PLUGIN_H
#define EULER_BATT_PLUGIN_H

#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <ros/ros.h>
#include <std_msgs/Float32.h>
#include <map>
#include <ros/master.h>
#include <cstdlib>
#include <ros/package.h> 

namespace gazebo {

class EulerBattPlugin : public ModelPlugin {
public:
    EulerBattPlugin();
    virtual ~EulerBattPlugin();

    void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf);

private:
    void OnUpdate(const common::UpdateInfo& _info);
    void ConfigureBatteryParameters();
    void CalculateAndSetBatteryMass();
    void CurrentComponentCallback(const std_msgs::Float32::ConstPtr& msg, const std::string& topic_name);
    void UpdateBatteryState(double dt);
    void PublishBatteryState();
	void PublishVoltage();
	void CheckForCurrentTopics(const ros::TimerEvent&);

    // Ponteiros e objetos
    physics::ModelPtr model_;
    physics::WorldPtr world_;
    physics::LinkPtr link_;
    event::ConnectionPtr update_connection_;
    ros::NodeHandle* nh_;
    ros::Publisher current_pub_;
    ros::Publisher battery_state_pub_;
	ros::Publisher voltage_pub_;

    // Parâmetros da bateria
    std::string battery_type_;
    int cells_in_series_;
    int cells_in_parallel_;
    int numrotors_;
    double cell_voltage_;
    double cell_capacity_;
    double cell_mass_;
    double total_voltage_;
    double total_capacity_;
    double available_voltage_;
    double state_of_charge_;
    double current_draw_;

	bool script_executed_;

    // Banco de dados de tipos de bateria
    struct BatteryInfo {
        double nominal_voltage;
        double capacity;
        double mass;
    };
    std::map<std::string, BatteryInfo> battery_database_;

    // Outros
	std::vector<ros::Subscriber> component_current_subs_;  // Armazena os subscribers dos componentes
	std::map<std::string, double> component_currents_;     // Armazena as correntes de cada componente, indexado pelo nome do tópico
    std::string link_name_;
    std::string current_topic_;
	std::string voltage_pub_topic_;
	std::string current_pub_topic_;
    common::Time last_update_time_;
};

}  // namespace gazebo

#endif  // EULER_BATT_PLUGIN_H

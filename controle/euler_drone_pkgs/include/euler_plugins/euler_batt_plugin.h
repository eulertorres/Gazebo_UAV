// euler_batt_plugin.h

#ifndef EULER_BATT_PLUGIN_H
#define EULER_BATT_PLUGIN_H

#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <ros/ros.h>
#include <mavros_msgs/SysStatus.h>
#include <sensor_msgs/BatteryState.h>
#include <mavros_msgs/StatusText.h>
#include <mavros_msgs/BatteryStatus.h> 
#include <std_msgs/Float32.h>
#include <map>
#include <mavros_msgs/ParamSet.h>

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
    void CurrentCallback(const std_msgs::Float32::ConstPtr& msg);
    void UpdateBatteryState(double dt);
    void PublishBatteryState();
	void PublishVoltage();
    // Novo: Método para publicar o status do sistema
    void PublishSysStatus();
	void SetSimBatteryVoltage(float voltage);

    // Ponteiros e objetos
    physics::ModelPtr model_;
    physics::WorldPtr world_;
    physics::LinkPtr link_;
    event::ConnectionPtr update_connection_;
    ros::NodeHandle* nh_;
    ros::Subscriber current_sub_;
    ros::Publisher battery_state_pub_;
	ros::Publisher voltage_pub_;
    // Novo: Publisher para o status do sistema
    ros::Publisher sys_status_pub_;


    // Parâmetros da bateria
    std::string battery_type_;
    int cells_in_series_;
    int cells_in_parallel_;
    double cell_voltage_;
    double cell_capacity_;
    double cell_mass_;
    double total_voltage_;
    double total_capacity_;
    double available_voltage_;
    double state_of_charge_;
    double current_draw_;

    // Banco de dados de tipos de bateria
    struct BatteryInfo {
        double nominal_voltage;
        double capacity;
        double mass;
    };
    std::map<std::string, BatteryInfo> battery_database_;

    // Outros
    std::string link_name_;
    std::string current_topic_;
	std::string voltage_pub_topic_;
    common::Time last_update_time_;
};

}  // namespace gazebo

#endif  // EULER_BATT_PLUGIN_H

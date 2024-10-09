// euler_batt_plugin.h

#ifndef EULER_BATT_PLUGIN_H
#define EULER_BATT_PLUGIN_H

#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <ros/ros.h>
#include <std_msgs/Float32.h>
#include <sensor_msgs/BatteryState.h>
#include <map>

namespace gazebo {

class EulerBattPlugin : public ModelPlugin {
public:
    EulerBattPlugin();
    virtual ~EulerBattPlugin();

    void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf);
    void OnUpdate(const common::UpdateInfo& _info);

private:
    void ConfigureBatteryParameters();
    void CalculateAndSetBatteryMass();
    void CurrentCallback(const std_msgs::Float32::ConstPtr& msg);
    void UpdateBatteryState(double dt);
    void PublishVoltage();
    void PublishBatteryState();

    // Estrutura para armazenar informações da bateria
    struct BatteryInfo {
        double nominal_voltage;
        double capacity;  // Em Ah
        double mass;      // Em kg
    };

    // Membros da classe
    physics::ModelPtr model_;
    physics::WorldPtr world_;
    physics::LinkPtr link_;
    event::ConnectionPtr update_connection_;

    // ROS
    ros::NodeHandle* nh_;
    ros::Subscriber current_sub_;
    ros::Publisher voltage_pub_;
    ros::Publisher battery_state_pub_;

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
    double state_of_charge_;  // Entre 0 e 1
    double current_draw_;     // Corrente atual em A

    // Banco de dados de baterias
    std::map<std::string, BatteryInfo> battery_database_;

    // Outros parâmetros
    std::string link_name_;
    std::string current_topic_;
    std::string voltage_pub_topic_;
    std::string battery_state_pub_topic_;

    common::Time last_update_time_;
};

}  // namespace gazebo

#endif // EULER_BATT_PLUGIN_H

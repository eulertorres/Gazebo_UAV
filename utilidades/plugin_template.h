// euler_mot_plugin.h

#ifndef EULER_MOT_PLUGIN_H
#define EULER_MOT_PLUGIN_H

#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>

#include <ros/ros.h>
#include <std_msgs/Float32.h>
#include <mavros_msgs/RCOut.h>

namespace gazebo {

class EulerMotPlugin : public ModelPlugin {
public:
    EulerMotPlugin();
    virtual ~EulerMotPlugin();

    virtual void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf);

protected:
    virtual void OnUpdate(const common::UpdateInfo& _info);

private:
    // Funções de callback
    void PwmCallback(const mavros_msgs::RCOut::ConstPtr& msg);
    void VoltageCallback(const std_msgs::Float32::ConstPtr& msg);

    // Funções para publicar dados
    void PublishCurrent();
    void PublishTorque();
    void PublishRPM();

    // Ponteiros para o modelo, mundo e articulação
    physics::ModelPtr model_;
    physics::WorldPtr world_;
    physics::JointPtr joint_;

    // Conexão de atualização
    event::ConnectionPtr update_connection_;

    // NodeHandle do ROS
    ros::NodeHandle* nh_;

    // Subscritores e publicadores
    ros::Subscriber pwm_sub_;
    ros::Subscriber voltage_sub_;
    ros::Publisher current_pub_;
    ros::Publisher torque_pub_;
    ros::Publisher rpm_pub_;

    // Variáveis para armazenar valores
    int motor_number_;
    double pwm_value_;
    double pwm_min_;
    double pwm_max_;
    double battery_voltage_;

    // Constantes do motor
    double KV_rpm_;
    double KV_rad_;
    double Kt_;
    double R_;
    double L_;
    double i0_;
    double n_esc_;

    // Constantes da hélice
    double Iz_;
    double a_;
    double b_;

    // Variáveis de estado
    double omega_;
    double i_;
    double t_;

    // Outras variáveis
    double torque_;
    double current_;
    double rpm;

    std::string joint_name_;
    std::string current_topic_;
    std::string torque_topic_;
    std::string rpm_topic_;
    std::string orientation_;

    double force_update_rate_;
    double pwm_timeout_;

    common::Time last_pwm_received_time_;
    common::Time last_update_time_;
    common::Time last_integration_time_;
};

}  // namespace gazebo

#endif  // EULER_MOT_PLUGIN_H

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
    void calcula(double dt);
	void PrintDebug(double duty, double Vq, double dIq_dt, double desired_angular_velocity, double error, double IE, double DE);

	// Funções intde ambiente
	double GetMaxCurrentForAWG(int awg);

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
    ros::Publisher rpm_des_pub_;
    ros::Publisher vq_pub_;
    ros::Publisher vq_des_pub_;

    // Variáveis para armazenar valores
    int motor_number_;
    int wire_gauge_awg_;
    double pwm_value_;
    double pwm_min_;
    double pwm_max_;
    double battery_voltage_;
    double max_angular_velocity;

    // Constantes do motor
    double KV_rpm_;
    double KV_rad_;
    double i0_;
    double n_esc_;
    double current_;        // Current in the q-axis
    double Kt_;             // Torque constant (Nm/A)
    double Kbq_;            // Back-EMF constant (V/(rad/s))
    double R_;              // Phase resistance (Ohms)
    double L_;              // Phase inductance (Henrys)
    double b_;              // Damping coefficient (N·m·s/rad)
    double angular_velocity_; // Angular velocity (rad/s)
    double rpm_;            // Rotational speed (RPM)
    double rpm_des;            // Rotational speed (RPM)
    double max_current_;            // 
    
	//PID controle PWM
    double Kp;
    double Ki;
    double Kd;
    double integral_error_;
    double prev_error_;

	bool is_delta_;            // Delta ou estrela
	bool debug_;            // Delta ou estrela

    // Variáveis de estado
    double omega_;
    double i_;
    double t_;

    // Outras variáveis
    double torque_;

    std::string joint_name_;
    std::string current_topic_;
    std::string torque_topic_;
    std::string rpm_topic_;
    std::string orientation_;

    double force_update_rate_;
    double pwm_timeout_;

    common::Time last_pwm_received_time_;
    common::Time last_update_time_;
    common::Time last_publish_time_;
};

}  // namespace gazebo

#endif  // EULER_MOT_PLUGIN_H

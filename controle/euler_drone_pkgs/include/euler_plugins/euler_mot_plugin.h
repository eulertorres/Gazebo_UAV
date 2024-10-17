// euler_mot_plugin.h

#ifndef EULER_MOT_PLUGIN_H
#define EULER_MOT_PLUGIN_H

// Inclui as bibliotecas do Gazebo
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>

// Inclui as bibliotecas do ROS
#include <ros/ros.h>
#include <std_msgs/Float32.h>
#include <mavros_msgs/RCOut.h>

// Outras bibliotecas necessárias
#include <cmath>
#include <string>

namespace gazebo {

// Declaração da classe EulerMotPlugin que herda de ModelPlugin
class EulerMotPlugin : public ModelPlugin {
public:
    // Construtor e destrutor
    EulerMotPlugin();
    virtual ~EulerMotPlugin();

    // Função chamada quando o plugin é carregado
    void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf);

private:
    // Função chamada a cada atualização do mundo
    void OnUpdate(const common::UpdateInfo& _info);

    // Funções auxiliares para publicar dados
    void PublishCurrent();
    void PublishTorque();
    void PublishRPM();

    // Callback para receber os valores de PWM
    void PwmCallback(const mavros_msgs::RCOut::ConstPtr& msg);

    // Callback para receber a tensão da bateria
    void VoltageCallback(const std_msgs::Float32::ConstPtr& msg);

    // Ponteiros e objetos
    physics::ModelPtr model_;             // Ponteiro para o modelo
    physics::WorldPtr world_;             // Ponteiro para o mundo
    physics::JointPtr joint_;             // Ponteiro para a junta (alterado de link_ para joint_)
    event::ConnectionPtr update_connection_; // Conexão para a função de atualização
    ros::NodeHandle* nh_;                 // NodeHandle do ROS
    ros::Publisher current_pub_;          // Publisher para a corrente
    ros::Publisher torque_pub_;           // Publisher para o torque
    ros::Publisher rpm_pub_;              // Publisher para o RPM

    // Parâmetros do motor
    int motor_number_;            // Número do motor
    double pwm_value_;            // Valor atual do PWM
    double battery_voltage_;      // Tensão atual da bateria
    double current_;              // Corrente atual
    double torque_;               // Torque atual
    double rpm;                   // RPM atual
    double pwm_min_;              // Valor mínimo do PWM
    double pwm_max_;              // Valor máximo do PWM
    double torque_coeff_;         // Coeficiente para cálculo do torque
    double current_coeff_;        // Coeficiente para cálculo da corrente
    double force_update_rate_;    // Taxa de atualização da força
    double drag_coeff_;           // Coeficiente de arrasto
    double back_emf_coeff_;       // Coeficiente de força contra-eletromotriz
    double pwm_timeout_;          // Tempo limite para recepção do PWM
    std::string joint_name_;      // Nome da junta (alterado de link_name_ para joint_name_)
    std::string current_topic_;   // Nome do tópico para publicar a corrente
    std::string torque_topic_;    // Nome do tópico para publicar o torque
    std::string rpm_topic_;       // Nome do tópico para publicar o RPM
    std::string orientation_;     // Orientação do motor ("CW" ou "CCW")

    common::Time last_update_time_;       // Tempo da última atualização
    common::Time last_pwm_received_time_; // Tempo em que o último PWM foi recebido

    // Subscriptores
    ros::Subscriber pwm_sub_;     // Subscriber para o PWM
    ros::Subscriber voltage_sub_; // Subscriber para a tensão da bateria
};

}  // namespace gazebo

#endif  // EULER_MOT_PLUGIN_H

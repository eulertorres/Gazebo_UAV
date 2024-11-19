// euler_mot_plugin_v2.h

#ifndef EULER_MOT_PLUGIN_V2_H
#define EULER_MOT_PLUGIN_V2_H

#include <map>
#include <vector>
#include <string>
#include <mutex>
#include <sdf/sdf.hh>

#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <ros/ros.h>

#include <std_msgs/Float32.h>
#include <std_msgs/Float32MultiArray.h>
#include <sensor_msgs/JointState.h>

#ifdef _WIN32
  #include <Winsock2.h>
  #include <Ws2def.h>
  #include <Ws2ipdef.h>
  #include <Ws2tcpip.h>
  typedef SSIZE_T ssize_t;
#else
  #include <sys/socket.h>
  #include <netinet/in.h>
  #include <arpa/inet.h>
  #include <fcntl.h>
  #include <unistd.h>
#endif

namespace gazebo {

#define MAX_MOTORS 255

// Estrutura para armazenar comandos de PWM recebidos via socket
struct ServoPacket {
    float motorSpeed[MAX_MOTORS] = {0.0f};
};

// Classe para lidar com comunicação via socket
class EulerMotSocketPrivate {
public:
    EulerMotSocketPrivate();
    ~EulerMotSocketPrivate();

    bool Bind(const char *_address, const uint16_t _port);
    void MakeSockAddr(const char *_address, const uint16_t _port, struct sockaddr_in &_sockaddr);
    ssize_t Recv(void *_buf, const size_t _size, uint32_t _timeoutMs);

private:
    int fd;
};

// Estrutura para armazenar informações de cada motor
struct MotorControl {
    int channel;
    physics::JointPtr joint;
    double cmd; // Comando recebido
    std::string joint_name;
    double multiplier;
    double offset;

    // Variáveis específicas do motor
    double pwm_min;
    double pwm_max;
    double KV_rpm;
    double KV_rad;
    double R;
    double L;
    double i0;
    double n_esc;
    double b;
    std::string orientation;
    int wire_gauge_awg;
    double Kp, Ki, Kd;
    double Kt;
    double Kbq;
    double max_current;
    double battery_voltage;
    double current;
    double torque;
    double angular_velocity;
    double rpm;
    double rpm_des;
    bool is_delta;
    double integral_error;
    double prev_error;
    double max_angular_velocity;
    bool debug;

    // Construtor padrão
    MotorControl();
};

// Classe principal do plugin
class EulerMotPluginV2 : public ModelPlugin {
public:
    EulerMotPluginV2();
    virtual ~EulerMotPluginV2();

    void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf);
    void OnUpdate(const common::UpdateInfo& _info);

private:
    void ReceiveMotorCommand();
    void ApplyMotorForces(double dt);
    bool InitSockets(sdf::ElementPtr _sdf);
    void ParseMotorSDF(sdf::ElementPtr _sdf);
    double GetMaxCurrentForAWG(int awg);

    physics::ModelPtr model_;
    physics::WorldPtr world_;
    event::ConnectionPtr update_connection_;

    // Sockets
    EulerMotSocketPrivate socket_in_;

    // Endereços e portas
    std::string fdm_addr_;
    std::string listen_addr_;
	unsigned int fdm_port_in_;
	unsigned int fdm_port_out_;

    // Vetor de motores
    std::vector<MotorControl> motors_;

    ros::Publisher motor_state_pub_;
    ros::Publisher motor_cmd_pub_;
    ros::Publisher joint_state_pub_;
#include <std_msgs/Float32.h>
#include <std_msgs/Float32MultiArray.h>
#include <sensor_msgs/JointState.h>


    // Tempo
    common::Time last_update_time_;
    common::Time last_publish_time_;

    // ROS NodeHandle (se ainda precisar para outras funcionalidades)
    ros::NodeHandle* nh_;
};

}  // namespace gazebo

#endif  // EULER_MOT_PLUGIN_V2_H

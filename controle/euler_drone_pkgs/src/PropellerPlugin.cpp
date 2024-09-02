#include <gazebo/gazebo.hh>
#include <gazebo/common/Plugin.hh>
#include <gazebo/physics/physics.hh>
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <thread>

namespace gazebo
{
  class PropellerPlugin : public ModelPlugin
  {
  public:
    PropellerPlugin() : nh_(nullptr) {}
    ~PropellerPlugin() { nh_->shutdown(); }

    void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
    {
      std::cout << "PropellerPlugin from catkin workspace is loaded." << std::endl;

      if (!ros::isInitialized())
      {
        int argc = 0;
        char** argv = nullptr;
        ros::init(argc, argv, "gazebo_client", ros::init_options::NoSigintHandler);
      }

      // Criar o NodeHandle depois de inicializar o ROS
      nh_ = new ros::NodeHandle();

      // Verifique se o ROS Master está acessível
      if (!nh_->ok())
      {
        ROS_FATAL_STREAM("Falha ao contactar o ROS Master. Certifique-se de que o roscore está rodando.");
        return;
      }

      ROS_INFO("O ROS TA VIVO REPITO O ROS TA VIVO");

      this->model = _model;

      // Check and retrieve SDF parameters
      if (_sdf->HasElement("frame_id"))
        frame_id_ = _sdf->Get<std::string>("frame_id");
      else
        frame_id_ = "pa_helice";

      if (_sdf->HasElement("publish_force"))
        publish_force_ = _sdf->Get<bool>("publish_force");
      else
        publish_force_ = true;

      if (_sdf->HasElement("prop_force_topic"))
        propeller_force_topic_ = _sdf->Get<std::string>("prop_force_topic");
      else
        propeller_force_topic_ = "/prop/force";

      // Publicador de mensagens
      if (publish_force_)
      {
        propeller_force_publisher_ = nh_->advertise<std_msgs::String>(propeller_force_topic_, 1);
        ROS_INFO("Publicando dados no topico: %s", propeller_force_topic_.c_str());
      }

      // Conectar-se ao evento de atualização
      this->update_connection_ = event::Events::ConnectWorldUpdateBegin(std::bind(&PropellerPlugin::OnUpdate, this));

      ROS_INFO("Plugin Carregado Com Sucesso");
    }

  private:
    void OnUpdate()
    {
      common::Time current_time = model->GetWorld()->SimTime();
      double dt = (current_time - last_update_time_).Double();

      if (dt > update_period_)
      {
       //#ROS_INFO("OnUpdate chamado");
        CalculateForces(); // Publica a mensagem "Hello from PropellerPlugin!"
        last_update_time_ = current_time;
      }
    }

    void CalculateForces()
    {
      std_msgs::String msg;
      msg.data = "Hello from PropellerPlugin!";
      propeller_force_publisher_.publish(msg);
      //ROS_INFO("Mensagem publicada: %s", msg.data.c_str());
    }

    // Variáveis membro
    physics::ModelPtr model;
    std::string frame_id_;
    std::string propeller_force_topic_;
    ros::NodeHandle* nh_;
    ros::Publisher propeller_force_publisher_;
    event::ConnectionPtr update_connection_;
    common::Time last_update_time_;
    double update_period_ = 0.1; // exemplo de período de atualização
    bool publish_force_ = true;
  };

  GZ_REGISTER_MODEL_PLUGIN(PropellerPlugin)
}

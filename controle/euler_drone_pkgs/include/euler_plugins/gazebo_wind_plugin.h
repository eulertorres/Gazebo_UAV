/*
codigo do Euler
 */

#ifndef ROTORS_GAZEBO_PLUGINS_GAZEBO_WIND_PLUGIN_H
#define ROTORS_GAZEBO_PLUGINS_GAZEBO_WIND_PLUGIN_H

#include <string>
#include <vector>

#include <gazebo/common/common.hh>
#include <gazebo/common/Plugin.hh>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <ignition/math/Vector3.hh>

#include <ros/ros.h>
#include <geometry_msgs/WrenchStamped.h>
#include <geometry_msgs/Vector3.h>

namespace gazebo {

// Valores padrão
static const std::string kDefaultFrameId = "world";
static const std::string kDefaultLinkName = "base_link";
static const std::string kDefaultWindSpeedPubTopic = "wind_speed";

static constexpr double kDefaultWindForceMean = 0.0;
static constexpr double kDefaultWindForceVariance = 0.0;
static constexpr double kDefaultWindGustForceMean = 0.0;
static constexpr double kDefaultWindGustForceVariance = 0.0;

static constexpr double kDefaultWindGustStart = 10.0;
static constexpr double kDefaultWindGustDuration = 0.0;

static constexpr double kDefaultWindSpeedMean = 0.0;
static constexpr double kDefaultWindSpeedVariance = 0.0;

static const ignition::math::Vector3d kDefaultWindDirection = ignition::math::Vector3d(1, 0, 0);
static const ignition::math::Vector3d kDefaultWindGustDirection = ignition::math::Vector3d(0, 1, 0);

static constexpr bool kDefaultUseCustomStaticWindField = false;

/// \brief    Este plugin do gazebo simula o vento agindo sobre um modelo.
/// \details  Este plugin publica em um tópico ROS as informações do vento.
class GazeboWindPlugin : public ModelPlugin {
 public:
  GazeboWindPlugin();
  virtual ~GazeboWindPlugin();

 protected:

  /// \brief Carrega o plugin.
  /// \param[in] _model Ponteiro para o modelo que carregou este plugin.
  /// \param[in] _sdf Elemento SDF que descreve o plugin.
  void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf) override;

  /// \brief Chamado quando o mundo é atualizado.
  /// \param[in] _info Informação de tempo de atualização.
  void OnUpdate(const common::UpdateInfo& _info);

 private:

  /// \brief Cria todos os publishers e subscribers necessários.
  void CreatePubsAndSubs();

  /// \brief Lê os dados do vento de um arquivo de texto e os salva.
  /// \param[in] custom_wind_field_path Caminho para o arquivo de campo de vento em ~/.ros.
  void ReadCustomWindField(std::string& custom_wind_field_path);

  /// \brief Interpolação linear
  ignition::math::Vector3d LinearInterpolation(double position, ignition::math::Vector3d* values, double* points) const;

  /// \brief Interpolação bilinear
  ignition::math::Vector3d BilinearInterpolation(double* position, ignition::math::Vector3d* values, double* points) const;

  /// \brief Interpolação trilinear
  ignition::math::Vector3d TrilinearInterpolation(ignition::math::Vector3d link_position, ignition::math::Vector3d* values, double* points) const;

  /// Conexão para o evento de atualização
  event::ConnectionPtr update_connection_;

  /// Ponteiros para o mundo, modelo e link do gazebo
  physics::WorldPtr world_;
  physics::ModelPtr model_;
  physics::LinkPtr link_;

  /// Node handle do ROS
  ros::NodeHandle nh_;

  /// Publishers do ROS
  ros::Publisher wind_force_pub_;
  ros::Publisher wind_speed_pub_;

  /// Parâmetros do plugin
  std::string namespace_;
  std::string frame_id_;
  std::string link_name_;
  std::string wind_force_pub_topic_;
  std::string wind_speed_pub_topic_;

  double wind_force_mean_;
  double wind_force_variance_;
  double wind_gust_force_mean_;
  double wind_gust_force_variance_;
  double wind_speed_mean_;
  double wind_speed_variance_;

  ignition::math::Vector3d xyz_offset_;
  ignition::math::Vector3d wind_direction_;
  ignition::math::Vector3d wind_gust_direction_;

  common::Time wind_gust_end_;
  common::Time wind_gust_start_;

  /// Variáveis para geração de campo de vento personalizado
  bool use_custom_static_wind_field_;
  float min_x_;
  float min_y_;
  int n_x_;
  int n_y_;
  float res_x_;
  float res_y_;
  std::vector<float> vertical_spacing_factors_;
  std::vector<float> bottom_z_;
  std::vector<float> top_z_;
  std::vector<float> u_;
  std::vector<float> v_;
  std::vector<float> w_;

  /// Estruturas para mensagens de vento e força
  struct WindSpeed {
    std::string header_frame_id;
    ignition::math::Vector3d velocity;
  } wind_speed_msg_;

  struct WrenchStamped {
    std::string header_frame_id;
    ignition::math::Vector3d force;
    ignition::math::Vector3d torque;
  } wrench_stamped_msg_;
};

}  // namespace gazebo

#endif  // ROTORS_GAZEBO_PLUGINS_GAZEBO_WIND_PLUGIN_H

/*
	Plugin para simulação de forças aerondinâmicas no Gazebo
	Desenvolvido por Euler Torres Feitosa (euler.feitosa@xmobots.com.br)
 */

#include "euler_plugins/euler_aero_plugin.h"

#include <fstream>
#include <math.h>
#include <ros/ros.h>
#include <geometry_msgs/WrenchStamped.h>
#include <geometry_msgs/Vector3.h>

namespace gazebo {

EulerAeroPlugin::EulerAeroPlugin() {
  
}

EulerAeroPlugin::~EulerAeroPlugin() {
  
}

void PublishWrenchStamped(ros::Publisher& pub, const std::string& frame_id,
                          const ignition::math::Vector3d& force,
                          const ignition::math::Vector3d& torque = ignition::math::Vector3d::Zero) {
    geometry_msgs::WrenchStamped wrench_msg;
    wrench_msg.header.frame_id = frame_id;
    wrench_msg.header.stamp = ros::Time::now();
    wrench_msg.wrench.force.x = force.X();
    wrench_msg.wrench.force.y = force.Y();
    wrench_msg.wrench.force.z = force.Z();
    wrench_msg.wrench.torque.x = torque.X();
    wrench_msg.wrench.torque.y = torque.Y();
    wrench_msg.wrench.torque.z = torque.Z();
    pub.publish(wrench_msg);
}


void PublishVector3(ros::Publisher& pub, const ignition::math::Vector3d& vector) {
    geometry_msgs::Vector3 vector_msg;

    vector_msg.x = vector.X();
    vector_msg.y = vector.Y();
    vector_msg.z = vector.Z();

    pub.publish(vector_msg);
}

void PublishWindAsWrench(ros::Publisher& pub, const std::string& frame_id, 
                         const ignition::math::Vector3d& wind_velocity) {
    geometry_msgs::WrenchStamped wrench_msg;
    wrench_msg.header.frame_id = frame_id;
    wrench_msg.header.stamp = ros::Time::now();

    // Considerando massa de 1 kg para simplificar
    wrench_msg.wrench.force.x = wind_velocity.X();
    wrench_msg.wrench.force.y = wind_velocity.Y();
    wrench_msg.wrench.force.z = wind_velocity.Z();

    // Torque pode ser considerado zero, se necessário
    wrench_msg.wrench.torque.x = 0;
    wrench_msg.wrench.torque.y = 0;
    wrench_msg.wrench.torque.z = 0;

    pub.publish(wrench_msg);
}

void EulerAeroPlugin::CalculaCustom() {
    ignition::math::Vector3d wind_velocity(0.0, 0.0, 0.0);
    ignition::math::Vector3d link_position = link_->WorldPose().Pos();

    std::size_t x_inf = floor((link_position.X() - min_x_) / res_x_);
    std::size_t y_inf = floor((link_position.Y() - min_y_) / res_y_);

    if (x_inf == n_x_ - 1u) {
        x_inf = n_x_ - 2u;
    }
    if (y_inf == n_y_ - 1u) {
        y_inf = n_y_ - 2u;
    }

    std::size_t x_sup = x_inf + 1u;
    std::size_t y_sup = y_inf + 1u;

    constexpr unsigned int n_vertices = 8;
    std::size_t idx_x[n_vertices] = {x_inf, x_inf, x_sup, x_sup, x_inf, x_inf, x_sup, x_sup};
    std::size_t idx_y[n_vertices] = {y_inf, y_inf, y_inf, y_inf, y_sup, y_sup, y_sup, y_sup};

    constexpr unsigned int n_columns = 4;
    float vertical_factors_columns[n_columns];
    for (std::size_t i = 0u; i < n_columns; ++i) {
        vertical_factors_columns[i] = (
            link_position.Z() - bottom_z_[idx_x[2u * i] + idx_y[2u * i] * n_x_]) /
            (top_z_[idx_x[2u * i] + idx_y[2u * i] * n_x_] - bottom_z_[idx_x[2u * i] + idx_y[2u * i] * n_x_]);
    }
    
    float vertical_factors_min = *std::min_element(vertical_factors_columns, vertical_factors_columns + n_columns);
    float vertical_factors_max = *std::max_element(vertical_factors_columns, vertical_factors_columns + n_columns);

    if (x_inf >= 0u && y_inf >= 0u && vertical_factors_max >= 0u && 
        x_sup <= (n_x_ - 1u) && y_sup <= (n_y_ - 1u) && vertical_factors_min <= 1u) {
        std::size_t idx_z[n_vertices] = {0u, static_cast<int>(vertical_spacing_factors_.size()) - 1u,
                                  0u, static_cast<int>(vertical_spacing_factors_.size()) - 1u,
                                  0u, static_cast<int>(vertical_spacing_factors_.size()) - 1u,
                                  0u, static_cast<int>(vertical_spacing_factors_.size()) - 1u};
        for (std::size_t i = 0u; i < n_columns; ++i) {
            if (vertical_factors_columns[i] < 0u) {
                idx_z[2u * i + 1u] = 1u;
            } else if (vertical_factors_columns[i] >= 1u) {
                idx_z[2u * i] = vertical_spacing_factors_.size() - 2u;
            } else {
                for (std::size_t j = 0u; j < vertical_spacing_factors_.size() - 1u; ++j) {
                    if (vertical_spacing_factors_[j] <= vertical_factors_columns[i] && 
                        vertical_spacing_factors_[j + 1u] > vertical_factors_columns[i]) {
                        idx_z[2u * i] = j;
                        idx_z[2u * i + 1u] = j + 1u;
                        break;
                    }
                }
            }
        }

        ignition::math::Vector3d wind_at_vertices[n_vertices];
        for (std::size_t i = 0u; i < n_vertices; ++i) {
            wind_at_vertices[i].X() = u_[idx_x[i] + idx_y[i] * n_x_ + idx_z[i] * n_x_ * n_y_];
            wind_at_vertices[i].Y() = v_[idx_x[i] + idx_y[i] * n_x_ + idx_z[i] * n_x_ * n_y_];
            wind_at_vertices[i].Z() = w_[idx_x[i] + idx_y[i] * n_x_ + idx_z[i] * n_x_ * n_y_];
        }

        constexpr unsigned int n_points_interp_z = 8;
        constexpr unsigned int n_points_interp_x = 4;
        constexpr unsigned int n_points_interp_y = 2;
        double interpolation_points[n_points_interp_x + n_points_interp_y + n_points_interp_z];
        for (std::size_t i = 0u; i < n_points_interp_x + n_points_interp_y + n_points_interp_z; ++i) {
            if (i < n_points_interp_z) {
                interpolation_points[i] = (
                    top_z_[idx_x[i] + idx_y[i] * n_x_] - bottom_z_[idx_x[i] + idx_y[i] * n_x_])
                    * vertical_spacing_factors_[idx_z[i]] + bottom_z_[idx_x[i] + idx_y[i] * n_x_];
            } else if (i >= n_points_interp_z && i < n_points_interp_x + n_points_interp_z) {
                interpolation_points[i] = min_x_ + res_x_ * idx_x[2u * (i - n_points_interp_z)];
            } else {
                interpolation_points[i] = min_y_ + res_y_ * idx_y[4u * (i - n_points_interp_z - n_points_interp_x)];
            }
        }

        wind_velocity = TrilinearInterpolation(link_position, wind_at_vertices, interpolation_points);
    } else {
        wind_velocity = wind_speed_mean_ * wind_direction_;
    }

    geometry_msgs::Vector3 wind_speed_msg;
    wind_speed_msg.x = wind_velocity.X();
    wind_speed_msg.y = wind_velocity.Y();
    wind_speed_msg.z = wind_velocity.Z();

    wind_speed_pub_.publish(wind_speed_msg);
}

//Carrega o plugin
void EulerAeroPlugin::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf) {

	// Armazena poneitro pro modelo
	model_ = _model;
	world_ = model_->GetWorld();

	std::cout << "EulerAeroPlugin from catkin workspace is loaded." << std::endl;

	if (!ros::isInitialized())
	{
		int argc = 0;
		char** argv = nullptr;
		ros::init(argc, argv, "gazebo_client", ros::init_options::NoSigintHandler);
	}

	// Criar o NodeHandle depois de inicializar o ROS
	/// Node handle do ROS
	ros::NodeHandle nh;

	ROS_INFO("O ROS TA VIVO REPITO O ROS TA VIVO");

    // Conecte-se ao evento de atualização do mundo do Gazebo
    update_connection_ = event::Events::ConnectWorldUpdateBegin(
        std::bind(&EulerAeroPlugin::OnUpdate, this, std::placeholders::_1));

	double wind_gust_start = kDefaultWindGustStart;
	double wind_gust_duration = kDefaultWindGustDuration;
	rho = 1.225;
	
	//==============================================//
//========== Ler os parâmetros do arquivo SDF ===========//
	//==============================================//

	std::string custom_wind_field_path; // Declaração da variável

	if (_sdf->HasElement("robotNamespace"))
	namespace_ = _sdf->GetElement("robotNamespace")->Get<std::string>();
	else
	gzerr << "[euler_aero_plugin] Especifique seu robotNamespace.\n";

	if (_sdf->HasElement("xyzOffset"))
	xyz_offset_ = _sdf->GetElement("xyzOffset")->Get<ignition::math::Vector3d >();
	else
	gzerr << "[euler_aero_plugin] Especifique xyzOffset.\n";

	if (_sdf->HasElement("windForcePubTopic"))
		wind_force_pub_topic_ = _sdf->Get<std::string>("windForcePubTopic");
	else
		wind_force_pub_topic_ = "wind_force_topic";

	if (_sdf->HasElement("windSpeedPubTopic"))
		wind_speed_pub_topic_ = _sdf->Get<std::string>("windSpeedPubTopic");
	else
		wind_speed_pub_topic_ = "wind_speed_topic";

	if (_sdf->HasElement("frameId"))
		frame_id_ = _sdf->Get<std::string>("frameId");
	else
		frame_id_ = kDefaultFrameId;

	if (_sdf->HasElement("linkName"))
		link_name_ = _sdf->Get<std::string>("linkName");
	else
		link_name_ = kDefaultLinkName;

	if (_sdf->HasElement("windSpeedMean"))
		wind_speed_mean_ = _sdf->Get<double>("windSpeedMean");
	else
		wind_speed_mean_ = kDefaultWindSpeedMean;

	if (_sdf->HasElement("windSpeedVariance"))
		wind_speed_variance_ = _sdf->Get<double>("windSpeedVariance"); // Não usado atualmente
	else
		wind_speed_variance_ = kDefaultWindSpeedVariance;

	if (_sdf->HasElement("windDirection"))
		wind_direction_ = _sdf->Get<ignition::math::Vector3d>("windDirection");
	else
		wind_direction_ = kDefaultWindDirection;

	if (_sdf->HasElement("useCustomStaticWindField"))
		use_custom_static_wind_field_ = _sdf->Get<bool>("useCustomStaticWindField");
	else
		use_custom_static_wind_field_ = kDefaultUseCustomStaticWindField;

	if (_sdf->HasElement("frontArea"))								//Adicionar
		front_area_ = _sdf->Get<double>("frontArea");
	else
		front_area_ = kDefaultFrontArea;				//Adicionar

	if (_sdf->HasElement("dragCoeff"))								//Adicionar
		drag_coeff_ = _sdf->Get<double>("dragCoeff");
	else
		drag_coeff_ = kDefaultDragCoeff;				//Adicionar

	if (_sdf->HasElement("forceUpdateRate"))								//Adicionar
		force_update_rate_ = _sdf->Get<double>("forceUpdateRate");
	else
		force_update_rate_ = kDefaultUpdate;				//Adicionar

	if (!use_custom_static_wind_field_) {
		gzdbg << "[euler_aero_plugin] Using user-defined constant wind field and gusts.\n";

		if (_sdf->HasElement("windForceVariance"))
			wind_force_variance_ = _sdf->Get<double>("windForceVariance"); // Não usado atualmente
		else
			wind_force_variance_ = kDefaultWindForceVariance;

		if (_sdf->HasElement("windGustStart"))
			wind_gust_start = _sdf->Get<double>("windGustStart");
		else
			wind_gust_start = kDefaultWindGustStart;

		if (_sdf->HasElement("windGustDuration"))
			wind_gust_duration = _sdf->Get<double>("windGustDuration");
		else
			wind_gust_duration = kDefaultWindGustDuration;

		if (_sdf->HasElement("windGustForceMean"))
			wind_gust_force_mean_ = _sdf->Get<double>("windGustForceMean");
		else
			wind_gust_force_mean_ = kDefaultWindGustForceMean;

		if (_sdf->HasElement("windGustForceVariance"))
			wind_gust_force_variance_ = _sdf->Get<double>("windGustForceVariance"); // Não usado atualmente
		else
			wind_gust_force_variance_ = kDefaultWindGustForceVariance;

		if (_sdf->HasElement("windGustDirection"))
			wind_gust_direction_ = _sdf->Get<ignition::math::Vector3d>("windGustDirection");
		else
			wind_gust_direction_ = kDefaultWindGustDirection;

		wind_direction_.Normalize();
		wind_gust_direction_.Normalize();
		wind_gust_start_ = common::Time(wind_gust_start);
		wind_gust_end_ = common::Time(wind_gust_start + wind_gust_duration);
	} else {
		gzdbg << "[euler_aero_plugin] Using custom wind field from text file.\n";

		if (_sdf->HasElement("customWindFieldPath"))
			custom_wind_field_path = _sdf->Get<std::string>("customWindFieldPath");
		else
			custom_wind_field_path = ""; // Ou algum valor padrão que você preferir

		ReadCustomWindField(custom_wind_field_path);
	}

	link_ = model_->GetLink(link_name_);
	if (link_ == NULL)
	gzthrow("[euler_aero_plugin] Couldn't find specified link \"" << link_name_
																   << "\".");

	// Initialize ROS publishers
	drag_force_pub_ = nh.advertise<geometry_msgs::WrenchStamped>("vento/drag", 1);
	resultant_force_pub_ = nh.advertise<geometry_msgs::WrenchStamped>("vento/resultante", 1);
	wind_as_force_pub_ = nh.advertise<geometry_msgs::WrenchStamped>(wind_speed_pub_topic_, 1);
	wind_force_pub_ = nh.advertise<geometry_msgs::WrenchStamped>(wind_force_pub_topic_, 1);
	// Listen to the update event. This event is broadcast every
	// simulation iteration.
	update_connection_ = event::Events::ConnectWorldUpdateBegin(
	  boost::bind(&EulerAeroPlugin::OnUpdate, this, _1));
}

void EulerAeroPlugin::OnUpdate(const common::UpdateInfo& _info) {
    common::Time now = world_->SimTime();

    // Velocidade atual do drone
    ignition::math::Vector3d drone_velocity = link_->WorldLinearVel();

	// Calcula a velocidade do vento
    ignition::math::Vector3d wind_velocity = wind_speed_mean_ * wind_direction_;
    ignition::math::Vector3d relative_wind_velocity = wind_velocity - drone_velocity;
	
    ignition::math::Vector3d wind_force(0.0, 0.0, 0.0);
    ignition::math::Vector3d drag_force(0.0, 0.0, 0.0);


    if (use_custom_static_wind_field_) {
        CalculaCustom();
        return;
    }

    // Calcula a força de arrasto considerando apenas a velocidade do drone
    if (drone_velocity.Length() > 0) {
        double v_squared = drone_velocity.Length() * drone_velocity.Length();
        double drag_force_magnitude = 0.5 * rho * drag_coeff_ * front_area_ * v_squared;
        drag_force = -drag_force_magnitude * drone_velocity.Normalized();
    }

    // Calcula a força do vento considerando a velocidade relativa
    if (relative_wind_velocity.Length() > 0) {
        double v_squared = relative_wind_velocity.Length() * relative_wind_velocity.Length();
        double wind_force_magnitude = 0.5 * rho * front_area_ * v_squared;
        wind_force = wind_force_magnitude * relative_wind_velocity.Normalized();
    }

    // Aplica as forças calculadas ao drone
    link_->AddForceAtRelativePosition(drag_force + wind_force, xyz_offset_);

    if ((now - last_force_update_time_).Double() >= (1.0 / force_update_rate_)) {
        // Publicar as forças calculadas pra debug
        PublishWrenchStamped(drag_force_pub_, frame_id_, drag_force);
        PublishWrenchStamped(wind_force_pub_, frame_id_, wind_force);
        PublishWrenchStamped(resultant_force_pub_, frame_id_, drag_force + wind_force);
        PublishWindAsWrench(wind_as_force_pub_, frame_id_, wind_velocity);
        last_force_update_time_ = now;
    }
}



void EulerAeroPlugin::ReadCustomWindField(std::string& custom_wind_field_path) {
  std::ifstream fin;
  fin.open(custom_wind_field_path);
  if (fin.is_open()) {
    std::string data_name;
    float data;
    // Read the line with the variable name.
    while (fin >> data_name) {
      // Save data on following line into the correct variable.
      if (data_name == "min_x:") {
        fin >> min_x_;
      } else if (data_name == "min_y:") {
        fin >> min_y_;
      } else if (data_name == "n_x:") {
        fin >> n_x_;
      } else if (data_name == "n_y:") {
        fin >> n_y_;
      } else if (data_name == "res_x:") {
        fin >> res_x_;
      } else if (data_name == "res_y:") {
        fin >> res_y_;
      } else if (data_name == "vertical_spacing_factors:") {
        while (fin >> data) {
          vertical_spacing_factors_.push_back(data);
          if (fin.peek() == '\n') break;
        }
      } else if (data_name == "bottom_z:") {
        while (fin >> data) {
          bottom_z_.push_back(data);
          if (fin.peek() == '\n') break;
        }
      } else if (data_name == "top_z:") {
        while (fin >> data) {
          top_z_.push_back(data);
          if (fin.peek() == '\n') break;
        }
      } else if (data_name == "u:") {
        while (fin >> data) {
          u_.push_back(data);
          if (fin.peek() == '\n') break;
        }
      } else if (data_name == "v:") {
        while (fin >> data) {
          v_.push_back(data);
          if (fin.peek() == '\n') break;
        }
      } else if (data_name == "w:") {
        while (fin >> data) {
          w_.push_back(data);
          if (fin.peek() == '\n') break;
        }
      } else {
        std::string restOfLine;
        getline(fin, restOfLine);
        gzerr << "[euler_aero_plugin] Invalid data name '" << data_name << restOfLine <<
              "' in custom wind field text file. Ignoring data on next line.\n";
        fin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
      }
    }
    fin.close();
    gzdbg << "[euler_aero_plugin] Successfully read custom wind field from text file.\n";
  } else {
    gzerr << "[euler_aero_plugin] Could not open custom wind field text file.\n";
  }
}

ignition::math::Vector3d EulerAeroPlugin::LinearInterpolation(
  double position, ignition::math::Vector3d* values, double* points) const {
  ignition::math::Vector3d value = values[0] + (values[1] - values[0]) /
                        (points[1] - points[0]) * (position - points[0]);
  return value;
}

ignition::math::Vector3d EulerAeroPlugin::BilinearInterpolation(
  double* position, ignition::math::Vector3d* values, double* points) const {
  ignition::math::Vector3d intermediate_values[2] = { LinearInterpolation(
                                             position[0], &(values[0]), &(points[0])),
                                           LinearInterpolation(
                                             position[0], &(values[2]), &(points[2])) };
  ignition::math::Vector3d value = LinearInterpolation(
                          position[1], intermediate_values, &(points[4]));
  return value;
}

ignition::math::Vector3d EulerAeroPlugin::TrilinearInterpolation(
  ignition::math::Vector3d link_position, ignition::math::Vector3d* values, double* points) const {
  double position[3] = {link_position.X(), link_position.Y(), link_position.Z()};
  ignition::math::Vector3d intermediate_values[4] = { LinearInterpolation(
                                             position[2], &(values[0]), &(points[0])),
                                           LinearInterpolation(
                                             position[2], &(values[2]), &(points[2])),
                                           LinearInterpolation(
                                             position[2], &(values[4]), &(points[4])),
                                           LinearInterpolation(
                                             position[2], &(values[6]), &(points[6])) };
  ignition::math::Vector3d value = BilinearInterpolation(
    &(position[0]), intermediate_values, &(points[8]));
  return value;
}

GZ_REGISTER_MODEL_PLUGIN(EulerAeroPlugin);

}
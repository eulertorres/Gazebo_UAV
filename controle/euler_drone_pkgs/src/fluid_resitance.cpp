#include "ros/callback_queue.h"
#include "ros/ros.h"
#include "ros/subscribe_options.h"
#include "geometry_msgs/Wrench.h" // Inclua para publicar forças
#include <functional>
#include <gazebo/common/common.hh>
#include <gazebo/gazebo.hh>
#include <gazebo/msgs/msgs.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/transport/transport.hh>
#include <ignition/math/Vector3.hh>
#include <thread>

namespace gazebo {
class FluidResitance : public ModelPlugin {
public:
  void Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf) {

    if (_sdf->HasElement("TopicName")) {
      this->TopicName =
          _sdf->Get<std::string>("TopicName");
    } else {
      ROS_WARN("No Topic Given name given, setting default name %s",
               this->TopicName.c_str());
    }

    if (_sdf->HasElement("LinkName")) {
      this->LinkName =
          _sdf->Get<std::string>("LinkName");
    } else {
      ROS_WARN("No LinkName Given name given, setting default "
               "name %s",
               this->LinkName.c_str());
    }

    if (_sdf->HasElement("uCoeff")) {
      this->fluid_resitance_index = _sdf->Get<double>("uCoeff");
    } else {
      ROS_WARN("Sem uCoeff Given, colocando valor aproximado do ar 1.0");
      this->fluid_resitance_index = 1.0; // Valor padrão, próximo ao ar na vida real
    }

    if (_sdf->HasElement("rate")) {
      this->rate = _sdf->Get<double>("rate");
    } else {
      ROS_WARN("No rate Given name given, setting default "
               "name %f",
               this->rate);
    }

    // Store the pointer to the model
    this->model = _parent;
    this->world = this->model->GetWorld();
    this->link_to_apply_resitance =
        this->model->GetLink(this->LinkName);

    // Listen to the update event. This event is broadcast every
    // simulation iteration.
    this->updateConnection = event::Events::ConnectWorldUpdateBegin(
        std::bind(&FluidResitance::OnUpdate, this));

    // Initialize ros, if it has not already bee initialized.
    if (!ros::isInitialized()) {
      int argc = 0;
      char **argv = NULL;
      ros::init(argc, argv, "model_mas_controler_rosnode",
                ros::init_options::NoSigintHandler);
    }

    // Create our ROS node. This acts in a similar manner to
    // the Gazebo node
    this->rosNode.reset(new ros::NodeHandle("model_mas_controler_rosnode"));

    // Cria um publisher para publicar as forças aplicadas
    this->force_pub = this->rosNode->advertise<geometry_msgs::Wrench>("/fluid_resitance_forces", 10);

#if (GAZEBO_MAJOR_VERSION >= 8)
    this->last_time = this->world->SimTime().Float();
#else
    this->last_time = this->world->GetSimTime().Float();
#endif

    // Freq
    ROS_WARN("Loaded FluidResitance Plugin with parent...%s, With Fluid "
             "Resitance = %f "
             "Started ",
             this->model->GetName().c_str(), this->fluid_resitance_index);
  }

  // Called by the world update start event
public:
  void OnUpdate() {
    float period = 1.0 / this->rate;

    // Get simulator time
#if (GAZEBO_MAJOR_VERSION >= 8)
    float current_time = this->world->SimTime().Float();
#else
    float current_time = this->world->GetSimTime().Float();
#endif
    float dt = current_time - this->last_time;

    if (dt <= period){
        ROS_DEBUG(">>>>>>>>>>TimePassed = %f, TimePeriod =%f ",dt, period);
        return;
    }else{
        this->last_time = current_time;
        this->ApplyResitance();
    }


  }

  void UpdateLinearVel() {
#if (GAZEBO_MAJOR_VERSION >= 8)
    this->now_lin_vel = this->model->RelativeLinearVel();
#else
    this->now_lin_vel = this->model->GetRelativeLinearVel();
#endif
  }

  void ApplyResitance() {

    this->UpdateLinearVel();

#if (GAZEBO_MAJOR_VERSION >= 8)
    ignition::math::Vector3d force, torque;
#else
    math::Vector3 force, torque;
#endif

    ROS_WARN("LinearSpeed = [%f,%f,%f] ",this->now_lin_vel.X(), this->now_lin_vel.Y(), this->now_lin_vel.Z());

    force.X() = -1.0 * this->fluid_resitance_index * this->now_lin_vel.X();
    force.Y() = -1.0 * this->fluid_resitance_index * this->now_lin_vel.Y();
    force.Z() = -1.0 * this->fluid_resitance_index * this->now_lin_vel.Z();

    // Changing the mass
    this->link_to_apply_resitance->AddRelativeForce(force);

    geometry_msgs::Wrench wrench_msg;
    wrench_msg.force.x = force.X();
    wrench_msg.force.y = force.Y();
    wrench_msg.force.z = force.Z();

    // Publish the forces applied
    this->force_pub.publish(wrench_msg);

    ROS_WARN("FluidResitanceApplying = [%f,%f,%f] ",force.X(), force.Y(), force.Z());
  }

private:
  void QueueThread() {
    static const double timeout = 0.01;
    while (this->rosNode->ok()) {
      this->rosQueue.callAvailable(ros::WallDuration(timeout));
    }
  }

  // Pointer to the model
private:
  physics::ModelPtr model;

  // Pointer to the update event connection
private:
  event::ConnectionPtr updateConnection;

  // Mas of model
  double fluid_resitance_index = 1.0;

  /// \brief A node use for ROS transport
private:
  std::unique_ptr<ros::NodeHandle> rosNode;

  /// \brief A ROS subscriber
private:
  ros::Subscriber rosSub;

  /// \brief A ROS publisher for the applied forces
private:
  ros::Publisher force_pub;

  /// \brief A ROS callbackqueue that helps process messages
private:
  ros::CallbackQueue rosQueue;
  /// \brief A thread the keeps running the rosQueue
private:
  std::thread rosQueueThread;

  /// \brief A ROS subscriber
private:
  physics::LinkPtr link_to_apply_resitance;

private:
  std::string TopicName = "fluid_resitance";

private:
  std::string LinkName = "base_link";

private:
#if (GAZEBO_MAJOR_VERSION >= 8)
  ignition::math::Vector3d now_lin_vel;
#else
  math::Vector3 now_lin_vel;
#endif

private:
  double rate = 1.0;

private:
  float last_time = 0.0;

private:
  /// \brief The parent World
  physics::WorldPtr world;
};

// Register this plugin with the simulator
GZ_REGISTER_MODEL_PLUGIN(FluidResitance)
} // namespace gazebo

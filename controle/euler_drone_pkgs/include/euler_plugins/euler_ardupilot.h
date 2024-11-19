/*
 * plugin do euleeeeeeeeeeeeeeeeeeeeeeeee RWXXXXXXXXX
 *
 */
#ifndef EULER_ARDUPILOT_HH_
#define EULER_ARDUPILOT_HH_

#include <sdf/sdf.hh>
#include <gazebo/common/common.hh>
#include <gazebo/physics/physics.hh>

// Optional: Include ROS header if needed
#include <ros/ros.h>

namespace gazebo
{
  // Forward declare private data class
  class ArduPilotSocketPrivate;
  class EulerArduPilotPluginPrivate;

  /// \brief Interface ArduPilot from ardupilot stack
  /// modeled after SITL/SIM_*
  ///
  /// The plugin requires the following parameters:
  /// <control>             control description block
  ///    <!-- inputs from Ardupilot -->
  ///    channel            attribute, ardupilot control channel
  ///    multiplier         command multiplier
  ///    <!-- output to Gazebo -->
  ///    type               type of control, VELOCITY, POSITION or EFFORT
  ///    <p_gain>           velocity pid p gain
  ///    <i_gain>           velocity pid i gain
  ///    <d_gain>           velocity pid d gain
  ///    <i_max>            velocity pid max integral correction
  ///    <i_min>            velocity pid min integral correction
  ///    <cmd_max>          velocity pid max command torque
  ///    <cmd_min>          velocity pid min command torque
  ///    <jointName>        motor joint, torque applied here
  ///    <turningDirection> rotor turning direction, 'cw' or 'ccw'
  ///    frequencyCutoff    filter incoming joint state
  ///    samplingRate       sampling rate for filtering incoming joint state
  ///    <rotorVelocitySlowdownSim> for rotor aliasing problem, experimental
  /// <imuName>     scoped name for the imu sensor
  /// <connectionTimeoutMaxCount> timeout before giving up on
  ///                             controller synchronization
  class GAZEBO_VISIBLE EulerArduPilotPlugin : public ModelPlugin
  {
    /// \brief Constructor.
    public: EulerArduPilotPlugin();

    /// \brief Destructor.
    public: ~EulerArduPilotPlugin();

    // Documentation Inherited.
    public: virtual void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf);

    /// \brief Update the control surfaces controllers.
    /// \param[in] _info Update information provided by the server.
    private: void OnUpdate();

    /// \brief Update PID Joint controllers.
    /// \param[in] _dt time step size since last update.
    private: void ApplyMotorForces(const double _dt);

    /// \brief Reset PID Joint controllers.
    private: void ResetPIDs();

    /// \brief Receive motor commands from ArduPilot
    private: void ReceiveMotorCommand();

    /// \brief Send state to ArduPilot
    private: void SendState() const;

    /// \brief Init ardupilot socket
    private: bool InitArduPilotSockets(sdf::ElementPtr _sdf) const;

    /// \brief Publish RC outputs at 40Hz
    private: void PublishRcOut();

    /// \brief Private data pointer.
    private: std::unique_ptr<EulerArduPilotPluginPrivate> dataPtr;

    /// \brief Transform from model orientation to x-forward and z-up
    private: ignition::math::Pose3d modelXYZToAirplaneXForwardZDown;

    /// \brief Transform from world frame to NED frame
    private: ignition::math::Pose3d gazeboXYZToNED;
  };
}
#endif

// euler_mot_plugin.cpp

#include "euler_plugins/euler_mot_plugin.h"

namespace gazebo {

GZ_REGISTER_MODEL_PLUGIN(EulerMotPlugin)

// Constructor and destructor
EulerMotPlugin::EulerMotPlugin()
    : ModelPlugin(),
      nh_(nullptr),
      pwm_value_(0),
      battery_voltage_(0.0),
      current_(0.0),
      torque_(0.0) {}

EulerMotPlugin::~EulerMotPlugin() {
    if (nh_) {
        nh_->shutdown();
        delete nh_;
    }
}

void EulerMotPlugin::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf) {
    // Store pointers to the model and world
    model_ = _model;
    world_ = model_->GetWorld();

    // Initialize ROS if it has not been initialized
    if (!ros::isInitialized()) {
        int argc = 0;
        char** argv = nullptr;
        ros::init(argc, argv, "gazebo_motor_plugin", ros::init_options::NoSigintHandler);
    }

    // ROS NodeHandle
    nh_ = new ros::NodeHandle();

    // Read parameters from SDF
    if (_sdf->HasElement("numero_motor"))
        motor_number_ = _sdf->Get<int>("numero_motor");
    else
        motor_number_ = 1;
    std::string motor_str = std::to_string(motor_number_);

    if (_sdf->HasElement("PWM_min"))
        pwm_min_ = _sdf->Get<double>("PWM_min");
    else
        pwm_min_ = 1100;

    if (_sdf->HasElement("PWM_max"))
        pwm_max_ = _sdf->Get<double>("PWM_max");
    else
        pwm_max_ = 1900;

    if (_sdf->HasElement("coeff"))
        torque_coeff_ = _sdf->Get<double>("coeff");
    else
        torque_coeff_ = 0.1;

    if (_sdf->HasElement("current_coeff"))
        current_coeff_ = _sdf->Get<double>("current_coeff");
    else
        current_coeff_ = 0.05;

    if (_sdf->HasElement("joint_name"))
        joint_name_ = _sdf->Get<std::string>("joint_name");
    else
        joint_name_ = "motor_joint";

    if (_sdf->HasElement("current_topic"))
        current_topic_ = _sdf->Get<std::string>("current_topic");
    else
        current_topic_ = "/battery" + motor_str;  // Custom topic name

    if (_sdf->HasElement("torque_topic"))
        torque_topic_ = _sdf->Get<std::string>("torque_topic");
    else
        torque_topic_ = "/motor" + motor_str + "/torque";  // Custom topic name

    if (_sdf->HasElement("rpm_topic"))
        rpm_topic_ = _sdf->Get<std::string>("rpm_topic");
    else
        rpm_topic_ = "/motor" + motor_str + "/rpm";  // Custom topic name

    if (_sdf->HasElement("forceUpdateRate"))
        force_update_rate_ = _sdf->Get<double>("forceUpdateRate");
    else
        force_update_rate_ = 0.1;

    if (_sdf->HasElement("orientacao"))
        orientation_ = _sdf->Get<std::string>("orientacao");
    else
        orientation_ = "CCW";  // Default is CCW

    if (_sdf->HasElement("coef_drag"))  // For debugging
        drag_coeff_ = _sdf->Get<double>("coef_drag");
    else
        drag_coeff_ = 0.05;

    if (_sdf->HasElement("back_emf"))  // For debugging
        back_emf_coeff_ = _sdf->Get<double>("back_emf");
    else
        back_emf_coeff_ = 0.005;

    pwm_timeout_ = 1.0;

    // Get the joint pointer
    joint_ = model_->GetJoint(joint_name_);
    if (!joint_) {
        gzerr << "[euler_mot_plugin] Could not find joint: " << joint_name_ << "\n";
        return;
    }

    // Publisher for current and torque
    current_pub_ = nh_->advertise<std_msgs::Float32>(current_topic_, 1);
    torque_pub_ = nh_->advertise<std_msgs::Float32>(torque_topic_, 1);
    rpm_pub_ = nh_->advertise<std_msgs::Float32>(rpm_topic_, 1);  // Publisher for RPM

    pwm_value_ = 1100;
    // Subscribe to the PWM and battery voltage topics
    pwm_sub_ = nh_->subscribe("/mavros/rc/out", 1, &EulerMotPlugin::PwmCallback, this);
    voltage_sub_ = nh_->subscribe("/battery/voltage", 1, &EulerMotPlugin::VoltageCallback, this);

    // Connect to the Gazebo update event
    update_connection_ = event::Events::ConnectWorldUpdateBegin(
        std::bind(&EulerMotPlugin::OnUpdate, this, std::placeholders::_1));

    std::cout << "[euler_mot_plugin] Motor plugin loaded successfully.\n";
    std::cout.flush();
    last_pwm_received_time_ = world_->SimTime();  // Initialize last PWM received time
    last_update_time_ = world_->SimTime();
}

void EulerMotPlugin::OnUpdate(const common::UpdateInfo& _info) {
    common::Time current_time = world_->SimTime();
    double dt = (current_time - last_update_time_).Double();

    // Check if PWM timeout has occurred
    if ((current_time - last_pwm_received_time_).Double() > pwm_timeout_) {
        pwm_value_ = 0;  // Set PWM to 0 if no updates
    }

    // Calculate the duty cycle based on the PWM value
    double duty_cycle = (pwm_value_ - pwm_min_) / (pwm_max_ - pwm_min_);
    if (duty_cycle < 0) duty_cycle = 0;
    if (duty_cycle > 1) duty_cycle = 1;

    // Get the current angular velocity of the joint
    double angular_velocity = std::abs(joint_->GetVelocity(0));  // Assuming index 0 for revolute joint

    if (angular_velocity < 0.01) angular_velocity = 0;

    // Calculate RPM based on the angular velocity
    rpm = (angular_velocity / (2 * M_PI)) * 60;

    // Calculate resistive torque based on angular velocity
    double torque_resistivo = drag_coeff_ * angular_velocity * angular_velocity;

    // Calculate main torque based on duty cycle and battery voltage
    double torque_motor = torque_coeff_ * battery_voltage_ * duty_cycle;

    // Calculate total torque
    torque_ = torque_motor - torque_resistivo;

    // Calculate the current based on the torque
    current_ = torque_ * current_coeff_;

    // If torque is negative, subtract braking torque (simulating kinetic energy loss)
    if (torque_ < 0) {
        double torque_back = 0.0;
        if (angular_velocity > 0) {
            // Calculate "back EMF" proportional to angular velocity
            torque_back = back_emf_coeff_ * angular_velocity * angular_velocity;
        }
        torque_ -= torque_back;  // Add resistance from reverse current
        current_ = current_ * 0.7;  // Part of the current is converted to heat
    }

    rpm = (angular_velocity / (2 * M_PI)) * 60;

    // Apply torque to the joint
    if (orientation_ == "CW") {
        // Apply torque in the opposite direction
        joint_->SetForce(0, -torque_);
    } else {
        // Apply torque in the positive direction
        joint_->SetForce(0, torque_);
    }

    // Publish current and torque at 10 Hz
    if (dt >= 0.1) {
        PublishCurrent();
        PublishTorque();
        PublishRPM();
        last_update_time_ = world_->SimTime();
    }
}

// Publish the calculated current
void EulerMotPlugin::PublishCurrent() {
    std_msgs::Float32 current_msg;
    current_msg.data = current_;
    current_pub_.publish(current_msg);
}

// Publish the calculated torque
void EulerMotPlugin::PublishTorque() {
    std_msgs::Float32 torque_msg;
    torque_msg.data = torque_;
    torque_pub_.publish(torque_msg);
}

// Publish the calculated RPM
void EulerMotPlugin::PublishRPM() {
    std_msgs::Float32 rpm_msg;
    rpm_msg.data = rpm;
    rpm_pub_.publish(rpm_msg);
}

void EulerMotPlugin::PwmCallback(const mavros_msgs::RCOut::ConstPtr& msg) {
    if (motor_number_ > 0 && motor_number_ <= msg->channels.size()) {
        pwm_value_ = msg->channels[motor_number_ - 1];
        last_pwm_received_time_ = world_->SimTime();  // Update last received time
    }
}

void EulerMotPlugin::VoltageCallback(const std_msgs::Float32::ConstPtr& msg) {
    battery_voltage_ = msg->data;
}

}  // namespace gazebo

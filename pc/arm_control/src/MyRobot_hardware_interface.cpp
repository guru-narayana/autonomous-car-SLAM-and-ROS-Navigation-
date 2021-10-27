#include <arm_control/MyRobot_hardware_interface.h>
MyRobot::MyRobot(ros::NodeHandle& nh) : nh_(nh) {
    init();
    controller_manager_.reset(new controller_manager::ControllerManager(this, nh_));
    loop_hz_=30;
    ros::Duration update_freq = ros::Duration(1.0/loop_hz_);
    ros::Subscriber sub = nh_.subscribe("Servo_joint_states", 1000, &MyRobot::ServoCallback,this);
    my_control_loop_ = nh_.createTimer(update_freq, &MyRobot::update, this);
}
MyRobot::~MyRobot() {
}
void MyRobot::init() {

    hardware_interface::JointStateHandle jointStateHandle1("j1", &joint_position_[0], &joint_velocity_[0], &joint_effort_[0]);
    joint_state_interface_.registerHandle(jointStateHandle1);
    hardware_interface::JointHandle jointPositionHandle1(jointStateHandle1, &joint_position_command_[0]);
    position_joint_interface_.registerHandle(jointPositionHandle1);
    joint_limits_interface::getJointLimits("j1", nh_, limits);
    joint_limits_interface::PositionJointSaturationHandle jointLimitsHandle1(jointPositionHandle1, limits);
    positionJointSaturationInterface.registerHandle(jointLimitsHandle1);

    hardware_interface::JointStateHandle jointStateHandle2("j2", &joint_position_[1], &joint_velocity_[1], &joint_effort_[1]);
    joint_state_interface_.registerHandle(jointStateHandle2);
    hardware_interface::JointHandle jointPositionHandle2(jointStateHandle2, &joint_position_command_[1]);
    position_joint_interface_.registerHandle(jointPositionHandle2);
    joint_limits_interface::getJointLimits("j2", nh_, limits);
    joint_limits_interface::PositionJointSaturationHandle jointLimitsHandle2(jointPositionHandle2, limits);
    positionJointSaturationInterface.registerHandle(jointLimitsHandle2);

    hardware_interface::JointStateHandle jointStateHandle3("j3", &joint_position_[2], &joint_velocity_[2], &joint_effort_[2]);
    joint_state_interface_.registerHandle(jointStateHandle3);
    hardware_interface::JointHandle jointPositionHandle3(jointStateHandle3, &joint_position_command_[2]);
    position_joint_interface_.registerHandle(jointPositionHandle3);
    joint_limits_interface::getJointLimits("j3", nh_, limits);
    joint_limits_interface::PositionJointSaturationHandle jointLimitsHandle3(jointPositionHandle3, limits);
    positionJointSaturationInterface.registerHandle(jointLimitsHandle3);

    hardware_interface::JointStateHandle jointStateHandle4("j4", &joint_position_[3], &joint_velocity_[3], &joint_effort_[3]);
    joint_state_interface_.registerHandle(jointStateHandle4);
    hardware_interface::JointHandle jointPositionHandle4(jointStateHandle4, &joint_position_command_[3]);
    position_joint_interface_.registerHandle(jointPositionHandle4);
    joint_limits_interface::getJointLimits("j2", nh_, limits);
    joint_limits_interface::PositionJointSaturationHandle jointLimitsHandle4(jointPositionHandle4, limits);
    positionJointSaturationInterface.registerHandle(jointLimitsHandle4);

    registerInterface(&joint_state_interface_);
    registerInterface(&position_joint_interface_);
    registerInterface(&positionJointSaturationInterface);    
}
void MyRobot::update(const ros::TimerEvent& e) {
    elapsed_time_ = ros::Duration(e.current_real - e.last_real);
    read();
    controller_manager_->update(ros::Time::now(), elapsed_time_);
    write(elapsed_time_);
}
void MyRobot::ServoCallback(const sensor_msgs::JointState::ConstPtr& msg) {
  joint_position_[0] = msg->position[0];
  joint_position_[1] = msg->position[1];
  joint_position_[2] = msg->position[2];
  joint_position_[3] = msg->position[3];
}
void MyRobot::read(){
}
void MyRobot::write(ros::Duration elapsed_time){
  positionJointSaturationInterface.enforceLimits(elapsed_time);
  ros::Publisher pub_servo = nh_.advertise<sensor_msgs::JointState>("robot_simulation/joint_states", 10);
  sensor_msgs::JointState joint_state;
  joint_state.name[0] = "j1";
  joint_state.name[1] = "j2";
  joint_state.name[2] = "j3";
  joint_state.name[3] = "j4";
  joint_state.position[0] = joint_position_command_[0];
  joint_state.position[1] = joint_position_command_[1];
  joint_state.position[2] = joint_position_command_[2];
  joint_state.position[3] = joint_position_command_[3];
  pub_servo.publish(joint_state);  
}
int main(int argc, char** argv)
{
    ros::init(argc, argv, "MyRobot_hardware_inerface_node");
    ros::NodeHandle nh;
    ros::AsyncSpinner spinner(2); 
    MyRobot ROBOT(nh);
    spinner.start();
    ros::waitForShutdown();
    return 0;
}
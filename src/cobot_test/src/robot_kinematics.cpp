#include <rclcpp/rclcpp.hpp>
#include <moveit_core/moveit/planning/robot_model_loader/include/moveit/robot_model_loader/robot_model_loader.hpp>
#include <moveit_core/moveit/robot_model/robot_model.hpp>
#include <moveit_core/moveit/robot_state/robot_state.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>

class RobotLoaderWithKinematics : public rclcpp::Node
{
public:
  RobotLoaderWithKinematics() : Node("robot_kinematics_node")
    {   
      // Subscribe to servo feedback (FK input)
      servo_feedback_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
        "/servo_feedback", 10, 
        std::bind(&RobotLoaderWithKinematics::servoFeedbackCallback, this, std::placeholders::_1));

      // Subscribe to desired pose (IK input)
      target_pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
          "/target_pose", 10, 
          std::bind(&RobotLoaderWithKinematics::targetPoseCallback, this, std::placeholders::_1));

      // Publisher for computed joint angles (IK output)
      joint_state_pub_ = this->create_publisher<sensor_msgs::msg::JointState>("/joint_state", 10);

      // Use a timer to delay the initialization
      this->timer_ = this->create_wall_timer(
        std::chrono::seconds(1),
        std::bind(&RobotLoaderWithKinematics::initializeKinematics, this)
      );
    }

    ~RobotLoaderWithKinematics() // Destructor
    {
        // Perform cleanup here
        if (this->timer_)
        {
            this->timer_->cancel();
        }
        this->robot_state.reset();
        this->robot_model.reset();
        RCLCPP_INFO(this->get_logger(), "RobotLoaderWithKinematics cleaned up.");
    }

private:
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr servo_feedback_sub_;
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_pub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr target_pose_sub_;

    moveit::core::RobotModelPtr robot_model;
    moveit::core::RobotStatePtr robot_state;
    const moveit::core::JointModelGroup *joint_model_group;
    Eigen::Isometry3d end_effector_state;
    rclcpp::TimerBase::SharedPtr timer_;
    std::vector<std::string> joint_names;

    void initializeKinematics()
    {
        // Initialize the kinematic model and state
        this->robot_model = getKinematicModel();
        if (!robot_model){
            RCLCPP_ERROR(this->get_logger(), "Failed to load the robot model.");
            return;
        }
        this->robot_state = std::make_shared<moveit::core::RobotState>(robot_model);
        this->robot_state->setToDefaultValues();
        this->joint_model_group = robot_model->getJointModelGroup("arm");
        if (!joint_model_group){
            RCLCPP_ERROR(this->get_logger(), "Failed to get joint model group.");
        }

        joint_names = joint_model_group->getVariableNames();

        RCLCPP_INFO(this->get_logger(), "Kinematic model and state initialized.");
        
        // Cancel the timer after initialization
        this->timer_->cancel();
    }

    moveit::core::RobotModelPtr getKinematicModel(std::string robot_description_topic="robot_description"){
      robot_model_loader::RobotModelLoader robot_model_loader(this->shared_from_this(), robot_description_topic);
      const moveit::core::RobotModelPtr& kinematic_model = robot_model_loader.getModel();
      RCLCPP_INFO(this->get_logger(), "Model frame: %s", kinematic_model->getModelFrame().c_str());
      return kinematic_model;
    }
    
    // std::vector<double> getJointGroupValues(auto& robot_state, const moveit::core::JointModelGroup* joint_model_group){
    //   const std::vector<std::string>& joint_names = joint_model_group->getVariableNames();

    //   // Get Joint Values
    //   std::vector<double> joint_values;
    //   robot_state->copyJointGroupPositions(joint_model_group, joint_values);
    //   for (std::size_t i = 0; i < joint_names.size(); ++i)
    //   {
    //     RCLCPP_INFO(this->get_logger(), "Joint %s: %f", joint_names[i].c_str(), joint_values[i]);
    //   }
    // }

    void publishJointAngles(const std::vector<double> &joint_values)
    {
      // Convert each element from double to float
      std_msgs::msg::Float32MultiArray joint_angles_msg;

      sensor_msgs::msg::JointState joint_state_msg;
      joint_state_msg.header.stamp = this->now();
      joint_state_msg.name = joint_names; // Example joint names
      for (const auto &value : joint_values){
        //joint_angles_msg.data.push_back(static_cast<float>(value)); 
        joint_state_msg.position.push_back(static_cast<double>(value));
   
      }

      joint_state_pub_->publish(joint_state_msg);

      RCLCPP_INFO(this->get_logger(), "Finished Publishing joint states");
  } 


    void inverseKinematics(const Eigen::Isometry3d &target_pose, double timeout=0.5){
      if (!robot_state || !joint_model_group){
          RCLCPP_ERROR(this->get_logger(), "Robot state or joint model group is not initialized.");
          return;
      }

      bool found_ik = robot_state->setFromIK(joint_model_group, target_pose, timeout);

      if (found_ik){
        end_effector_state = target_pose;
        std::vector<double> joint_values;
        robot_state->copyJointGroupPositions(joint_model_group, joint_values);
        RCLCPP_INFO(this->get_logger(), "Performed IK");
        handleJointLimits(joint_values);
        RCLCPP_INFO(this->get_logger(), "Handled Joint Limits");
        publishJointAngles(joint_values);

        // for (std::size_t i = 0; i < joint_values.size(); ++i){
        //   RCLCPP_INFO(this->get_logger(), "After IK Joint" + joint_names[i].c_str() + " " + joint_values[i]);
        // }
      }
      else{
        RCLCPP_INFO(this->get_logger(), "Did not find IK solution");
      }
    }

    void forwardKinematics(){
      if (!robot_state || !joint_model_group)
      {
          RCLCPP_ERROR(this->get_logger(), "Robot state or joint model group is not initialized.");
          return;
      }

      robot_state->setToRandomPositions(joint_model_group);
      const Eigen::Isometry3d &end_effector_state = robot_state->getGlobalLinkTransform(joint_model_group->getLinkModelNames().back());

      RCLCPP_INFO_STREAM(this->get_logger(), "Translation: \n" << end_effector_state.translation() << "\n");
      RCLCPP_INFO_STREAM(this->get_logger(), "Rotation: \n" << end_effector_state.rotation() << "\n");
    }

    void handleJointLimits(std::vector<double> &joint_values){
      for (size_t i = 0; i < joint_values.size(); i++){
          const moveit::core::JointModel *joint_model = joint_model_group->getActiveJointModels()[i];
          const moveit::core::JointModel::Bounds &bounds = joint_model->getVariableBounds();
          joint_values[i] = std::max(bounds[0].min_position_, std::min(joint_values[i], bounds[0].max_position_));
      }
      /* Check whether any joint is outside its joint limits */
      RCLCPP_INFO_STREAM(this->get_logger(), "Current state is " << (robot_state->satisfiesBounds() ? "valid" : "not valid"));
      /* Enforce the joint limits for this state and check again*/
      robot_state->enforceBounds();
      RCLCPP_INFO_STREAM(this->get_logger(), "Current state is " << (robot_state->satisfiesBounds() ? "valid" : "not valid"));
    }

    Eigen::MatrixXd getRobotJacobian(){
      Eigen::Vector3d reference_point_position(0.0, 0.0, 0.0);
      Eigen::MatrixXd jacobian;
      robot_state->getJacobian(joint_model_group, robot_state->getLinkModel(joint_model_group->getLinkModelNames().back()),
                              reference_point_position, jacobian);
      RCLCPP_INFO_STREAM(this->get_logger(), "Jacobian: \n" << jacobian << "\n");
      return jacobian;
    }
    
  
    void servoFeedbackCallback(const sensor_msgs::msg::JointState::SharedPtr msg){
      RCLCPP_INFO(this->get_logger(), "ServoFeedbackCallback fct called");

      if (msg->position.empty()) return;

      robot_state->setJointGroupPositions(joint_model_group, msg->position);
      end_effector_state = robot_state->getGlobalLinkTransform(joint_model_group->getLinkModelNames().back());
      inverseKinematics(end_effector_state);
    }

     // **IK: Compute joint angles from desired end-effector pose**
     void targetPoseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
     {
        RCLCPP_INFO(this->get_logger(), "TargetPoseCallback fct called");
        Eigen::Isometry3d target_pose;
        target_pose.translation() = Eigen::Vector3d(msg->pose.position.x, msg->pose.position.y, msg->pose.position.z);
        target_pose.linear() = Eigen::Quaterniond(msg->pose.orientation.w, msg->pose.orientation.x, msg->pose.orientation.y, msg->pose.orientation.z).toRotationMatrix();

        inverseKinematics(target_pose);
      } 
    
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);

  // rclcpp::NodeOptions node_options;
  // node_options.automatically_declare_parameters_from_overrides(true);
  //auto node = rclcpp::Node::make_shared("robot_kinematics_node", node_options);

  auto node = std::make_shared<RobotLoaderWithKinematics>();

  rclcpp::spin(node);

  rclcpp::shutdown();

  return 0;
}
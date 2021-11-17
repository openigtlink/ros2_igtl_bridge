#include <memory>
#include <iostream>
#include <iomanip>

#include "rclcpp/rclcpp.hpp"

#include "geometry_msgs/msg/point.h"
#include "std_msgs/msg/string.h"

#include <moveit_msgs/srv/get_position_fk.hpp>
#include <moveit_msgs/srv/get_position_ik.hpp>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_model_loader/robot_model_loader.h>

#include <moveit/robot_state/robot_state.h>
#include <moveit/planning_interface/planning_interface.h>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/action/execute_trajectory.h>

#include <rclcpp_action/rclcpp_action.hpp>
#include <moveit_msgs/action/execute_trajectory.h>

#include <tf2/transform_datatypes.h>
#include <tf2/convert.h>
#include <ros2_igtl_bridge/msg/point.hpp>
#include <chrono>
using namespace std::chrono_literals;

static const rclcpp::Logger LOGGER = rclcpp::get_logger("spine_control");

bool points_received = false;

class igtl_subscriber : public rclcpp::Node {

public:

  geometry_msgs::msg::Point entry;
  geometry_msgs::msg::Point target;

private:
  
  rclcpp::Subscription<ros2_igtl_bridge::msg::Point>::SharedPtr sub_igtl_point;
  
  bool valid_entry;
  bool valid_target;

public:

  igtl_subscriber( ) :
    Node("igtl_subscriber"),
    valid_entry(false),
    valid_target(false){
    
    sub_igtl_point = this->create_subscription<ros2_igtl_bridge::msg::Point>
      ("/IGTL_POINT_IN", 10,
       std::bind(&igtl_subscriber::igtl_point_cb, this,std::placeholders::_1));
  }

  void igtl_point_cb( const ros2_igtl_bridge::msg::Point::SharedPtr point ){
    if(point->name.compare("Entry") == 0){
      valid_entry = true;
      // -603.73 401.57 150.40
      entry.x = -point->pointdata.x / 1000.0; // Convert mm to m
      entry.y = -point->pointdata.y / 1000.0; // Convert mm to m
      entry.z = point->pointdata.z / 1000.0; // Convert mm to m      
      RCLCPP_INFO(LOGGER, "[spine_control] Entry point has been received: \n");
    } else if(point->name.compare("Target") == 0){
      // -608.74 393.21 135.82
      valid_target = true;
      target.x = -point->pointdata.x / 1000.0; // Convert mm to m
      target.y = -point->pointdata.y / 1000.0; // Convert mm to m
      target.z = point->pointdata.z / 1000.0; // Convert mm to m      
      RCLCPP_INFO(LOGGER, "[spine_control] Target point has been received: \n");
    }
    if( valid_entry && valid_target ){
      points_received = true;
    }
  }
  
};

int main( int argc, char** argv ){

  rclcpp::init(argc, argv);

  auto sub_node = std::make_shared<igtl_subscriber>();

  rclcpp::NodeOptions node_options;
  node_options.automatically_declare_parameters_from_overrides(true);
  auto node = rclcpp::Node::make_shared("spine_plan", node_options);
  
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  executor.add_node(sub_node);
  std::thread([&executor]() { executor.spin(); }).detach();

  // move group
  moveit::planning_interface::MoveGroupInterface move_group(node, "ur_manipulator");
  moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

  while (!points_received && rclcpp::ok()){
      std::cout << "Waiting request" << std::endl;
    std::this_thread::sleep_for(std::chrono::milliseconds(1000));
  }

  // try to get the state
  moveit::core::RobotStatePtr current_state;
  current_state=move_group.getCurrentState();
  current_state->printStatePositions(std::cout);
  
  std::cout << "Reveived" << std::endl;

  sensor_msgs::msg::JointState seed;
  seed.name.push_back( "shoulder_pan_joint" );
  seed.name.push_back( "shoulder_lift_joint" );
  seed.name.push_back( "elbow_joint" );
  seed.name.push_back( "wrist_1_joint" );
  seed.name.push_back( "wrist_2_joint" );
  seed.name.push_back( "wrist_3_joint" );
  seed.position.push_back( 174.65*M_PI/180.0 );
  seed.position.push_back(-101.60*M_PI/180.0 );
  seed.position.push_back(-134.21*M_PI/180.0 );
  seed.position.push_back(  -8.01*M_PI/180.0 );
  seed.position.push_back(  29.84*M_PI/180.0 );
  seed.position.push_back( -52.88*M_PI/180.0 );
  
  // FK
  auto fk_node = rclcpp::Node::make_shared("fk_node");
  rclcpp::Client<moveit_msgs::srv::GetPositionFK>::SharedPtr client_fk =
    fk_node->create_client<moveit_msgs::srv::GetPositionFK>("compute_fk");
  auto fk_request=std::make_shared<moveit_msgs::srv::GetPositionFK::Request>();
  fk_request->header.frame_id = "base_link";
  fk_request->fk_link_names.push_back("tip");
  fk_request->robot_state.joint_state = seed;

  auto fk_result = client_fk->async_send_request(fk_request);
  if (rclcpp::spin_until_future_complete(fk_node, fk_result) == rclcpp::FutureReturnCode::SUCCESS){
    std::cout << "FK result: " << fk_result.get()->error_code.val << std::endl;
  }
  else {
    RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Failed to call service add_two_ints");
  }
  
  std::string PLANNING_GROUP = "ur_manipulator";
  auto ik_request=std::make_shared<moveit_msgs::srv::GetPositionIK::Request>();
  ik_request->ik_request.group_name = PLANNING_GROUP;
  ik_request->ik_request.robot_state.joint_state = seed;
  ik_request->ik_request.robot_state.is_diff = false;
  ik_request->ik_request.avoid_collisions = true;
  ik_request->ik_request.pose_stamped = fk_result.get()->pose_stamped[0];
  ik_request->ik_request.pose_stamped.pose.position = sub_node->entry;
  ik_request->ik_request.timeout.sec = 5;

  // Get the inverse kin  
  auto ik_node = rclcpp::Node::make_shared("ik_node");
  rclcpp::Client<moveit_msgs::srv::GetPositionIK>::SharedPtr client_ik =
    ik_node->create_client<moveit_msgs::srv::GetPositionIK>("compute_ik");
  
  auto result = client_ik->async_send_request(ik_request);
  if (rclcpp::spin_until_future_complete(ik_node, result) == rclcpp::FutureReturnCode::SUCCESS){
    std::cout << "IK result: " << result.get()->error_code.val << std::endl;
    sensor_msgs::msg::JointState goal = result.get()->solution.joint_state;
    for( std::size_t i=0; i<goal.position.size() ; i++)
      { std::cout << std::setw(13) << goal.position[i]; }
    std::cout << std::endl;

    move_group.setStartState(*current_state);
    move_group.setJointValueTarget(goal);
    moveit::planning_interface::MoveGroupInterface::Plan plan;
    moveit::planning_interface::MoveItErrorCode code=move_group.plan(plan);
    if( (bool)code ){
      std::cout << "Moving to entry" << std::endl;
      move_group.execute(plan);
      std::cout << "Entry point reached" << std::endl;

      current_state=move_group.getCurrentState();
      current_state->printStatePositions(std::cout);

      moveit::core::RobotState start_state(*move_group.getCurrentState());
      move_group.setStartState(start_state);
      
      geometry_msgs::msg::Pose current_pose = fk_result.get()->pose_stamped[0].pose;
      geometry_msgs::msg::Pose desired_pose = fk_result.get()->pose_stamped[0].pose;
      desired_pose.position = sub_node->target;

      std::cout << std::setw(13) << current_pose.position.x
		<< std::setw(13) << current_pose.position.y
		<< std::setw(13) << current_pose.position.z << std::endl;
      
      std::cout << std::setw(13) << current_pose.position.x
		<< std::setw(13) << current_pose.position.y
		<< std::setw(13) << current_pose.position.z << std::endl;
      
      std::vector<geometry_msgs::msg::Pose> waypoints;
      waypoints.push_back(current_pose);
      waypoints.push_back(desired_pose);
      
      move_group.clearPathConstraints();
      const double jump_threshold = 0;
      const double eef_step = 0.001;
      moveit_msgs::msg::RobotTrajectory trajectory;
      double fraction = move_group.computeCartesianPath(waypoints,
							eef_step,
							jump_threshold,
							trajectory,
							false,
							&code);
      // disable collision for this one because the model is a little bit off.
      
      std::cout << "fraction " << fraction << " " << code.val << std::endl;
      for( std::size_t i=0; i<trajectory.joint_trajectory.points.size(); i++ ){
	for( std::size_t j=0; j<trajectory.joint_trajectory.points[i].positions.size(); j++){
	  std::cout << std::setw(13) << trajectory.joint_trajectory.points[i].positions[j];
	}
	std::cout << std::endl;
	trajectory.joint_trajectory.points[i].time_from_start.sec *= 4;
      }
      
      auto action_node = rclcpp::Node::make_shared("action_node");
      auto action_client =
	rclcpp_action::create_client<moveit_msgs::action::ExecuteTrajectory>
	(action_node, "/execute_trajectory");
      action_client->wait_for_action_server(std::chrono::seconds(2));

      moveit_msgs::action::ExecuteTrajectory_Goal action_goal;
      action_goal.trajectory.joint_trajectory = trajectory.joint_trajectory;
      auto goal_handle_future = action_client->async_send_goal(action_goal);
      if (rclcpp::spin_until_future_complete(action_node, goal_handle_future) ==
	  rclcpp::FutureReturnCode::SUCCESS){
	std::cout << "GOOD JOB!" << std::endl;
      }
      
    }
    
  }
  else {
    RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Failed to call service add_two_ints");
  }

  std::cout << "shutdown" << std::endl;
  rclcpp::shutdown();
  
  return 0;
}

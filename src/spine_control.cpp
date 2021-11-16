#include <memory>
#include <iostream>
#include <iomanip>

#include "rclcpp/rclcpp.hpp"

#include "geometry_msgs/msg/point.h"
#include "std_msgs/msg/string.h"

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

class spine_control : public rclcpp::Node {

private:

  geometry_msgs::msg::Point entry;
  geometry_msgs::msg::Point target;

  rclcpp::Subscription<ros2_igtl_bridge::msg::Point>::SharedPtr sub_igtl_point;
  
  bool valid_entry;
  bool valid_target;

  moveit::planning_interface::MoveGroupInterface* move_group;
  moveit::core::RobotStatePtr current_state;  
public:

  spine_control( ) :
    Node("spine_control"),
    valid_entry(false),
    valid_target(false){
    
    sub_igtl_point = this->create_subscription<ros2_igtl_bridge::msg::Point>
      ("/IGTL_POINT_IN", 10,
       std::bind(&spine_control::igtl_point_cb, this,std::placeholders::_1));

    //this->declare_parameter<std::string>("robot_description", "default");
    //this->declare_parameter<std::string>("robot_description_semantic", "default");
    //rclcpp::SyncParametersClient::SharedPtr param_client = std::make_shared<rclcpp::SyncParametersClient>(this, "move_group");
    //while(!param_client->wait_for_service(1s)){}
    //std::string robot_description = param_client->get_parameter<std::string>("robot_description");
    //std::string robot_semantic = param_client->get_parameter<std::string>("robot_description_semantic");
    //std::vector<rclcpp::Parameter> params;
    //params.push_back( rclcpp::Parameter("robot_description", robot_description ) );
    //params.push_back( rclcpp::Parameter("robot_description_semantic", robot_semantic ) );
    //this->set_parameters(params);
  }

  void create_mg(){
    // cannot use shared_from_this() in constructor :(
    move_group = new moveit::planning_interface::MoveGroupInterface
      (shared_from_this(), "ur_manipulator");

    current_state=move_group->getCurrentState();
    current_state->printStatePositions(std::cout);
  }
  
  void igtl_point_cb( const ros2_igtl_bridge::msg::Point::SharedPtr point ){
    if(point->name.compare("Entry") == 0){
      valid_entry = true;
      // -603.73 401.57 150.40
      entry.x = -603.73/1000; //point->pointdata.x / 1000.0; // Convert mm to m
      entry.y =  401.57/1000; //point->pointdata.y / 1000.0; // Convert mm to m
      entry.z =  150.40/1000; //point->pointdata.z / 1000.0; // Convert mm to m      
      RCLCPP_INFO(LOGGER, "[spine_control] Entry point has been received: \n");
    } else if(point->name.compare("Target") == 0){
      // -608.74 393.21 135.82
      valid_target = true;
      target.x = -608.74/1000; //point->pointdata.x / 1000.0; // Convert mm to m
      target.y =  393.21/1000; //point->pointdata.y / 1000.0; // Convert mm to m
      target.z =  135.82/1000; //point->pointdata.z / 1000.0; // Convert mm to m      
      RCLCPP_INFO(LOGGER, "[spine_control] Target point has been received: \n");
    }
    if( valid_entry && valid_target ){
      execute();
    }
  }

  void execute(){
    std::string PLANNING_GROUP = "ur_manipulator";

    tf2::Vector3 p1( entry.x, entry.y, entry.z ); 
    tf2::Vector3 p2( target.x, target.y, target.z ); 
    tf2::Vector3 a = p2-p1;
    a = a / a.length();

    tf2::Vector3 o( 0.0, 1.0, 0.0 );
    tf2::Vector3 n = o.cross(a);
    /*
    tf2::Matrix3x3 R( n.getX(), o.getX(), a.getX(),
		      n.getY(), o.getY(), a.getY(),
		      n.getZ(), o.getZ(), a.getZ() );
    */
    tf2::Matrix3x3 R( -0.9466,   -0.1493,   -0.2858,
		      0.0039,    0.8810,   -0.4730,
		      0.3224,   -0.4489,   -0.8334 );
    /*
    std::cout << std::setw(13) << n.getX() << std::setw(13) << o.getX() << std::setw(13) << a.getX() << std::endl;
    std::cout << std::setw(13) << n.getY() << std::setw(13) << o.getY() << std::setw(13) << a.getY() << std::endl;
    std::cout << std::setw(13) << n.getZ() << std::setw(13) << o.getZ() << std::setw(13) << a.getZ() << std::endl<< std::endl;;
    std::cout << std::setw(13) << p1.getX() << std::setw(13) << p1.getY() << std::setw(13) << p1.getZ() << std::endl;
    */
    
    geometry_msgs::msg::PoseStamped pose;
    tf2::Transform tfpose(R, p1);
    pose.pose.position.x = tfpose.getOrigin().getX();
    pose.pose.position.y = tfpose.getOrigin().getY();
    pose.pose.position.z = tfpose.getOrigin().getZ();
    pose.pose.orientation.x = tfpose.getRotation().getX();
    pose.pose.orientation.y = tfpose.getRotation().getY();
    pose.pose.orientation.z = tfpose.getRotation().getZ();
    pose.pose.orientation.w = tfpose.getRotation().getW();
    
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
    seed.position.push_back(  -7.01*M_PI/180.0 );
    seed.position.push_back(  30.34*M_PI/180.0 );
    seed.position.push_back( -54.57*M_PI/180.0 );

    auto request=std::make_shared<moveit_msgs::srv::GetPositionIK::Request>();
    request->ik_request.group_name = PLANNING_GROUP;
    request->ik_request.robot_state.joint_state = seed;
    request->ik_request.robot_state.is_diff = false;
    request->ik_request.avoid_collisions = true;
    request->ik_request.pose_stamped = pose;
    //request->ik_request.timeout = ros::Duration(5);

    /*
    rclcpp::Client<moveit_msgs::srv::GetPositionIK>::SharedPtr client =
      this->create_client<moveit_msgs::srv::GetPositionIK>("compute_ik");
    auto result = client->async_send_request(request);
    std::shared_ptr<rclcpp::Node> tmp = rclcpp::Node::make_shared("tmp");
    if(rclcpp::spin_until_future_complete(tmp, result) ==
     rclcpp::FutureReturnCode::SUCCESS){
    */
    move_group->setStartState(*current_state);
    //move_group->setJointValueTarget(result.get()->solution.joint_state);
    move_group->setJointValueTarget(seed);
    moveit::planning_interface::MoveGroupInterface::Plan plan;
    moveit::planning_interface::MoveItErrorCode code=move_group->plan(plan);
    if( (bool)code ){
      std::cout << "Moving to entry" << std::endl;
      move_group->execute(plan);
      std::cout << "Entry point reached" << std::endl;
      geometry_msgs::msg::Pose current_pose = pose.pose;
      geometry_msgs::msg::Pose desired_pose = pose.pose;
      desired_pose.position = target;

      std::vector<geometry_msgs::msg::Pose> waypoints;
      waypoints.push_back(current_pose);
      waypoints.push_back(desired_pose);
      
      const double jump_threshold = 1;
      const double eef_step = 0.001;
      moveit_msgs::msg::RobotTrajectory trajectory;
      move_group->setPoseReferenceFrame("base");
      move_group->setEndEffectorLink("tip");
      move_group->setPoseTarget(desired_pose);
      
      /*
      double fraction = move_group->computeCartesianPath(waypoints,
							 eef_step,
							 jump_threshold, trajectory,
							 false,
							 &code);
      // disable collision for this one because the model is a little bit off.
      
      std::cout << "fraction " << fraction << " " << code.val << std::endl;
      for( int i=0; i<trajectory.joint_trajectory.points.size(); i++ ){
	for( int j=0; j<trajectory.joint_trajectory.points[i].positions.size(); j++){
	  std::cout << std::setw(13) << trajectory.joint_trajectory.points[i].positions[j];
	}
	trajectory.joint_trajectory.points[i].time_from_start.sec *= 4;
      }
      */

      std::cout << "Inserting" << std::endl;
      move_group->move();
      std::cout << "Inserted" << std::endl;

      /*
      for( int i=0; i<trajectory.joint_trajectory.points.size(); i++ ){
	for( int j=0; j<trajectory.joint_trajectory.points[i].positions.size(); j++){
	  //std::cout << std::setw(13) << trajectory.joint_trajectory.points[i].positions[j];
	}
	trajectory.joint_trajectory.points[i].time_from_start.sec *= 4;
      }

      auto action_client =
	rclcpp_action::create_client<moveit_msgs::msg::ExecuteTrajectoryAction>
	(this, "/execute_trajectory");
      action_client->wait_for_action_server(std::chrono::seconds(2));

      moveit_msgs::msg::ExecuteTrajectoryGoal goal;
      goal.trajectory.joint_trajectory = trajectory.joint_trajectory;
      auto goal_handle_future = action_client->async_send_goal(goal_msg);
      if (rclcpp::spin_until_future_complete(this, goal_handle_future) !=
	  rclcpp::FutureReturnCode::SUCCESS){
      }
      */
    }
  }
  
};

int main( int argc, char** argv ){

  rclcpp::init(argc, argv);

  auto move_group_node = std::make_shared<spine_control>();

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(move_group_node);
  std::thread([&executor]() { executor.spin(); }).detach();

  move_group_node->create_mg();
  getchar();
  rclcpp::shutdown();
  
  return 0;
}

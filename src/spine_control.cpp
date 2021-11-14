#include <memory>
#include <iostream>

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

public:

  spine_control( ) :
    Node("spine_control"),
    valid_entry(false),
    valid_target(false){
    
    sub_igtl_point = this->create_subscription<ros2_igtl_bridge::msg::Point>
      ("/IGTL_POINT_IN", 10, std::bind(&spine_control::igtl_point_cb, this, std::placeholders::_1));

    this->declare_parameter<std::string>("robot_description", "world");
    rclcpp::SyncParametersClient::SharedPtr param_client = std::make_shared<rclcpp::SyncParametersClient>(this, "robot_state_publisher");
    while(!param_client->wait_for_service(1s)){}
    std::string robot_description = param_client->get_parameter<std::string>("robot_description");
    std::vector<rclcpp::Parameter> params;
    params.push_back( rclcpp::Parameter("robot_description", robot_description ) );
    this->set_parameters(params);
  }

  void igtl_point_cb( const ros2_igtl_bridge::msg::Point::SharedPtr point ){

    if(point->name.compare("Entry") == 0){
      valid_entry = true;
      entry.x = point->pointdata.x / 1000.0; // Convert mm to m
      entry.y = point->pointdata.y / 1000.0; // Convert mm to m
      entry.z = point->pointdata.z / 1000.0; // Convert mm to m      
      RCLCPP_INFO(LOGGER, "[spine_control] Entry point has been received: \n");
    } else if(point->name.compare("Target") == 0){
      valid_target = true;
      target.x = point->pointdata.x / 1000.0; // Convert mm to m
      target.y = point->pointdata.y / 1000.0; // Convert mm to m
      target.z = point->pointdata.z / 1000.0; // Convert mm to m      
      RCLCPP_INFO(LOGGER, "[spine_control] Target point has been received: \n");
    }
    if( valid_entry && valid_target ){
      execute();
    }
  }

  void execute(){

    rclcpp::NodeOptions node_options;
    node_options.automatically_declare_parameters_from_overrides(true);

    std::string PLANNING_GROUP = "ur_manipulator";
    moveit::planning_interface::MoveGroupInterface
      move_group(shared_from_this(), PLANNING_GROUP);
    moveit::core::RobotStatePtr current_state = move_group.getCurrentState();
    std::cout << current_state << std::endl;

    tf2::Vector3 p1( entry.x, entry.y, entry.z ); 
    tf2::Vector3 p2( target.x, target.y, target.z ); 
    tf2::Vector3 a = p2-p1;
    a = a / a.length();

    tf2::Vector3 o( 0.0, 1.0, 0.0 );
    tf2::Vector3 n = o.cross(a);
      
    tf2::Matrix3x3 R( n.getX(), o.getX(), a.getX(),
		      n.getY(), o.getY(), a.getY(),
		      n.getZ(), o.getZ(), a.getZ() );

    geometry_msgs::msg::PoseStamped pose;
    tf2::Transform tfpose(R, p1);
    pose.pose.position.x = tfpose.getOrigin().getX();
    pose.pose.position.y = tfpose.getOrigin().getY();
    pose.pose.position.z = tfpose.getOrigin().getZ();
    pose.pose.orientation.x = tfpose.getRotation().getX();
    pose.pose.orientation.y = tfpose.getRotation().getY();
    pose.pose.orientation.z = tfpose.getRotation().getZ();
    pose.pose.orientation.w = tfpose.getRotation().getW();
    //pose.pose = tf2::toMsg<tf2::Transform,geometry_msgs::msg::PoseStamped>(tfpose);
    //tf2::convert( tfpose, pose);
    
    sensor_msgs::msg::JointState seed;
    seed.name.push_back( "shoulder_pan_joint" );
    seed.name.push_back( "shoulder_lift_joint" );
    seed.name.push_back( "elbow_joint" );
    seed.name.push_back( "wrist_1_joint" );
    seed.name.push_back( "wrist_2_joint" );
    seed.name.push_back( "wrist_3_joint" );
    seed.position.push_back( -0.4735596817074883 );
    seed.position.push_back( -1.7595926907275998 );
    seed.position.push_back( 2.6589746902651235 );
    seed.position.push_back( -0.9003782216382002 );
    seed.position.push_back( -0.506688355320275 );
    seed.position.push_back( -0.7767779676822881 );

    auto request = std::make_shared<moveit_msgs::srv::GetPositionIK::Request>();
    request->ik_request.group_name = PLANNING_GROUP;
    request->ik_request.robot_state.joint_state = seed;
    request->ik_request.robot_state.is_diff = false;
    request->ik_request.avoid_collisions = true;
    request->ik_request.pose_stamped = pose;
    //request->ik_request.timeout = ros::Duration(5);

    //rclcpp::Client<moveit_msgs::srv::GetPositionIK>::SharedPtr client =
    //  this->create_client<moveit_msgs::srv::GetPositionIK>("compute_ik");
    //auto result = client->async_send_request(request);

    //if (rclcpp::spin_until_future_complete(shared_from_this(), result) ==
    //rclcpp::FutureReturnCode::SUCCESS){
      moveit_msgs::msg::RobotTrajectory trajectory;
      move_group.setStartState(*current_state);
      move_group.setJointValueTarget(seed);
      //move_group.setJointValueTarget(result.get()->solution.joint_state);
      moveit::planning_interface::MoveGroupInterface::Plan plan;
      moveit::planning_interface::MoveItErrorCode code = move_group.plan( plan );
      if( (bool)code ){
	move_group.execute(plan);
      }
      //}
      //else {}
  }
  
  //void result_cb( const moveit_msgs::msg::ExecuteTrajectoryActionResult& result ){
    /*
    ros::AsyncSpinner spinner(1);
    spinner.start();
    std::string PLANNING_GROUP = "ur5";
    moveit::planning_interface::MoveGroupInterface move_group(PLANNING_GROUP);
    
    geometry_msgs::PoseStamped current_pose = move_group.getCurrentPose( "needle" );

    geometry_msgs::Pose desired_pose = current_pose.pose;
    desired_pose.position = target;

    std::vector<geometry_msgs::Pose> waypoints;
    waypoints.push_back(current_pose.pose);
    waypoints.push_back(desired_pose);


    ros::Duration(4).sleep();
    
    const double jump_threshold = 0.0;
    const double eef_step = 0.005;
    moveit_msgs::RobotTrajectory trajectory;
    double fraction = move_group.computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory);

    for( int i=0; i<trajectory.joint_trajectory.points.size(); i++ ){
      for( int j=0; j<trajectory.joint_trajectory.points[i].positions.size(); j++){
	//std::cout << std::setw(13) << trajectory.joint_trajectory.points[i].positions[j];
      }
      trajectory.joint_trajectory.points[i].time_from_start *= 4.0;  // slow motion down
    }

    actionlib::SimpleActionClient<moveit_msgs::ExecuteTrajectoryAction> client("/execute_trajectory", true);
    std::cout << "Wait for server" << std::endl;
    client.waitForServer();    

    moveit_msgs::ExecuteTrajectoryGoal goal;
    goal.trajectory.joint_trajectory = trajectory.joint_trajectory;
    client.sendGoal(goal);
    bool finished_before_timeout = client.waitForResult(ros::Duration(60.0));
    if(finished_before_timeout){
      actionlib::SimpleClientGoalState state = client.getState();
      ROS_INFO("Action finished: %s",state.toString().c_str());
    }
    */
  //}

  
};

int main( int argc, char** argv ){

  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<spine_control>());
  getchar();
  rclcpp::shutdown();

  //auto node = std::make_shared<spine_control>("spinecontrol_node");

  //ros::NodeHandle nh;
  //ros::AsyncSpinner spinner(1);
  //spinner.start();
  
  //spine_control control(nh);

  //ros::spinOnce();
  
  
  return 0;
}

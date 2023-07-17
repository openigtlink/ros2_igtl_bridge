#include <memory>
#include <chrono>
#include <functional>
#include <string>
#include <math.h>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "ros2_igtl_bridge/msg/string.hpp"
#include "ros2_igtl_bridge/msg/transform.hpp"
#include "ros2_igtl_bridge/msg/point_array.hpp"
#include "ros2_igtl_bridge/msg/pose_array.hpp"
// #include "sensor_msgs/msg/image.hpp"

using std::placeholders::_1;
using namespace std::chrono_literals;

class IGTLPublisher : public rclcpp::Node
{
public:
  IGTLPublisher(): Node("minimal_subscriber"), count_(0)
  {
    string_publisher_      = this->create_publisher<ros2_igtl_bridge::msg::String>("IGTL_STRING_OUT", 10);
    transform_publisher_   = this->create_publisher<ros2_igtl_bridge::msg::Transform>("IGTL_TRANSFORM_OUT", 10);
    pointarray_publisher_  = this->create_publisher<ros2_igtl_bridge::msg::PointArray>("IGTL_POINT_OUT", 10);
    posearray_publisher_   = this->create_publisher<ros2_igtl_bridge::msg::PoseArray>("IGTL_POSEARRAY_OUT", 10);
    // image_publisher_       = this->create_publisher<sensor_msgs::msg::Image>("IGTL_IMAGE_OUT", 10);

    timer_ = this->create_wall_timer(500ms, std::bind(&IGTLPublisher::timer_callback, this));    
  }

private:

  void timer_callback()
  {
    // String message
    auto string_msg = ros2_igtl_bridge::msg::String();
    string_msg.name = "test_string";
    string_msg.data = "Hello, world! " + std::to_string(count_++);
    RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", string_msg.data.c_str());
    string_publisher_->publish(string_msg);

    // Transform message
    auto transform_msg   = ros2_igtl_bridge::msg::Transform();
    transform_msg.name = "test_transform";
    transform_msg.transform.translation.x = 10.0;
    transform_msg.transform.translation.y = 20.0;
    transform_msg.transform.translation.z = 30.0;
    transform_msg.transform.rotation.x = 0.0;
    transform_msg.transform.rotation.y = 0.0;
    transform_msg.transform.rotation.z = 0.0;
    transform_msg.transform.rotation.w = 1.0;
    RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", transform_msg.name.c_str());
    transform_publisher_->publish(transform_msg);

    // Point message
    auto pointarray_msg       = ros2_igtl_bridge::msg::PointArray();
    pointarray_msg.name = "test_point";
    int npoints = 20;
    pointarray_msg.pointdata.resize(npoints);
    for (int i = 0; i < npoints; i ++)
      {
      pointarray_msg.pointdata[i].x = 20.0 * cos(M_PI * (float)i / 10.0);
      pointarray_msg.pointdata[i].y = 20.0 * sin(M_PI * (float)i / 10.0);
      pointarray_msg.pointdata[i].z = 5.0* (float)i;
      }

    RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", pointarray_msg.name.c_str());
    pointarray_publisher_->publish(pointarray_msg);

    // PoseArray message
    auto posearray_msg       = ros2_igtl_bridge::msg::PoseArray();
    posearray_msg.name = "test_pose_array";
    const int nposes = 5;
    posearray_msg.posearray.poses.resize(nposes);
    for (int i = 0; i < nposes; i ++)
      {
      posearray_msg.posearray.poses[i].position.x = 10.0 * (float)i;
      posearray_msg.posearray.poses[i].position.y = 20.0 * (float)i;
      posearray_msg.posearray.poses[i].position.z = 30.0 * (float)i;
      posearray_msg.posearray.poses[i].orientation.x = 0.0;
      posearray_msg.posearray.poses[i].orientation.y = 0.0;
      posearray_msg.posearray.poses[i].orientation.z = 0.0;
      posearray_msg.posearray.poses[i].orientation.w = 1.0;
      }
    
    RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", posearray_msg.name.c_str());
    posearray_publisher_->publish(posearray_msg);
    
    // // Image message
    // auto image_msg;      = sensor_msgs::msg::Image();
    // RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", image_msg.name.c_str());
    // image_publisher_->publish(image_msg);
    
  }

  size_t count_;
  
  rclcpp::TimerBase::SharedPtr timer_;
  
  rclcpp::Publisher<ros2_igtl_bridge::msg::String>::SharedPtr      string_publisher_;
  rclcpp::Publisher<ros2_igtl_bridge::msg::Transform>::SharedPtr   transform_publisher_;
  rclcpp::Publisher<ros2_igtl_bridge::msg::PointArray>::SharedPtr pointarray_publisher_;
  rclcpp::Publisher<ros2_igtl_bridge::msg::PoseArray>::SharedPtr   posearray_publisher_;
  // rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr           image_publisher_;

};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<IGTLPublisher>());
  rclcpp::shutdown();
  return 0;
}

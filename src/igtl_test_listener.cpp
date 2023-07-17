#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "ros2_igtl_bridge/msg/string.hpp"
#include "ros2_igtl_bridge/msg/transform.hpp"
#include "ros2_igtl_bridge/msg/point_array.hpp"
#include "ros2_igtl_bridge/msg/pose_array.hpp"
// #include "sensor_msgs/msg/image.hpp"

using std::placeholders::_1;

class IGTLSubscriber : public rclcpp::Node
{
public:
  IGTLSubscriber(): Node("minimal_subscriber")
  {
    string_subscription_ =
      this->create_subscription<ros2_igtl_bridge::msg::String>("IGTL_STRING_IN", 10,
                                                               std::bind(&IGTLSubscriber::string_callback, this, _1));
    transform_subscription_ =
      this->create_subscription<ros2_igtl_bridge::msg::Transform>("IGTL_TRANSFORM_IN", 10,
                                                               std::bind(&IGTLSubscriber::transform_callback, this, _1));
    pointarray_subscription_ =
      this->create_subscription<ros2_igtl_bridge::msg::PointArray>("IGTL_POINT_IN", 10,
                                                               std::bind(&IGTLSubscriber::pointarray_callback, this, _1));
    posearray_subscription_ =
      this->create_subscription<ros2_igtl_bridge::msg::PoseArray>("IGTL_POSEARRAY_IN", 10,
                                                               std::bind(&IGTLSubscriber::posearray_callback, this, _1));
    // image_subscription_ =
    //   this->create_subscription<sensor_msgs::msg::Image>("IGTL_IMAGE_IN", 10,
    //                                                            std::bind(&IGTLSubscriber::image_callback, this, _1));
  }

private:
  void string_callback(const ros2_igtl_bridge::msg::String::SharedPtr msg) const
  {
    RCLCPP_INFO(this->get_logger(), "Received ros2_igtl_bridge::msg::String");
    RCLCPP_INFO(this->get_logger(), "    name: '%s'", msg->name.c_str());
    RCLCPP_INFO(this->get_logger(), "    data: '%s'", msg->data.c_str());    
  }
  
  void transform_callback(const ros2_igtl_bridge::msg::Transform::SharedPtr msg) const
  {
    RCLCPP_INFO(this->get_logger(), "Received ros2_igtl_bridge::msg::Transform");
    RCLCPP_INFO(this->get_logger(), "    name: '%s'", msg->name.c_str());
    RCLCPP_INFO(this->get_logger(), "    translation : (%f, %f, %f)",
                msg->transform.translation.x,
                msg->transform.translation.y,
                msg->transform.translation.z);
    RCLCPP_INFO(this->get_logger(), "    rotation    : (%f, %f, %f, %f)",
                msg->transform.rotation.x,
                msg->transform.rotation.y,
                msg->transform.rotation.z,
                msg->transform.rotation.w);
  }
  
  void pointarray_callback(const ros2_igtl_bridge::msg::PointArray::SharedPtr msg) const
  {
    RCLCPP_INFO(this->get_logger(), "Received ros2_igtl_bridge::msg::PointArray");
    RCLCPP_INFO(this->get_logger(), "    name        : '%s'", msg->name.c_str());
    int npoints = msg->pointdata.size();
    for (int i = 0; i < npoints; i ++)
      {
      RCLCPP_INFO(this->get_logger(), "    point[%d] : (%f, %f, %f)",
                  i,
                  msg->pointdata[i].x,
                  msg->pointdata[i].y,
                  msg->pointdata[i].z);
      }

  }

  void posearray_callback(const ros2_igtl_bridge::msg::PoseArray::SharedPtr msg) const
  {
    RCLCPP_INFO(this->get_logger(), "Received ros2_igtl_bridge::msg::PoseArray");
    RCLCPP_INFO(this->get_logger(), "    name        : '%s'", msg->name.c_str());

    int nposes = msg->posearray.poses.size();
    RCLCPP_INFO(this->get_logger(), "Number of poses: %d",  nposes);
    for (int i = 0; i < nposes; i ++)
      {
      std::stringstream ss;
      ss << "POSE_" << i;
      RCLCPP_INFO(this->get_logger(), "    position : (%f, %f, %f)",
                  msg->posearray.poses[i].position.x,
                  msg->posearray.poses[i].position.y,
                  msg->posearray.poses[i].position.z);
      RCLCPP_INFO(this->get_logger(), "    orientation : (%f, %f, %f, %f)",
                  msg->posearray.poses[i].orientation.x,
                  msg->posearray.poses[i].orientation.y,
                  msg->posearray.poses[i].orientation.z,
                  msg->posearray.poses[i].orientation.w);
      }
  }
  

  // void image_callback(const sensor_msgs::msg::Image::SharedPtr msg) const
  // {
  //   RCLCPP_INFO(this->get_logger(), "Received sensor_msgs::msg::Image::SharedPtr");
  //   RCLCPP_INFO(this->get_logger(), "    name: '%s'", msg->name.c_str());
  //   RCLCPP_INFO(this->get_logger(), "    data: '%s'", msg->data.c_str());    
  // }

  rclcpp::Subscription<ros2_igtl_bridge::msg::String>::SharedPtr string_subscription_;
  rclcpp::Subscription<ros2_igtl_bridge::msg::Transform>::SharedPtr transform_subscription_;
  rclcpp::Subscription<ros2_igtl_bridge::msg::PointArray>::SharedPtr pointarray_subscription_;
  rclcpp::Subscription<ros2_igtl_bridge::msg::PoseArray>::SharedPtr posearray_subscription_;
    // rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_subscription_;
};


int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<IGTLSubscriber>());
  rclcpp::shutdown();
  return 0;
}

/*=========================================================================

  Program:   Converter Class for PointArray
  Language:  C++

  Copyright (c) Brigham and Women's Hospital. All rights reserved.

  This software is distributed WITHOUT ANY WARRANTY; without even
  the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR
  PURPOSE.  See the above copyright notices for more information.

  =========================================================================*/

#include "rib_converter_pointarray.h"
#include "rib_converter_manager.h"
#include "rclcpp/rclcpp.hpp"
#include "igtlPointMessage.h"
#include "geometry_msgs/msg/point.hpp"

RIBConverterPointArray::RIBConverterPointArray()
  : RIBConverter<ros2_igtl_bridge::msg::PointArray>()
{
}

RIBConverterPointArray::RIBConverterPointArray(rclcpp::Node::SharedPtr n)
  : RIBConverter<ros2_igtl_bridge::msg::PointArray>(n)
{
}

RIBConverterPointArray::RIBConverterPointArray(const char* topicPublish, const char* topicSubscribe, rclcpp::Node::SharedPtr n)
  : RIBConverter<ros2_igtl_bridge::msg::PointArray>(topicPublish, topicSubscribe, n)
{
}

int RIBConverterPointArray::onIGTLMessage(igtl::MessageHeader * header)
{
  igtl::PointMessage::Pointer pointMsg = igtl::PointMessage::New();
  pointMsg->SetMessageHeader(header);
  pointMsg->AllocatePack();

  igtl::Socket::Pointer socket = this->manager->GetSocket();
  if (socket.IsNull())
    {
    return 0;
    }

  bool timeout = false;
  socket->Receive(pointMsg->GetPackBodyPointer(), pointMsg->GetPackBodySize(), timeout);
  int c = pointMsg->Unpack(1);
  
  if ((c & igtl::MessageHeader::UNPACK_BODY) == 0) 
    {
    RCLCPP_ERROR(this->node->get_logger(), "Failed to unpack the message. Datatype: POINT.");
    return 0;
    }
  
  int npoints = pointMsg->GetNumberOfPointElement ();
  
  if (npoints > 0)
    {
    ros2_igtl_bridge::msg::PointArray msg;
    msg.pointdata.resize(npoints);

    for (int i = 0; i < npoints; i ++)
      {
      igtlFloat32 point[3];
      igtl::PointElement::Pointer elem = igtl::PointElement::New();
      pointMsg->GetPointElement (i,elem);
      elem->GetPosition(point);
      msg.pointdata[i].x = point[0];
      msg.pointdata[i].y = point[1];
      msg.pointdata[i].z = point[2];
      msg.name = elem->GetName();
      }
    this->publisher->publish(msg);
    }
  else
    {
    RCLCPP_ERROR(this->node->get_logger(), "Message POINT is empty");
    return 0;
    }
  
  return 1;
  
}


void RIBConverterPointArray::onROSMessage(const ros2_igtl_bridge::msg::PointArray::SharedPtr msg)
{

  igtl::Socket::Pointer socket = this->manager->GetSocket();
  if (socket.IsNull())
    {
    return;
    }

  int pcl_size = msg->pointdata.size();
  if (!pcl_size)
    {
    RCLCPP_ERROR(this->node->get_logger(), "[ROS-IGTL-Bridge] PointArray is empty!");
    return;
    }

  igtl::PointMessage::Pointer pointMsg = igtl::PointMessage::New();
  pointMsg->SetDeviceName(msg->name.c_str());

  for (int i = 0; i<pcl_size;i++)
    {
    std::stringstream ss;
    igtl::PointElement::Pointer pointE;
    pointE = igtl::PointElement::New();
    pointE->SetPosition(msg->pointdata[i].x, msg->pointdata[i].y,msg->pointdata[i].z);
    ss << msg->name << i;
    pointE->SetName(ss.str().c_str());
    pointMsg->AddPointElement(pointE);
    }
  pointMsg->Pack();
  socket->Send(pointMsg->GetPackPointer(), pointMsg->GetPackSize());

}

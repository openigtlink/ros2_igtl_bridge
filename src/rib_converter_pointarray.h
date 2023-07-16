/*=========================================================================

  Program:   Converter Class for PointArray
  Language:  C++

  Copyright (c) Brigham and Women's Hospital. All rights reserved.

  This software is distributed WITHOUT ANY WARRANTY; without even
  the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR
  PURPOSE.  See the above copyright notices for more information.

=========================================================================*/

#ifndef __RIBConverterPointArray_H
#define __RIBConverterPointArray_H

#include "rib_converter.h"

// ROS header files
#include "rclcpp/rclcpp.hpp"

// ROS message header files
#include "ros2_igtl_bridge/msg/point_array.hpp"

// OpenIGTLink message files
#include "igtlStringMessage.h"


class RIBConverterPointArray : public RIBConverter<ros2_igtl_bridge::msg::PointArray>
{

public:
  RIBConverterPointArray();
  RIBConverterPointArray(rclcpp::Node::SharedPtr n);
  RIBConverterPointArray(const char* topicPublish, const char* topicSubscribe, rclcpp::Node::SharedPtr n=NULL);
  
  //virtual uint32_t queueSizePublish() { return 10; }
  //virtual uint32_t queueSizeSubscribe() { return 10; }
  virtual const char* messageTypeString() { return "POINT"; }

public:  
  virtual int onIGTLMessage(igtl::MessageHeader * header);
 protected:
  virtual void onROSMessage(const ros2_igtl_bridge::msg::PointArray::SharedPtr msg);
};

#endif // __RIBConverterPointArray_H

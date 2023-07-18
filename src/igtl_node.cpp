#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <thread>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

#include "igtl_node.h"

#include "rib_converter_manager.h"
#include "rib_converter_string.h"
#include "rib_converter_transform.h"
#include "rib_converter_pointarray.h"
#include "rib_converter_image.h"
#include "rib_converter_posearray.h"

//#include "rib_converter_polydata.h"

#include "igtlOSUtil.h"

using namespace std::chrono_literals;


OpenIGTLinkNode::OpenIGTLinkNode() : Node(IGTL_DEFAULT_NODE_NAME), count_(0)
{
  //publisher_ = this->create_publisher<std_msgs::msg::String>("topic", 10);
  //timer_ = this->create_wall_timer(500ms, std::bind(&OpenIGTLinkNode::timer_callback, this));
   
  //Declare parameters
  this->declare_parameter("RIB_type", "server");
  this->declare_parameter("RIB_server_ip", "127.0.0.1");
  this->declare_parameter("RIB_port", 18944);

}


OpenIGTLinkNode::OpenIGTLinkNode(const std::string nodeName) : Node(nodeName), count_(0)
{
  //publisher_ = this->create_publisher<std_msgs::msg::String>("topic", 10);
  //timer_ = this->create_wall_timer(500ms, std::bind(&OpenIGTLinkNode::timer_callback, this));

  //Declare parameters
  this->declare_parameter("RIB_type", "server");
  this->declare_parameter("RIB_server_ip", "127.0.0.1");
  this->declare_parameter("RIB_port", 18944);
}


void OpenIGTLinkNode::addConverters()
{

  RCLCPP_INFO(get_logger(), "Adding converters.");

  
  this->converterManager = new RIBConverterManager;
  rclcpp::Node::SharedPtr ptr = shared_from_this();
  //OpenIGTLinkNode::SharedPtr ptr = shared_from_this();
  this->converterManager->setNode(ptr);

  // Regisgter converter classes
  RIBConverterTransform* transform = new RIBConverterTransform;
  RIBConverterString* string = new RIBConverterString;
  RIBConverterPointArray * pointarray = new RIBConverterPointArray;
  RIBConverterPoseArray * posearray = new RIBConverterPoseArray;
  // RIBConverterImage* image = new RIBConverterImage;
  // RIBConverterPolyData* polydata = new RIBConverterPolyData;

  this->converterManager->AddConverter(string, 10, "IGTL_STRING_IN", "IGTL_STRING_OUT");
  this->converterManager->AddConverter(transform, 10, "IGTL_TRANSFORM_IN", "IGTL_TRANSFORM_OUT");
  this->converterManager->AddConverter(pointarray, 10, "IGTL_POINT_IN", "IGTL_POINT_OUT");
  this->converterManager->AddConverter(posearray, 10, "IGTL_POSEARRAY_IN", "IGTL_POSEARRAY_OUT");
  // this->converterManager->AddConverter(image, 10, "IGTL_IMAGE_IN", "IGTL_IMAGE_OUT");
  //this->converterManager->AddConverter(polydata, 10, "IGTL_POLYDATA_IN", "IGTL_POLYDATA_OUT");


  RCLCPP_INFO(get_logger(), "Checking parameters.");  
  
  
  // run bridge as client or server
  std::string type;
  if(this->get_parameter("RIB_type",type))
    {
    if(type == "client")
      {
      this->isServer = false;
      }
    else if(type == "server")
      {
      this->isServer = true;
      }
    else
      {
      RCLCPP_ERROR(get_logger(), "Unknown Value for Parameter 'RIB_type'");
      // TODO
      rclcpp::shutdown();
      }
    }
  else
    {
    short srvcl = 0;
    while(1)
      {
      std::cout << "Please type <1> or <2> to run node as OpenIGTLink client or server"<<std::endl;
      std::cout << "1 : SERVER" << std::endl << "2 : CLIENT" << std::endl;
      std::cin>>srvcl;
      
      if (srvcl==1)
        {
        this->isServer = true;
        break;
        }
      else if (srvcl==2)
        {
        this->isServer = false;
        break;
        }
      else
        {
        RCLCPP_ERROR(get_logger(), "Invalid answer.");
        }
      }
    }

  this->port     = 18944;  // std port
  
  if (this->isServer)
    {
    if(this->get_parameter("RIB_port", this->port))
      {}
    else
      {
      RCLCPP_INFO(get_logger(), "Input socket port: ");
      std::cin >> this->port;
      }
    }
  else // if this is a client
    {
    // get IP address
    if(this->get_parameter("RIB_server_ip", this->address))
      {}
    else
      {
      RCLCPP_INFO(get_logger(), "Please enter ServerIP: ");
      std::cin >> this->address;
      }
    // get port
    if(this->get_parameter("RIB_port",this->port))
      {}
    else
      {
      RCLCPP_INFO(get_logger(), "Please enter ServerPort:  ");
      std::cin >> this->port;
      }
    }

  RCLCPP_INFO(get_logger(), "ROS-IGTL-Bridge is up and Running.");
  
  // start OpenIGTLink thread
  this->igtlThread = std::thread(&OpenIGTLinkNode::IGTLThread, this);
  this->igtlThread.detach();
}


//----------------------------------------------------------------------
igtl::Socket::Pointer OpenIGTLinkNode::GetSocketPointer()
{
  igtl::Socket::Pointer socket_ptr = static_cast<igtl::Socket::Pointer>(socket);
  return socket_ptr;
}


//----------------------------------------------------------------------
int OpenIGTLinkNode::StartIGTLServer()
{

  static igtl::ServerSocket::Pointer serverSocket;

  if (serverSocket.IsNull()) // if called for the first time
    {
    serverSocket = igtl::ServerSocket::New();
    int c = serverSocket->CreateServer(this->port);
    if (c < 0)
      {
      RCLCPP_ERROR(get_logger(), "Cannot create a server socket.");
      return 0;
      }
    }
  
  RCLCPP_INFO(get_logger(), "Server socket created. Please connect to port: %d",port);
  
  // wait for connection
  while (1)
    {
    RCLCPP_ERROR(get_logger(), "Waiting for connection.");
    this->socket = serverSocket->WaitForConnection(1000);
    this->converterManager->SetSocket(this->socket);
    if (this->socket.IsNotNull()) 
      {
      RCLCPP_ERROR(get_logger(), "Connected.");      
      return 1;
      }
    }
  
  return 0;
}

//----------------------------------
int OpenIGTLinkNode::ConnectToIGTLServer()
{
  static igtl::ClientSocket::Pointer clientSocket;

  if (clientSocket.IsNull()) // if called for the first time
    {
    clientSocket = igtl::ClientSocket::New();
    }
  
  // connect to server
  int r = clientSocket->ConnectToServer(this->address.c_str(), this->port);
  
  if (r != 0)
    {
    RCLCPP_ERROR(get_logger(), "Cannot connect to server.");
    return 0;
    }
  
  this->socket = (igtl::Socket *)(clientSocket);
  this->converterManager->SetSocket(this->socket);
  return 1;
}

void OpenIGTLinkNode::IGTLThread()
{
  int r;

  while (1)
    {
    if (this->isServer)
      {
      r = this->StartIGTLServer();
      }
    else
      {
      r = this->ConnectToIGTLServer();
      }
    if (r == 0)
      {
      RCLCPP_ERROR(get_logger(), "Failed to create a socket. Terminating...");
      rclcpp::shutdown();// TODO: Can the thread shutdown the process?
      }
      
    igtl::MessageHeader::Pointer headerMsg;
    headerMsg = igtl::MessageHeader::New();
    igtlUint64 rs = 0;
    int loop = 1;

    RCLCPP_INFO(get_logger(), "Connection established. Start the IGTL loop..");
    while(loop)
      {
      headerMsg->InitPack();
      // receive packet
      bool timeout = false;
      rs = this->socket->Receive(headerMsg->GetPackPointer(), headerMsg->GetPackSize(), timeout);
          
      if (rs == 0)
        {
        this->socket->CloseSocket();
        this->converterManager->SetSocket(NULL);
        loop = 0; // Terminate the thread.
        }
          
      if (rs != headerMsg->GetPackSize())
        continue;
          
      this->converterManager->ProcessIGTLMessage(headerMsg);
      }
    }
}


void OpenIGTLinkNode::timer_callback()
{
  auto message = std_msgs::msg::String();
  message.data = "Hello, world! " + std::to_string(count_++);
  RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str());
  publisher_->publish(message);
}



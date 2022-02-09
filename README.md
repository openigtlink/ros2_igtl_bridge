ROS-IGTL-Bridge
===============

Author:Tobias Frank, Junichi Tokuda (Brigham and Women's Hospital)

This ROS-Node provides an OpenIGTLink bridge to exchange data with ROS2. 
It supports sending and receiving Transformations, Images, Strings, 
PolyData, Points and Pointclouds.

For the OpenIGTLink bridge for ROS1, please refer to:
- [The ROS-IGTL-Bridge repository](https://github.com/openigtlink/ROS-IGTL-Bridge).

For further information regarding the OpenIGTLink protocol please see:
- [The OpenIGTLink Page](http://openigtlink.org/)


----------------------------------------------------------------------------------------------------------------------------------------

Build Instruction
-----------------

The following steps were tested on:

- Ubuntu 20.04 + ROS2 Foxy

First, install OpenIGTLink in your local computer. A detailed instruction can be found at http://openigtlink.org/. In the following instruction, we assume that the build directory for the OpenIGTLink library is located at: ~/igtl/OpenIGTLink-build

    $ cd <your OpenIGTLink directory>
    $ git clone https://github.com/openigtlink/OpenIGTLink.git
    $ mkdir OpenIGTLink-build
    $ cd OpenIGTLink-build
    $ cmake -DBUILD_SHARED_LIBS:BOOL=ON ../OpenIGTLink
    $ make

Install ROS 2 following [the ROS 2 Documentation](https://docs.ros.org/en/foxy/Installation.html). Then create your ROS workspace following the [documentation](https://docs.ros.org/en/foxy/Tutorials/Workspace/Creating-A-Workspace.html) as follows:

    $ source /opt/ros/foxy/setup.bash
    $ mkdir -p ~/dev_ws/src
    $ cd ~/dev_ws/
    $ rosdep install -i --from-path src --rosdistro foxy -y # Make sure to resolve dependency
	
Then download the ros_igtl_bridge package from GitHub:

    $ cd ~/dev_ws/src
    $ git clone https://github.com/openigtlink/ros2_igtl_bridge

and execute catkin_make in your workspace directory:

    $ cd ~/dev_ws/
    $ colcon build --cmake-args -DOpenIGTLink_DIR:PATH=<your OpenIGTLink directory>/OpenIGTLink-build

Note that you may need to use the absolute path, when setting the OpenIGTLink_DIR cmake variable.

To run the bridge, type:

    $ . install/setup.bash
    $ ros2 run ros2_igtl_bridge igtl_node

If the bridge is set up, you can launch the test procedure for communication with [3D Slicer] (https://www.slicer.org/):

    $ ros2 run ros_igtl_bridge igtl_test_publisher

It is possible to set the bridge parameters from the command line:

    $ ros2 run ros2_igtl_bridge igtl_node --ros-args -p RIB_port:=18944 -p RIB_type:="client" -p RIB_server_ip:="localhost"

You can also edit the launch file and set your IP & Port. Open the bridge.launch file and edit the parameters:

    $ <param name="RIB_server_ip" value="111.111.111.111" type="str"/>
    $ <param name="RIB_port" value="18944" type="int"/>
    $ <param name="RIB_type" value="client" type="str"/>
    
Then run the launch file as:

    $ ros2 launch ros2_igtl_bridge bridge.launch

The node can be run as server or client. If you executed the test procedure, the node will send
a "ROS_IGTL_Test_Transform" with random translation. 


References
----------
1. Frank T, Krieger A, Leonard S, Patel NA, Tokuda J. ROS-IGTL-Bridge: an open network interface for image-guided therapy using the ROS environment. Int J Comput Assist Radiol Surg. 2017 May 31. doi: 10.1007/s11548-017-1618-1. PubMed PMID: [28567563](https://www.ncbi.nlm.nih.gov/pubmed/?term=28567563).



    
    



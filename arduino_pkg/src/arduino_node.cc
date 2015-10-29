#include <std_srvs/Empty.h>
#include <std_msgs/Float32.h>
#include <boost/thread/mutex.hpp>
#include <ros/ros.h>
#include <arduino_pkg/SetPID.h>

// Dynamic reconfigure includes.
#include <dynamic_reconfigure/server.h>
// Auto-generated from cfg/ directory.
#include <arduino_pkg/arduino_nodeConfig.h>

/**
  * This class implements a ros node that provides services and status updates for the arduino motor controller.
  */
class ArduinoNode
{
public:
  ///default constructor  
  ArduinoNode();
  ///destructor
  virtual ~ArduinoNode();

private:
  ///a node handle to the local (home) node space
  ros::NodeHandle nh_;
  
  ///a node handle to the global node space
  ros::NodeHandle n_;

  ///forwards dynamic reconfigure targets to the arduino node service
  ros::ServiceClient request_pid_;

  ///procides a server connection to dynamic reconfigure
  dynamic_reconfigure::Server<arduino_pkg::arduino_nodeConfig> dr_srv;
  dynamic_reconfigure::Server<arduino_pkg::arduino_nodeConfig>::CallbackType cb;

  ///callback for the dynamic reconfigure events
  ///TODO: don't forget to fill in the correct parameters in the cfg/arduino_node.cfg file
  void configCallback(arduino_pkg::arduino_nodeConfig &config, uint32_t level)
  {
      ROS_INFO("Received a request to change PID parameters");
      //read out configs
      //TODO: here read the parameters from the updated config and call the PID service 
      arduino_pkg::SetPID pid_request;
      //TODO: next, call the service with the correct parameters

  } 
};


ArduinoNode::ArduinoNode()
{
  std::cerr<<"starting node\n";
  nh_ = ros::NodeHandle("~");
  n_ = ros::NodeHandle();

  //TODO: here fill in correct name of PID service
  request_pid_= n_.serviceClient<arduino_pkg::SetPID>("set_pid");  

  // Set up a dynamic reconfigure server.
  // This should be done before reading parameter server values.
  cb = boost::bind(&ArduinoNode::configCallback, this, _1, _2);
  dr_srv.setCallback(cb);
  
}

ArduinoNode::~ArduinoNode()
{
}

int main( int argc, char* argv[] )
{
    ros::init(argc, argv, "arduino_node");
    ArduinoNode arduinoNode;
    ros::spin();
    return 0;
}

 

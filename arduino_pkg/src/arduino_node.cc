#include <std_srvs/Empty.h>
#include <std_msgs/Float32.h>
#include <boost/thread/mutex.hpp>
#include <boost/thread/thread.hpp>
#include <ros/ros.h>

#include "comm.h"
#include <arduino_pkg/SetPosition.h>
#include <arduino_pkg/MotorState.h>

// Dynamic reconfigure includes.
#include <dynamic_reconfigure/server.h>
// Auto-generated from cfg/ directory.
#include <arduino_pkg/arduino_nodeConfig.h>

class ArduinoNode
{
public:
  ArduinoNode();
  virtual ~ArduinoNode();

private:
  ros::NodeHandle nh_;

  ros::Timer heartbeat_status_;
  boost::mutex arduino_mutex_;

  std::string port_;
  ArduinoComm *arduino_comm;

  ros::Publisher status_publisher_;
  
  ros::ServiceServer request_pos_;
  ros::ServiceServer request_on_;
  ros::ServiceServer request_off_;

  //reconfigure stuff
  dynamic_reconfigure::Server<arduino_pkg::arduino_nodeConfig> dr_srv;
  dynamic_reconfigure::Server<arduino_pkg::arduino_nodeConfig>::CallbackType cb;

  bool request_motor_on(std_srvs::Empty::Request  &req,
             std_srvs::Empty::Response &res );
  bool request_motor_off(std_srvs::Empty::Request  &req,
             std_srvs::Empty::Response &res );
  bool request_motor_pos(arduino_pkg::SetPosition::Request  &req,
             arduino_pkg::SetPosition::Response &res );
  
  void publishStatus(const ros::TimerEvent& event);

  float encoder_pos, motor_curr, setpoint, kp, ki, kd;
  bool isConnected;

  void configCallback(arduino_pkg::arduino_nodeConfig &config, uint32_t level)
  {
      if(arduino_comm == NULL) return;
      //read out configs
      //TODO: here read the parameters from the updated config and call the PID parameter update function
  } 
};


ArduinoNode::ArduinoNode():arduino_comm(NULL)
{
  std::cerr<<"starting node\n";
  isConnected = false;
  nh_ = ros::NodeHandle("~");
  
  // Set up a dynamic reconfigure server.
  // This should be done before reading parameter server values.
  cb = boost::bind(&ArduinoNode::configCallback, this, _1, _2);
  dr_srv.setCallback(cb);
  
  nh_.param<std::string>("port", port_,"/dev/ttyACM0");
  arduino_comm = new ArduinoComm(port_);
  
  //This is how you advertise a service
  request_on_ = nh_.advertiseService("motor_on", &ArduinoNode::request_motor_on, this);
  //TODO: advertise services for motor_off and motor_pos
  
  //create publisher for status messages
  status_publisher_ = ros::NodeHandle().advertise<arduino_pkg::MotorState>("motor_state", 10); 
  //call the publish status function at 100 Hz
  heartbeat_status_ = nh_.createTimer(ros::Duration(0.01), &ArduinoNode::publishStatus, this);

  encoder_pos = 0, motor_curr = 0, setpoint = 0, kp = 0, ki = 0, kd = 0;
  std::cerr<<"connecting to arduino board at "<<port_<<std::endl;
  isConnected = arduino_comm->connect();
}

ArduinoNode::~ArduinoNode()
{
    delete arduino_comm;
}

void ArduinoNode::publishStatus(const ros::TimerEvent& event) 
{ 
  
  if(!isConnected) return;
  arduino_mutex_.lock();
  arduino_comm->getStatus(encoder_pos, motor_curr, setpoint, kp, ki, kd);
  arduino_mutex_.unlock();
 
  //TODO: here create a MotorState message, update values and publish it on the topic 

  return;
}

bool ArduinoNode::request_motor_on(std_srvs::Empty::Request  &req,
             std_srvs::Empty::Response &res ) {

   //This is how you should pass service calls to your board
   arduino_mutex_.lock();
   arduino_comm->setOn();
   arduino_mutex_.unlock();
   
   return true;
}

bool ArduinoNode::request_motor_off(std_srvs::Empty::Request  &req,
             std_srvs::Empty::Response &res ) {
   
   //TODO: here call OFF service 
   return true;
}

bool ArduinoNode::request_motor_pos(arduino_pkg::SetPosition::Request  &req,
             arduino_pkg::SetPosition::Response &res ) {
   //TODO: here call POS service
    return true;
}

int main( int argc, char* argv[] )
{
    ros::init(argc, argv, "arduino_node");
    ArduinoNode arduinoNode;
    ros::spin();
    return 0;
}

 

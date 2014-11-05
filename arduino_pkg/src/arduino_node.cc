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

  ///This timer ensures we publish status messages at a fixed frequency
  ros::Timer heartbeat_status_;
  ///mutex to protect the arduino communications
  boost::mutex arduino_mutex_;

  ///the port to connect to
  std::string port_;
  ///a communication object
  ArduinoComm *arduino_comm;

  ///publisher for status messages
  ros::Publisher status_publisher_;
  
  ///provides services for setting a new desired position
  ros::ServiceServer request_pos_;
  ///provides a service to switch on the motors
  ros::ServiceServer request_on_;
  ///provides a service to switch off the motors
  ros::ServiceServer request_off_;

  ///procides a server connection to dynamic reconfigure
  dynamic_reconfigure::Server<arduino_pkg::arduino_nodeConfig> dr_srv;
  dynamic_reconfigure::Server<arduino_pkg::arduino_nodeConfig>::CallbackType cb;

  ///callbacks for the services
  bool request_motor_on(std_srvs::Empty::Request  &req,
             std_srvs::Empty::Response &res );
  bool request_motor_off(std_srvs::Empty::Request  &req,
             std_srvs::Empty::Response &res );
  bool request_motor_pos(arduino_pkg::SetPosition::Request  &req,
             arduino_pkg::SetPosition::Response &res );

  ///callback for the ros timer, publishes the current status
  void publishStatus(const ros::TimerEvent& event);

  ///storage for status variables
  float encoder_pos, motor_curr, setpoint, kp, ki, kd;
  ///is the connection active?
  bool isConnected;

  ///callback for the dynamic reconfigure events
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

 

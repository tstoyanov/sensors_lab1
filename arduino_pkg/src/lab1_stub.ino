//TODO: change this varriable to 1 once you are ready to compile with rosserial_arduino support
#define ROS_SUPPORT 0

#if ROS_SUPPORT
#define USE_USBCON
#include <ros.h>
#include <arduino_pkg/MotorState.h>
#include <arduino_pkg/SetPosition.h>
#include <arduino_pkg/SetPID.h>
#include <arduino_pkg/SetVelocity.h>
#include <std_srvs/Empty.h>
#endif

//======================= Struct Definitions ======================
//this struct holds the state values of a desired control point
struct ControlStates
{
  float r_; //setpoint value at current iteration
  float rf_; //final setpoint value
  float ri_; //initial setpoint value
  float e_; //error
  float de_; //error derivative

  float ti_; //time when we initialized motion (seconds)
  float T_; //time for executing the loop

  float u_; //computed control

  bool active_; //flag indicating whether the corresponding controller is active or not

  ControlStates(float r, float rf, float ri, float e, float de, float ti, float T, int u, bool active) : r_(r), rf_(rf), ri_(ri), e_(e), de_(de), ti_(ti), T_(T), u_(u), active_(active) {};
};

//this struct holds the variables for a PID controller
struct PIDParameters {
  float Kp_;
  float Kd_;
  float Ki_;
  float u_max_; //Maximum controller output (<= max PWM)
  float u_min_; //Minimum controller output [>= -(max PWM)]
  float I_;     //Serves as memory for the integral term [i.e., I=dT*(Ki*e_0, ... , Ki*e_t)]

  PIDParameters(float Kp, float Ki, float Kd, float u_max, float u_min, float I) : Kp_(Kp), Kd_(Kd), Ki_(Ki), u_max_(u_max), u_min_(u_min), I_(I) {};
};

//this struct holds the pin numbers for the motor shield
struct MotorShieldPins {
  int DIR_; //direction pin
  int PWM_; //pwm pin
  int BRK_; //brake pin
  int CUR_; //current sensor

  MotorShieldPins(int DIR, int BRK, int PWM_pin, int CUR) : DIR_(DIR), BRK_(BRK), PWM_(PWM_pin), CUR_(CUR) {};
};

//this struct holds the pin for the encoder and the current number of ticks
struct EncoderStates
{
  int ENC_; //pin for the encoder
  int p_;   //current encoder pulses
  float dp_;//time-derivative of the encoder pulses

  EncoderStates(int PIN, int pos) : ENC_(PIN), p_(pos) {
    dp_ = 0;
  };
};

/////////////////Function Declarations//////////////////
//function to execute upon change of the encoder pin
void readEncoder();
//this function writes a desired control value to the motor
void actuate(float control, MotorShieldPins *mps);
//this function performs position control using the PID and minimumJerk
void positionControl(ControlStates* c_s, EncoderStates* e_s, MotorShieldPins* m_pins, PIDParameters* pid_p);
//this function performs velocity control using the PID and minimumJerk
void velocityControl(ControlStates* c_s, EncoderStates* e_s, MotorShieldPins* m_pins, PIDParameters* pid_p);
//this function computes the next setpoint to obtain a mnimum jerk trajectory
float minimumJerk(float t0, float t, float T, float q0, float qf);
//this function computes controls with a PID
float pid(float e, float de, PIDParameters* p);
//this function should update the motor state variable from the current measured values
void updateState();

#if ROS_SUPPORT
/////////////////ROS Callbacks//////////////////////////
//callback function for setting a new position
void setPosCallback(const arduino_pkg::SetPosition::Request &req, arduino_pkg::SetPosition::Response &res);
//callback function for setting new PID parameters
void setPIDCallback(const arduino_pkg::SetPID::Request &req, arduino_pkg::SetPID::Response &res);
//callback function for setting a new velocities
void setVelCallback(const arduino_pkg::SetVelocity::Request &req, arduino_pkg::SetVelocity::Response &res);
//callback function for switching off the motor brake
void setOn(const std_srvs::Empty::Request &req, std_srvs::Empty::Response &res);
//callback function for switching on the motor brake
void setOff(const std_srvs::Empty::Request &req, std_srvs::Empty::Response &res);
///////////////////////////////////////////////////////////////////////////////////

///////////////////ROS global variables/////////////////////////
ros::NodeHandle nh;
arduino_pkg::MotorState state;
ros::Publisher state_publisher("/motor_state", &state);
ros::ServiceServer<arduino_pkg::SetPosition::Request, arduino_pkg::SetPosition::Response> pos_server("set_pos", &setPosCallback);
ros::ServiceServer<arduino_pkg::SetVelocity::Request, arduino_pkg::SetVelocity::Response> vel_server("set_vel", &setVelCallback);
ros::ServiceServer<arduino_pkg::SetPID::Request, arduino_pkg::SetPID::Response> pid_server("set_pid", &setPIDCallback);
ros::ServiceServer<std_srvs::Empty::Request, std_srvs::Empty::Response> on_server("set_on", &setOn);
ros::ServiceServer<std_srvs::Empty::Request, std_srvs::Empty::Response> off_server("set_off", &setOff);
///////////////////////////////////////////////////////////////////////////////////////////////
#endif

//////////////Global Vars////////////
//PWM resoluion: 12 bit, i.e., 0 - 4095
const float pwm_resolution = 4095;
//storage for timers
int t_new, t_old, t_old_serial;
//sampling time in microseconds
int dT = 1000;
int dT_serial = 50000;
//storage of the electric current through the motors
float current;

//TODO: instantiate these structs to the correct PIN values
//MotorShieldPins *motor1 = new MotorShieldPins(DIR, BRK, PWM, CUR);
//ControlStates *mc1 = new ControlStates(0, 0, 0, 0, 0, 0, 0, 0, false);
//PIDParameters *pid_mc1 = new PIDParameters(KP, KI, KD, pwm_resolution, -pwm_resolution, 0);
//EncoderStates *enc1 = new EncoderStates(ENCODER_PIN, 0);
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void setup() {

  t_old = micros();
  t_old_serial = micros();

  //TODO:
  // -setup serial port for first task. Remove once you start using rosserial_arduino!
  // -set the resolution of the analogWrite(...) function to 12 bit, i.e., between 0 - 4095
  // -set the resolution of the analogRead(...) function to 12 bit, i.e., between 0 - 4095
  // -setup pins for motor shield
  // -write initial values, switch on break
  // -setup encoder pin
  // -turn on pullup resistor for encoder
  // -add interrupt and connect it to readEncoder function
  // -write default low value to PWM

  //Setup ROS related variables

#if ROS_SUPPORT
  ////// ROS initializations////
  nh.initNode();
  nh.advertise(state_publisher);
  nh.advertiseService(pos_server);
  nh.advertiseService(vel_server);
  nh.advertiseService(pid_server);
  nh.advertiseService(on_server);
  nh.advertiseService(off_server);
  ///////////////////////////////
#endif
}

void readEncoder() {
  //TODO: -check motor direction and increment encoder
}

void actuate(float control, MotorShieldPins *mps) {
  //TODO: check motor direction, clamp control value inside pwm_resolution and write pwm
}

//--------------------------------------------------------------------------
void positionControl(ControlStates* c_s, EncoderStates* e_s, MotorShieldPins* m_pins, PIDParameters* pid_p)
{
  //TODO:
  // -update the setpoint using minimum Jerk
  // -calculate position error
  // -calculate derivative of the position error
  // -update the control states for the next iteration

  // -compute control using pid()
}
//--------------------------------------------------------------------------
void velocityControl(ControlStates* c_s, EncoderStates* e_s, MotorShieldPins* m_pins, PIDParameters* pid_p)
{
  //TODO:
  // -update the setpoint using minimum Jerk
  // -calculate velocity error
  // -calculate derivative of the velocity error
  // -update the control states for the next iteration

  // -compute control using pid()
}
//--------------------------------------------------------------------------
float minimumJerk(float t0, float t, float T, float q0, float qf)
{
  //TODO: calculate minimumJerk set point
}
//--------------------------------------------------------------------------
float pid(float e, float de, PIDParameters* p)
{
  //TODO:
  // -update the integral term
  // -compute the control value
  // -clamp the control value and if necessary back-calculate the integral term (to avoid windup)
  // -return control value
}
//--------------------------------------------------------------------------
void updateState() {

}

#if ROS_SUPPORT
//////////////////////ROS Services ////////////////////////////////////
void setPosCallback(const arduino_pkg::SetPosition::Request &req, arduino_pkg::SetPosition::Response &res) {

}

void setPIDCallback(const arduino_pkg::SetPID::Request &req, arduino_pkg::SetPID::Response &res) {

}

void setVelCallback(const arduino_pkg::SetVelocity::Request &req, arduino_pkg::SetVelocity::Response &res) {

}

void setOn(const std_srvs::Empty::Request &req, std_srvs::Empty::Response &res) {

}

void setOff(const std_srvs::Empty::Request &req, std_srvs::Empty::Response &res) {

}
#endif

////////////////////////////////////////////////////////////////
///////////////////////// MAIN LOOP ////////////////////////////
////////////////////////////////////////////////////////////////
void loop() {
#if ROS_SUPPORT
  //spin and check if we should publish
  t_new = micros();
  nh.spinOnce();
  if (abs(t_new - t_old_serial) > dT_serial) {
    updateState();
    state_publisher.publish(&state);
    t_old_serial = t_new;
  }
#endif

  //add the ROS overhead to the time since last loop
  t_new = micros();
  //Do nothing if the sampling period didn't pass yet
  if (abs(t_new - t_old) < dT)
    return;
  t_old = t_new;

  //TODO:
  // -read in current sensor
  // -if mc1 is active, calculate position control and actuate
  // -if in velocity control mode, calculate with velocity control and actuate

}

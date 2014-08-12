//======================= Struct Definitions ======================
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

struct PIDParameters {
  float Kp_;
  float Kd_;
  float Ki_;
  float u_max_; //Maximum controller output (<= max PWM)
  float u_min_; //Minimum controller output [>= -(max PWM)]
  float I_;     //Serves as memory for the integral term [i.e., I=dT*(Ki*e_0, ... , Ki*e_t)]

  PIDParameters(float Kp, float Ki, float Kd, float u_max, float u_min, float I) : Kp_(Kp), Kd_(Kd), Ki_(Ki), u_max_(u_max), u_min_(u_min), I_(I) {};
};

struct MotorShieldPins {
  int DIR_; //direction pin
  int PWM_; //pwm pin
  int BRK_; //brake pin
  int CUR_; //current sensor

  MotorShieldPins(int DIR, int BRK, int PWM_pin, int CUR) : DIR_(DIR), BRK_(BRK), PWM_(PWM_pin), CUR_(CUR) {};
};

struct EncoderStates
{
  int ENC_; //pin for the encoder
  int p_;

  EncoderStates(int PIN, int pos) : ENC_(PIN), p_(pos) {};
};

/////////////////Function Declarations//////////////////
void readEncoder();
void actuate(float control, MotorShieldPins *mps);
void processMessages();
short getShort(char *buf, short pos);
void publishStatus();
void positionControl(ControlStates* c_s, EncoderStates* e_s, MotorShieldPins* m_pins, PIDParameters* pid_p);
float minimumJerk(float t0, float t, float T, float q0, float qf);
float pid(float e, float de, PIDParameters* p);

//////////////Global Vars////////////
const float pwm_resolution = 4095; //PWM resoluion: 12 bit, i.e., 0 - 4095
int t_new, t_old, t_old_comm;
int dT = 1000; //sampling time
float current;

//TODO: instantiate structs to the correct PIN values
//MotorShieldPins *motor1 = new MotorShieldPins(DIR, BRK, PWM, CUR);
//ControlStates *mc1 = new ControlStates(0, 0, 0, 0, 0, 0, 0, 0, false);
//PIDParameters *pid_mc1 = new PIDParameters(KP, KI, KD, pwm_resolution, -pwm_resolution, 0);
//EncoderStates *enc1 = new EncoderStates(ENCODER_PIN, 0);
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void setup() {

  //TODO:
  // -setup serial port
  // -set the resolution of the analogWrite(...) function to 12 bit, i.e., between 0 - 4095
  // -set the resolution of the analogRead(...) function to 12 bit, i.e., between 0 - 4095
  // -initialize timers
  // -setup pins for motor shield
  // -write initial values, switch on break
  // -setup encoder pin
  // -turn on pullup resistor
  // -add interrupt and connect it to readEncoder function
  // -write default low value to PWM
}

void readEncoder() {
  //TODO: -check motor direction and increment encoder
}

void actuate(float control, MotorShieldPins *mps) {
  //TODO: check motor direction, clamp control value inside pwm_resolution and write pwm
}

//--------------------------------------------------------------------------
void processMessages() {

  short target_val = 0;

  if (Serial.available()) {
    delay(10);
    char code[100];
    short i = 0;
    short j = 0;
    while (Serial.available() && i < 100) {
      code[i++] = Serial.read();
    }

    /* * * * * * * communication protocol * * * * * * * *
     * POS val[short] dt[short]                 : set drive to position (float)val and time period to achieve target is (float)dt/1000 [Sec]
     * SET p[short] i[short] d[short]           : set pid parameters to (float)param/10.
     * OFF                                      : disable motor
     * ON                                       : enable all motors
     * * * * * * * * * * * * * * * * * * * * * * * * * */

    while (j < i) {
      if (code[j + 0] == 'P' && code[j + 1] == 'O' && code[j + 2] == 'S') {
        //TODO: position mode. Get message payload and set corresponding controler targets
        j = j + 7; //continue reading from there (3 bytes for POS + 2 bytes val + 2 bytes T
        continue;
      }

      if (code[j + 0] == 'S' && code[j + 1] == 'E' && code[j + 2] == 'T') {
        //TODO: set mode. Read Kp, Ki, Kd and set them
        j = j + 9; //continue reading from there. (3 bytes for SET + 3x2 bytes for each param
        continue;
      }

      if (code[j] == 'O' && code[j + 1] == 'F' && code[j + 2] == 'F') {
        //TODO: enable Break
        j = j + 3;
        continue;
      }
      if (code[j] == 'O' && code[j + 1] == 'N') {
        //TODO: disable Break
        j = j + 2;
        continue;
      }
      return; //we couldn't parse anything meaningful
    }
  }
}

short getShort(char *buf, short pos) {
  byte b1, b2;
  short val;
  b1 = buf[pos];
  b2 = buf[pos + 1];
  val = b2;
  val = val << 8;
  val = val | b1;
  return val;
}

//--------------------------------------------------------------------------
void publishStatus() {
  //TODO: print comma separated values
  //encoder position
  //current sensor value
  //current set point
  //Kp
  //Ki
  //Kd
  //TODO: print termination character: Serial.print("\r\n");
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

////////////////////////////////////////////////////////////////
///////////////////////// MAIN LOOP ////////////////////////////
////////////////////////////////////////////////////////////////
void loop() {
  processMessages();
  //TODO:
  // -check if sampling period has passed
  // -read in current sensor
  // -if mc1 is active, calculate position control and actuate
  // -if communication time has passed, publish status
}

#include <comm.h>
#include <iostream>

using namespace std;

int main(int argc, char * argv[]) {

  ArduinoComm arduino_comm ("/dev/ttyACM0");
  arduino_comm.connect();

  //TODO: implement a simple main loop that prints the current status of the arduino
  //TODO: try sending simple commands to the arduino using the interface
  
}

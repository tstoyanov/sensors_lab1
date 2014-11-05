#ifndef _ARDUINO_COMM_H
#define _ARDUINO_COMM_H

#include <cstdio>
#include <cstring>
#include <errno.h>
#include <termios.h>
#include <unistd.h>
#include <iostream>
#include <fcntl.h>   // File control definitions
#include <sys/types.h>
#include <sys/stat.h>
#include <cstdlib>
#include <boost/thread/mutex.hpp>
#include <boost/thread/thread.hpp>
#include <vector>
/**
  This class implements the basic communication with the arduino board, over the serial port.
  The class provides methods to connect/disconnect to the board, as well as asynchronous read/write
  operations and basic implementation of the protocol defined in the lab instructions.
  */
class ArduinoComm {

    public:
	///Constructor @param port: the string to the arduino port, e.g. "/dev/ttyUSB0"
	ArduinoComm(std::string port) {
	    port_ = port;
	    arduino_fd = -1;
	    isConnected_ = false;
            doUpdateStatus = true;
	}

	///connect to Arduino
	bool connect() {
	    if(port_ == "") {
		return false;
	    }
	    //open file descriptor for reading and writing, no CTTY
	    arduino_fd = open (port_.c_str(), O_RDWR | O_NOCTTY ); 
	    if (arduino_fd < 0)
	    {
		printf("error %d opening %s: %s", errno, port_.c_str(), strerror (errno));
		return false;
	    }
	    sleep(2);

	    set_interface_attribs (arduino_fd, B19200);  // set speed to 19200 bps, 8n1 (no parity)
	    //flush buffers
	    tcflush(arduino_fd, 0);
	    //set connected
	    isConnected_ = true;
	    std::cout<<"Connected!\n";

	    //start STATUS thread (asynchronous reads)
            status_thread_ = boost::thread(boost::bind(&ArduinoComm::updateStatus,this));

	    return true;
	}
	
	///close the communication to the arduino board
        bool closeComm() {
	    //signal update thread to quit
            doUpdateStatus = false;
            //join thread
            status_thread_.join();

            if(isConnected_ && arduino_fd >= 0) {
		//close file descriptor
                close(arduino_fd);
		isConnected_ = false;
                return true;
            }   
            return false;

        }

	///is there an open connection?
	bool isConnected() { return isConnected_; }
	
	///this function returns the last status read by the status thread
	bool getStatus(float &encoderPos, float &currentMA, float &setPoint, 
		float &Kp, float &Ki, float &Kd) {
	    
            if(!isConnected_) return false;
	    //protect data access with a mutex
	    data_mutex_.lock();
	    //TODO: here copy internal state variables into output variables
	    data_mutex_.unlock();
            return true;
	}
	
	///this function sends a target encoder position to the arduino board
        bool setTargetPos (float target, float sec) {
            if(!isConnected_) return false;

	    short val = floor(target);
	    std::cout<<"SET TARGET POS VALUE: "<<val<<std::endl;
	    short t_val = sec*1000; //miliseconds
            short msg_size = 3*sizeof(short) + 3; 
            //create a message buffer
	    char *payload = new char[msg_size];
	    //print the message header
            snprintf(payload,4,"POS");
	    //copy in the encoder value
            memcpy(payload+3,&val,sizeof(short));
	    //copy in the time value
            memcpy(payload+3+sizeof(short),&t_val,sizeof(short));

	    //lock the communication mutex
            arduino_mutex_.lock();
	    //send message
            int wrote = write (arduino_fd, payload , msg_size);
	    //unlock communication mutex
            arduino_mutex_.unlock();

	    //check if the full message was written to the file descriptor
            if(wrote != msg_size) {
                printf("wrote %d, error!\n",wrote);
            }
            printf("msg size %d, string %s\n",msg_size,payload);
	    //free memory
            delete [] payload;

	    //return success or failure
            return (wrote == msg_size);
        }

	///this function sends a new set of target PID parameters to the arduino board
	bool setPIDParams (float kp, float ki, float kd) {
            if(!isConnected_) return false;
	    //TODO: here send message with PID parameters
	}
	
	///this function signals to the arduino board to switch the motors off
	bool setOff () {
            if(!isConnected_) return false;
	    //TODO: here send message to switch on brake
	}	    
	
	///this function signals to the arduino to switch the motors on
	bool setOn () {
            if(!isConnected_) return false;
	    //TODO: here send message to switch off brake
	}	    

	///destructor, exit cleanly
        ~ArduinoComm() {
            this->closeComm();
        }

    private:
	///buffer for message reading
	char buf[1000];
	///port to connect to the arduino
        std::string port_;
	///arduino file descriptor
	int arduino_fd;
	///booleans for connection and update thread
	bool isConnected_, doUpdateStatus;
	///state variables
	float encoderPos_, currentMA_, setPoint_, Kp_, Ki_, Kd_;
	///mutexes to protect members for asynchronous read
	boost::mutex arduino_mutex_, data_mutex_;
	///thread for reading arduino status
        boost::thread status_thread_;

	///this function sets up the serial communication interface
	int set_interface_attribs (int fd, int speed, bool canon=true)
	{
		struct termios tty;
		memset (&tty, 0, sizeof tty);
		if (tcgetattr (fd, &tty) != 0)
		{
			printf("error %d from tcgetattr", errno);
			return -1;
		}
		//set baud rate
		cfsetospeed (&tty, speed);
		cfsetispeed (&tty, speed);

	        //8N1
		tty.c_cflag &= ~PARENB;
		tty.c_cflag &= ~CSTOPB;
		tty.c_cflag &= ~CSIZE;
		tty.c_cflag |= CS8;

		//set cannonical
		tty.c_lflag |= (canon) ? ICANON : ~ICANON;
		tty.c_oflag &= ~OPOST;
		if (tcsetattr (fd, TCSANOW, &tty) != 0)
		{
			printf ("error %d from tcsetattr", errno);
			return -1;
		}
		return 0;
	}

	/**this function reads one line of input from the file descriptor fd, terminated by \r\n
	   returns the most recent full line read in the buffer */
        char* readInput(int fd) {

	    //zero buffer
	    memset(buf,0,sizeof(buf));
            int sofar = 0;
            bool readMore = true;
            char* tok;

	    //try reading
            while (readMore) {
		//protect arduino_fd for asynchronous reading
                arduino_mutex_.lock();
	        //try reading bytes	
		int n = read (fd, buf+sofar, sizeof(buf)-sofar);  // read up to 100 characters if ready to read
		arduino_mutex_.unlock();    
		//if we read anything
		if ( n>0) {
		    //update the positon of the last read character in the buffer
		    sofar+=n;
		    //tokenize the buffer
		    tok = strtok(buf, "\r\n");
		    //if there is a token available
		    if(tok!=NULL) {
			//std::cout<<"TOKEN: "<<tok<<std::endl;
			//return the last token that's not null
			char *tmptok = tok;
			while(tmptok!=NULL) {
			    tok = tmptok;
			    tmptok = strtok(NULL,"\r\n");
			}
			return tok;
		    } else {
			//did not read a full token, we will repeat the read call
			std::cout<<"!!!!Non-Token: "<<tok<<std::endl;

		    }
		    //if there was no delimiter in the last 500 characters, something is wrong
		    if(sofar > 500) {
			std::cout<<"didn't get delimiters\n";
			std::cout<<buf<<std::endl;
			readMore = false;
		    }
		}
            }
            return NULL;
        }

	/*this is the function executed by the status update thread.
	  it reads lines from the descriptor and updates status variables */
        void updateStatus() {
            
	    //storage for the parsed integer values
            std::vector<int> readint;
	    //tokens
            char *tok;
            while(doUpdateStatus) {
		//read a line of input
                char *lineptr = readInput(arduino_fd);
		//TODO: here, tokenize the line, based on your delimiters and store the integer values read
                data_mutex_.lock();
                //TODO: here, update state variables with the tokens you read
		data_mutex_.unlock();
		
		//thread may need to sleep to allow time for other operations
		//usleep(1000);

		//clear parsed integers
		readint.clear();
            }
        }
};

#endif

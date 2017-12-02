// exampleApp.c

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <termios.h>
#include <time.h>
#include <sys/time.h>
#include <iostream>
#include <string>
#include <unistd.h>
#include "jetsonGPIO.h"
#include <sstream>

using namespace std;

std::string exec(char* cmd) {
    FILE* pipe = popen(cmd, "r");
    if (!pipe) return "ERROR";
    char buffer[128];
    std::string result = "";
    while(!feof(pipe)) {
        if(fgets(buffer, 128, pipe) != NULL)
            result += buffer;
    }
    pclose(pipe);
    return result;
}

int getkey() {
    int character;
    struct termios orig_term_attr;
    struct termios new_term_attr;

    /* set the terminal to raw mode */
    tcgetattr(fileno(stdin), &orig_term_attr);
    memcpy(&new_term_attr, &orig_term_attr, sizeof(struct termios));
    new_term_attr.c_lflag &= ~(ECHO|ICANON);
    new_term_attr.c_cc[VTIME] = 0;
    new_term_attr.c_cc[VMIN] = 0;
    tcsetattr(fileno(stdin), TCSANOW, &new_term_attr);

    /* read a character from the stdin stream without blocking */
    /*   returns EOF (-1) if no character is available */
    character = fgetc(stdin);

    /* restore the original terminal attributes */
    tcsetattr(fileno(stdin), TCSANOW, &orig_term_attr);

    return character;
}

int main(int argc, char *argv[]){

//    cout << "Starting Logs" << endl;
    if (argc != 3) {
        cout << ("Need 2 arguments: date and day") << endl;
	return 1;
    }

    std::string date;
    std::string day;
    date = argv[1];
    day = argv[2];

    std::ostringstream oss;

    int counter = 1;

    int greenLED = 398; // LED to acknowledge button press
    int roscoreLED = 298; // LED to signify roscore is running
    int rosbagLED = 388; // LED to signify rosbag is running

    char*  rosbagchk = "if ps -e | grep \"rosbag\" > \\dev\\null \n then echo 1 \n else echo 0 \n fi";
    char*  roscorechk = "if ps -e | grep \"roslaunch\" > \\dev\\null \n then echo 1 \n else echo 0 \n fi";
    cout<<atoi(exec(rosbagchk).c_str())<<endl;

    int pushButton = 481; // Input
    // Make the button and LEDs available in user space
    gpioUnexport(greenLED);
    gpioUnexport(roscoreLED);
    gpioUnexport(rosbagLED);

    gpioExport(pushButton);
    gpioExport(greenLED);
    gpioExport(roscoreLED);
    gpioExport(rosbagLED);

    gpioSetDirection(pushButton, inputPin);
    gpioSetDirection(greenLED, outputPin);
    gpioSetDirection(roscoreLED, outputPin);
    gpioSetDirection(rosbagLED, outputPin);
    // Reverse the button wiring; this is for when the button is wired
    // with a pull up resistor
    // gpioActiveLow(pushButton, true);

    // Wait for the push button to be pressed
    cout << "Please press the button! ESC key quits the program" << endl;

    unsigned int value = low;
    //    ledValue = low ; 
    bool running = false;
    // Turn off the LED
    gpioSetValue(greenLED,low) ; 
    while(getkey() != 27) {
        gpioGetValue(pushButton, &value) ;
        // Useful for debugging
        //cout << "Button " << value << endl;
        if (value==high) {
            gpioSetValue(greenLED, on);
            if (running) {
                system("./script_stop.sh ");
        		cout<<"Logs STOPPED"<<endl;
                running = false;
        		while(value==high) {
		        	gpioGetValue(pushButton, &value) ;
		        }
            } else {
                oss << "./script_run.sh " << date << " " << day << " " << counter;
                system(oss.str().c_str());
                cout<<"Logs STARTED" <<endl;
                running = true;
                counter += 1;
		        while(value==high) {
			        gpioGetValue(pushButton, &value) ;
        		}
            }
        } else {
            gpioSetValue(greenLED, off);
        }
	if(atoi(exec(roscorechk).c_str()))
            gpioSetValue(roscoreLED, on);
        else
            gpioSetValue(roscoreLED, off);
	if(atoi(exec(rosbagchk).c_str()))
            gpioSetValue(rosbagLED, on);
        else
            gpioSetValue(rosbagLED, off);
        usleep(1000); // sleep for a millisecond
    }

    cout << "Logging Finished" << endl;
    gpioUnexport(greenLED);     // unexport the LED */
    gpioUnexport(pushButton);      // unexport the push button
    return 0;
}



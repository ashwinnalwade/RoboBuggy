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
using namespace std;

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

    int redLED = 398;     // Ouput */
    int pushButton = 481; // Input
    // Make the button and led available in user space
    gpioUnexport(redLED);
    gpioExport(pushButton);
    gpioExport(redLED); 
    gpioSetDirection(pushButton,inputPin) ;
    gpioSetDirection(redLED,outputPin) ;
    // Reverse the button wiring; this is for when the button is wired
    // with a pull up resistor
    // gpioActiveLow(pushButton, true);


    // Flash the LED 5 times
    /*for(int i=0; i<5; i++){
        cout << "Setting the LED on" << endl;
        gpioSetValue(redLED, on);
        usleep(200000);         // on for 200ms
        cout << "Setting the LED off" << endl;
        gpioSetValue(redLED, off);
        usleep(200000);         // off for 200ms
    }*/

    // Wait for the push button to be pressed
    cout << "Please press the button! ESC key quits the program" << endl;

    unsigned int value = low;
    //    ledValue = low ; 
    bool running = false;
    // Turn off the LED
    gpioSetValue(redLED,low) ; 
    while(getkey() != 27) {
        gpioGetValue(pushButton, &value) ;
        // Useful for debugging
        //cout << "Button " << value << endl;
        if (value==high) {
            if (running) {
                system("./script_stop.sh");
		cout<<"Logs STOPPED"<<endl;
                running = false;
		gpioSetValue(redLED, off);
		while(value==high) {
			gpioGetValue(pushButton, &value) ;
		}
            } else {
                system("./script_run.sh");
                cout<<"Logs STARTED" <<endl;
                running = true;
		gpioSetValue(redLED, on);
		while(value==high) {
			gpioGetValue(pushButton, &value) ;
		}
            }
            // button is pressed ; turn the LED on
        }
        usleep(1000); // sleep for a millisecond
    }

    cout << "Logging Finished" << endl;
    gpioUnexport(redLED);     // unexport the LED */
    gpioUnexport(pushButton);      // unexport the push button
    return 0;
}



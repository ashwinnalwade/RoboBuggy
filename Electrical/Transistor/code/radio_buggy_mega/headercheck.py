""" headercheck.py: 
* Description: This is a python script to create a header file defining
*   all of the rbsm message headers.  It reads from the file rbsm_config.txt
*   and creates a headerfile, rbsm_config.h, to define each message type and 
*   error type. This is done to ensure that both high level and low level 
*   are using the same headers.
*
*  Note: To run this, the user must have python 2.7
*
*   By Sean Buckley
"""

import sys

def main():
    print("Now creating rbsm_config.h...")

    #If rbsmheaders.h already exists, it will be overwritten
    try:
        file = open("../lib_avr/rbserialmessages/rbsm_config.h", 'w')
    except:
        print("Error! Unable to create ../lib_avr/rbserialmessages/rbsm_config.h")
        sys.exit(1)

    settingsFile = None
    try:
        settingsFile = open("../../../../Common/rbsm_config.txt")
    except:
        print("Error! Unable to find ../../rbsm_config.txt\n")
        sys.exit(1)
 
    file.write("/* rbsm_config.h: A compilation-time created file containing message \n")
    file.write("headers and error codes for rbsm. Generated by headercheck.py. */\n\n")
    file.write("#ifndef RBSMHEADERS_H\n#define RBSMHEADERS_H\n\n")

    for lineNum, line in enumerate(settingsFile):
        definition = line.split(", ")

        #Skip any lines intended as comments
        if (line[0:2] == "//"):
            continue

        #Ignore empty lines and anything without enough information
        if (len(definition) < 2):
            print("Skipping line %d for insufficient length" %(lineNum + 1))
            continue

        parsedValue = -1

        try:
            parsedValue = int(definition[1])
        except:
            print("Error at line %d rbsm_headers.txt format! Is 'HEADER, VALUE' 'string, integer' ?" %(lineNum + 1))
            continue


        try:
            file.write("#define %s %d\n" %(definition[0], parsedValue))
        except:
            print("Error at line %d rbsm_headers.txt format! Is it 'HEADER, VALUE' ?" %(lineNum + 1))
            continue


    file.write("\n#endif")

    file.close()
    print("rbsm_config.h created.\n")

main()
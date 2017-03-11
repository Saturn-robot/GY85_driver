/*
 Name:		Test_DC_motor.ino
 Created:	7/22/2016 8:56:52 PM
 Author:	Xiao Ming
*/
#include "imu.h"
#include "command.h"

char incoming_chr;
char cmd;
byte index = 0;
byte arg = 0;
char argv1[16];
char argv2[16];
char argv3[16];
char argv4[16];
// The arguments converted to integers
long arg1;
long arg2;
long arg3;
long arg4;

/* Clear the current command parameters */
void resetCommand() {
	cmd = 0;
	memset(argv1, 0, sizeof(argv1));
	memset(argv2, 0, sizeof(argv2));
	memset(argv3, 0, sizeof(argv3));
	memset(argv4, 0, sizeof(argv4));
	arg = 0;
	arg1 = 0;
	arg2 = 0;
	arg3 = 0;
	arg4 = 0;
	index = 0;
}

int runCommand() {
	byte i;
	arg1 = atoi(argv1);
        arg2 = atoi(argv2);
        arg3 = atoi(argv3);
        arg4 = atoi(argv4);

	switch (cmd) {
		case READ_IMU:
			imu_data = readIMU();
			Serial.print(imu_data.ax);
			Serial.print(F(" "));
			Serial.print(imu_data.ay);
			Serial.print(F(" "));
                        Serial.print(imu_data.az);
                        Serial.print(F(" "));
                        Serial.print(imu_data.gx);
                        Serial.print(F(" "));
                        Serial.print(imu_data.gy);
                        Serial.print(F(" "));
                        Serial.print(imu_data.gz);
                        Serial.print(F(" "));
                        Serial.print(imu_data.mx);
                        Serial.print(F(" "));
                        Serial.print(imu_data.my);
                        Serial.print(F(" "));
                        Serial.print(imu_data.mz);
                        Serial.print(F(" "));
                        Serial.print(imu_data.roll);
                        Serial.print(F(" "));
                        Serial.print(imu_data.pitch);
                        Serial.print(F(" "));
                        Serial.println(imu_data.uh);
		             break;
		             default:
			Serial.println(F("Invalid Command"));
			break;
	}
}
// the setup function runs once when you press reset or power the board
void setup() {
	Serial.begin(57600);
        initIMU();
}

// the loop function runs over and over again until power down or reset
void loop() {
	while (Serial.available() > 0) {
		incoming_chr = Serial.read();
		// run command according to arguements until finding a CR
		if (incoming_chr == 13) {
			if (arg == 1) argv1[index] = NULL;
			else if (arg == 2) argv2[index] = NULL;
			runCommand();
			resetCommand();
		}
		// use spaces to delimit parts of the command
		else if (incoming_chr == ' ') {
				if (arg == 0) arg = 1;
				else if (arg == 1) {
					argv1[index] = NULL;
					arg = 2;
					index = 0;
				}
		}
		else {
			if (arg == 0) {
				// the first arg is the number of wheels
				cmd = incoming_chr;
				//Serial.println("get it!");
			}
			else if (arg == 1) {
				argv1[index] = incoming_chr;
				index++;
			}
			else if (arg == 2) {
				argv2[index] = incoming_chr;
				index++;
			}
			}
		}
	}

#include "Arduino.h"
int message = 0;     //  This will hold one byte of the serial message
int redLEDPin = 13;   //  What pin is the red LED connected to?
int redLED = 0;          //  The value/brightness of the LED, can be 0-255

void setup() {
	Serial.begin(115200);  //set serial to 115200 baud rate
}

void loop() {
	if (Serial.available() > 0) { //  Check if there is a new message
		message = Serial.read();    //  Put the serial input into the message

		if (message == 'R') {  //  If a capitol R is received...
			redLED = 255;       //  Set redLED to 255 (on)
		}
		if (message == 'r') {  //  If a lowercase r is received...
			redLED = 0;         //  Set redLED to 0 (off)
		}

	}
	analogWrite(redLEDPin, redLED);  //  Write an analog value between 0-255
}

#include "Arduino.h"
/*
 Coded by Marjan Olesch
 Sketch from Insctructables.com
 Open source - do what you want with this code!
 */
#include <Servo.h>

int lastSpeed = 0;
int Speed = 0;
int value = 1060; // set values you need to zero

Servo firstESC, secondESC, thirdESC, fourESC; //Create as much as Servoobject you want. You can controll 2 or more Servos at the same time

void setup() {

	firstESC.attach(9);    // attached to pin 9 I just do this with 1 Servo
	secondESC.attach(10);
	thirdESC.attach(11);
	fourESC.attach(12);
	Serial.begin(115200);    // start serial at 115200 baud

}

void loop() {

	//First connect your ESC WITHOUT Arming. Then Open Serial and follo Instructions
	firstESC.writeMicroseconds(value);
	secondESC.writeMicroseconds(value);
	thirdESC.writeMicroseconds(value);
	fourESC.writeMicroseconds(value);
	// Afro ESC ARM 1060 - Start 1127 (1130) - Highest 1860
	lastSpeed = Speed;
	if (Serial.available()) {
		Speed = Serial.parseInt();    // Parse an Integer from Serial
		if (Speed == 0)
			value = 1060;
		else
			value = map(Speed, 1, 100, 1130, 1860);
	}
	if (lastSpeed != Speed)
		Serial.println(value);
}

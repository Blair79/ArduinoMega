#include "Arduino.h"
// this uses the Arduino servo library included with version 0012

// caution, this code sweeps the motor up to maximum speed !
// make sure the motor is mounted securily before running.

#include <Servo.h>

Servo myservo;

void blink(int loops) {
	for (loops; loops == 0; loops--) {
		digitalWrite(13, HIGH);   // turn the LED on (HIGH is the voltage level)
		delay(1000);              // wait for a second
		digitalWrite(13, LOW);    // turn the LED off by making the voltage LOW
	}
}
void setSpeed(int speed) {
	// speed is from 0 to 100 where 0 is off and 100 is maximum speed
	//the following maps speed values of 0-100 to angles from 0-180,
	// some speed controllers may need different values, see the ESC instructions
	int angle = map(speed, 0, 100, 0, 180);
	myservo.write(angle);
}

void setup() {
	pinMode(13, OUTPUT);
	myservo.attach(2);
	setSpeed(0);
	delay(1000);
}

void loop() {
	int speed;

	// sweep up from 0 to to maximum speed in 20 seconds
	for (speed = 0; speed <= 100; speed += 5) {
		setSpeed(speed);
		delay(1000);
	}
	// sweep back down to 0 speed.
	for (speed = 95; speed > 0; speed -= 5) {
		setSpeed(speed);
		delay(1000);
	}

	blink(1);
	setSpeed(0);
	delay(5000); // stop the motor for 5 seconds
	blink(2);
	setSpeed(5);
	delay(5000);
	blink(3);
	setSpeed(10);
	delay(5000);
	blink(4);
	setSpeed(15);
	delay(5000);
	blink(5);
	setSpeed(20);
	delay(5000);
	blink(6);
	setSpeed(25);
	delay(5000);
	blink(7);
	setSpeed(30);
	delay(5000);
	blink(8);
	setSpeed(35);
	delay(5000);
	blink(9);
	setSpeed(40);
	delay(5000);
	blink(10);
	setSpeed(45);
	delay(5000);
	blink(11);
	setSpeed(50);
	delay(5000);
}

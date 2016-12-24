#include "Arduino.h"
//The setup function is called once at startup of the sketch
/*
 Blink
 Turns on an LED on for one second, then off for one second, repeatedly.
 This example code is in the public domain.
 */

// Pin 13 has an LED connected on most Arduino boards.
int ledState = LOW;
int led = 13;
int sensorPin = A0;    // select the input pin for the potentiometer
int sensorValue = 0;  // variable to store the value coming from the sensor

// the setup routine runs once when you press reset:
void setup() {
	// initialize the digital pin as an output.
	pinMode(led, OUTPUT);
	Serial2.begin(115200); //This initialices the USB as a serial port
}
// the loop routine runs over and over again forever:
void loop() {

	char incomingByte = (char) Serial2.read();
	if (incomingByte == '1') {
		digitalWrite(led, HIGH);
		delay(5000);
		digitalWrite(led, LOW);
	}

	sensorValue = analogRead(sensorPin); //Reads the voltage of the resistor.

	Serial2.println(sensorValue);

}

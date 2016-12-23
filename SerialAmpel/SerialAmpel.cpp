#include "Arduino.h"
char value;
int ledpin = 13;
int grpin = 12;
int gepin = 11;
int ropin = 10;

void setup() {
	pinMode(ledpin, OUTPUT);
	Serial.begin(115200);
}

void loop() {
	if (Serial.available()) {
		;
	}

	value = Serial.read();
	digitalWrite(ledpin, HIGH);
	Serial.println("LED on");
	delay(50);
	digitalWrite(ledpin, LOW);
	Serial.println("LED off");

	if (value == '0') {
		digitalWrite(ledpin, HIGH);
		Serial.println("LED on");
		delay(5000);
		digitalWrite(ledpin, LOW);
		Serial.println("LED off");
	}

	if (value == '1') {
		analogWrite(grpin, 255);
		Serial.println("Gruene LED on");
		analogWrite(gepin, 0);
		analogWrite(ropin, 0);
		delay(5000);
		analogWrite(grpin, 0);
	}

	if (value == '2') {
		analogWrite(gepin, 255);
		Serial.println("Gelbe LED on");
		analogWrite(grpin, 0);
		analogWrite(ropin, 0);
		delay(5000);
		analogWrite(gepin, 0);
	}

	if (value == '3') {
		analogWrite(ropin, 255);
		Serial.println("Rote LED on");
		analogWrite(grpin, 0);
		analogWrite(gepin, 0);
		delay(5000);
		analogWrite(ropin, 0);
	}

	if (value != '1') {
		for (int fadeValue = 0; fadeValue <= 255; fadeValue += 10) {
			// sets the value (range from 0 to 255):
			analogWrite(grpin, fadeValue);
			// wait for 30 milliseconds to see the dimming effect
			delay(20);
		}

		// fade out from max to min in increments of 5 points:
		for (int fadeValue = 255; fadeValue >= 0; fadeValue -= 10) {
			// sets the value (range from 0 to 255):
			analogWrite(grpin, fadeValue);
			// wait for 30 milliseconds to see the dimming effect
			delay(20);
			analogWrite(grpin, 0);
		}
	}

	if (value != '2') {
		for (int fadeValue = 0; fadeValue <= 255; fadeValue += 10) {
			// sets the value (range from 0 to 255):
			analogWrite(gepin, fadeValue);
			// wait for 30 milliseconds to see the dimming effect
			delay(20);
		}

		// fade out from max to min in increments of 5 points:
		for (int fadeValue = 255; fadeValue >= 0; fadeValue -= 10) {
			// sets the value (range from 0 to 255):
			analogWrite(gepin, fadeValue);
			// wait for 30 milliseconds to see the dimming effect
			delay(20);
			analogWrite(gepin, 0);
		}
	}

	if (value != '3') {
		for (int fadeValue = 0; fadeValue <= 255; fadeValue += 10) {
			// sets the value (range from 0 to 255):
			analogWrite(ropin, fadeValue);
			// wait for 30 milliseconds to see the dimming effect
			delay(20);
		}

		// fade out from max to min in increments of 5 points:
		for (int fadeValue = 255; fadeValue >= 0; fadeValue -= 10) {
			// sets the value (range from 0 to 255):
			analogWrite(ropin, fadeValue);

			// wait for 30 milliseconds to see the dimming effect
			delay(20);
			analogWrite(ropin, 0);
		}
	}
}


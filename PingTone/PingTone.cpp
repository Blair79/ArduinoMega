#include "Arduino.h"
//The setup function is called once at startup of the sketch
#include "pitches.h"

int pingPin = 13;
int inPin = 12;

int notes[] = {
NOTE_A4, NOTE_B4, NOTE_C3, NOTE_A3, NOTE_D8 };

void setup() {
	Serial.begin(115200);
}

long microsecondsToCentimeters(long microseconds) {
// The speed of sound is 340 m/s or 29 microseconds per centimeter.
// The ping travels out and back, so to find the distance of the
// object we take half of the distance travelled.
	return microseconds / 29 / 2;

}


long microsecondsToInches(long microseconds) {
// According to Parallax's datasheet for the PING))), there are
// 73.746 microseconds per inch (i.e. sound travels at 1130 feet per
// second).  This gives the distance travelled by the ping, outbound
// and return, so we divide by 2 to get the distance of the obstacle.
	return microseconds / 74 / 2;
}

void loop() {
// establish variables for duration of the ping,
// and the distance result in inches and centimeters:
	long duration, inches, cm;

// The PING))) is triggered by a HIGH pulse of 2 or more microseconds.
// Give a short LOW pulse beforehand to ensure a clean HIGH pulse:
	pinMode(pingPin, OUTPUT);
	digitalWrite(pingPin, LOW);
	delayMicroseconds(2);
	digitalWrite(pingPin, HIGH);
	delayMicroseconds(10);
	digitalWrite(pingPin, LOW);

// The same pin is used to read the signal from the PING))): a HIGH
// pulse whose duration is the time (in microseconds) from the sending
// of the ping to the reception of its echo off of an object.
	pinMode(inPin, INPUT);
	duration = pulseIn(inPin, HIGH);

// convert the time into a distance
	inches = microsecondsToInches(duration);
	cm = microsecondsToCentimeters(duration);

	Serial.println(cm, DEC);

	if (cm < 3) {
		// play the note corresponding to this sensor:
		tone(8, notes[4], 30);
	}

	if ((cm >= 3) && (cm <= 9)) {
		// play the note corresponding to this sensor:
		tone(8, notes[0], 30);
	}

	if ((cm >= 10) && (cm <= 19)) {
		// play the note corresponding to this sensor:
		tone(8, notes[1], 30);
	}
	if ((cm >= 20) && (cm <= 29)) {
		// play the note corresponding to this sensor:
		tone(8, notes[2], 30);
	}

	if ((cm >= 30) && (cm <= 49)) {
		// play the note corresponding to this sensor:
		tone(8, notes[3], 30);
	}

	delay(100);
}




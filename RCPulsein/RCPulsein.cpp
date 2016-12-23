#include "Arduino.h"
//Includes Servo Library and "Tones"
#include <Servo.h>
#include "pitches.h"

//Debugmode define or undef
#undef DEBUG

//Joystick Preferences
#define NUM_BUTTONS	40
#define NUM_AXES	8	       // 8 axes, X, Y, Z, etc

#define Speaker         13

int receiverpin[] = { A0, A1, A2, A3, A4, A5, A6, A7 };
unsigned long pinsignal[NUM_AXES];
int errormap[NUM_AXES][2];
int automap[NUM_AXES][2];

int melody[] = {
NOTE_C4, NOTE_G3, NOTE_G3, NOTE_A3, NOTE_G3, 0, NOTE_B3, NOTE_C4 };
int noteDurations[] = { 4, 8, 8, 4, 4, 4, 4, 4 };

typedef struct joyReport_t {
	int16_t axis[NUM_AXES];
	uint8_t button[(NUM_BUTTONS + 7) / 8]; // 8 buttons per byte
} joyReport_t;
joyReport_t joyReport;

int ServoValue;
Servo myservo1;
Servo myservo2;

void playTone() {
	for (uint8_t thisNote = 0; thisNote < 8; thisNote++) {

		// to calculate the note duration, take one second
		// divided by the note type.
		//e.g. quarter note = 1000 / 4, eighth note = 1000/8, etc.
		int noteDuration = 1000 / noteDurations[thisNote];
		tone(Speaker, melody[thisNote], noteDuration);

		// to distinguish the notes, set a minimum time between them.
		// the note's duration + 30% seems to work well:
		int pauseBetweenNotes = noteDuration * 1.30;
		delay(pauseBetweenNotes);
		// stop the tone playing:
		noTone(Speaker);
	}
}
void setup() {
	for (uint8_t i = 0; i < NUM_AXES; i++) {
		pinMode(receiverpin[i], INPUT);
	}

	Serial.begin(115200);

	//Disable Speaker in Debugmode
#ifndef DEBUG
	playTone();
#endif

	delay(200);

	for (uint8_t i = 0; i < NUM_AXES; i++) {
		joyReport.axis[i] = 0;
	}
	for (uint8_t i = 0; i < sizeof(joyReport.button); i++) {
		joyReport.button[i] = 0;
	}

	//  myservo1.attach(4);
	//  myservo2.attach(5);
}

#ifndef DEBUG
#endif

// Send an HID report to the USB interface
void sendJoyReport(struct joyReport_t *report) {
#ifndef DEBUG
	//for (int i=0; i<NUM_AXES; i++)if (pinsignal[i] <= 0);
	Serial.write((uint8_t *) report, sizeof(joyReport_t));
#else
	// dump human readable output for debugging
	for (uint8_t i=0; i<NUM_AXES; i++) {
		Serial.print(automap[i][0]);
		Serial.print(" ");
		Serial.print(automap[i][1]);
		Serial.print(" ");
		Serial.print("axis[");
		Serial.print(i);
		Serial.print("]= ");
		Serial.print(report->axis[i]);
		Serial.print(" ");
	}

	Serial.println();
	/*for (uint8_t ind=0; ind<NUM_BUTTONS/8; ind++) {
	 Serial.print("button[");
	 Serial.print(ind);
	 Serial.print("]= ");
	 Serial.print(report->button[ind], HEX);
	 Serial.print(" ");
	 }
	 Serial.println();*/
#endif
}

// turn a button on
void setButton(joyReport_t *joy, uint8_t button) {
	uint8_t index = button / 8;
	uint8_t bit = button - 8 * index;

	joy->button[index] |= 1 << bit;
}

// turn a button off
void clearButton(joyReport_t *joy, uint8_t button) {
	uint8_t index = button / 8;
	uint8_t bit = button - 8 * index;

	joy->button[index] &= ~(1 << bit);
}

/*int Mittelstellung(int ServoValue,int MittelValue)
 {
 if (ServoValue > (MittelValue - 5) && ServoValue < (MittelValue + 5))
 return MittelValue;
 else
 return ServoValue;
 }*/

//mainloop
void loop() {
	memcpy(automap, errormap, sizeof(errormap));
	for (uint8_t i = 0; i < NUM_AXES; i++) {
		pinsignal[i] = pulseIn(receiverpin[i], HIGH, 20000);

		if ((pinsignal[i] > 1000) && (pinsignal[i] < 2000)) {
			if (automap[i][0] == 0)
				automap[i][0] = pinsignal[i];
			else if (pinsignal[i] < automap[i][0])
				automap[i][0] = pinsignal[i];

			if (pinsignal[i] > automap[i][1])
				automap[i][1] = pinsignal[i];

			//joyReport.axis[i] = pinsignal[i];
			if (i > 3)
				joyReport.axis[i] = map(pinsignal[i], automap[i - 4][0],
						automap[i - 4][1], -10000, 10000);
			else
				joyReport.axis[i] = map(pinsignal[i], automap[i][0],
						automap[i][1], -10000, 10000);
		} else {
			//Errorhandling
			tone(Speaker, NOTE_A1, 1);
			noTone(Speaker);
			//automap[i][0] = 0;
			//automap[i][1] = 0;
			joyReport.axis[i] = 0;
		}
	}
	int j = 0;
	for (uint8_t i = 0; i < NUM_AXES; i++)
		j = joyReport.axis[i] + joyReport.axis[i - 1];
	if (j != 0)
		memcpy(errormap, automap, sizeof(automap));
	else {
		for (uint8_t i = 0; i < NUM_AXES; i++) {
			automap[i][0] = 0;
			automap[i][1] = 0;
			joyReport.axis[i] = 0;
		}
	}

	/*durse = pulseIn(rse, LOW, 40000);
	 //joyReport.axis[0] = durse;
	 joyReport.axis[0] = map(durse, 19650, 20400, -10000, 10000);
	 //ServoValue = map(durse, 1100, 1850, 0, 255);
	 //Serial.print(ServoValue, DEC);
	 //Serial.print(" ");
	 durwa = pulseIn(rwa, LOW, 40000);
	 ServoValue = map(durwa, 19730, 20500, 0, 180);
	 ServoValue = map(ServoValue, 0, 180, 45, 124);
	 ServoValue = Mittelstellung(ServoValue, 78);
	 //ServoPrint(ServoValue);
	 if (ServoValue != 78)myservo2.write(ServoValue);
	 joyReport.axis[1] = map(durwa, 19730, 20500, -10000, 10000);
	 //else
	 //myservo2.detach();
	 //delay(10);
	 //Serial.print(Mittelstellung(ServoValue, 90));

	 dulse = pulseIn(lse, LOW, 40000);
	 ServoValue = map(dulse, 19770, 20480, 0, 180);
	 ServoValue = map(ServoValue, 0, 180, 53, 162);
	 ServoValue = Mittelstellung(ServoValue, 98);
	 //ServoPrint(ServoValue);
	 //Serial.println("");
	 if (ServoValue != 98)myservo1.write(ServoValue);
	 joyReport.axis[2] = map(dulse, 19770, 20480, -10000, 10000);
	 //else
	 //myservo1.detach();
	 //Serial.println(Mittelstellung(ServoValue, 80));
	 //delay(10);
	 //Serial.print(" ");
	 dulwa = pulseIn(lwa, LOW, 40000);
	 //joyReport.axis[3] = dulwa;
	 joyReport.axis[3] = map(dulwa, 19680, 20430, -10000, 10000);
	 //ServoValue = map(dulwa, 1100, 1850, 0, 255);
	 //Serial.println(ServoValue, DEC);

	 urse = pulseIn(ruse, LOW, 40000);
	 //Serial.print (SignalCheck(urse));
	 joyReport.axis[4] = map(urse, 19680, 20430, -10000, 10000);
	 urwa = pulseIn(ruwa, LOW, 40000);
	 joyReport.axis[5] = map(urwa, 19680, 20430, -10000, 10000);
	 ulse = pulseIn(luse, LOW, 40000);
	 joyReport.axis[6] = map(ulse, 19680, 20430, -10000, 10000);
	 ulwa = pulseIn(luwa, LOW, 40000);
	 joyReport.axis[7] = map(ulwa, 19680, 20430, -10000, 10000);*/
	sendJoyReport(&joyReport);
}


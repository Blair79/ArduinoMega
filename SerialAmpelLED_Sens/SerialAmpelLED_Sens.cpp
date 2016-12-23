#include "Arduino.h"
#include "pitches.h"
#define LED_Sensor_NEG 8
#define LED_Sensor_POS 7
#define BoardPin 13
#define GPin 12
#define YPin 11
#define RPin 10

char Value;

int melody[] = {
NOTE_C4, NOTE_G3, NOTE_G3, NOTE_A3, NOTE_G3, 0, NOTE_B3, NOTE_C4 };

// note durations: 4 = quarter note, 8 = eighth note, etc.:
int noteDurations[] = { 4, 8, 8, 4, 4, 4, 4, 4 };

void playMelody() {
	Serial.println("spiele Melodie");
	for (int thisNote = 0; thisNote < 8; thisNote++) {

		// to calculate the note duration, take one second
		// divided by the note type.
		//e.g. quarter note = 1000 / 4, eighth note = 1000/8, etc.
		int noteDuration = 1000 / noteDurations[thisNote];
		tone(6, melody[thisNote], noteDuration);

		// to distinguish the notes, set a minimum time between them.
		// the note's duration + 30% seems to work well:
		int pauseBetweenNotes = noteDuration * 1.30;
		delay(pauseBetweenNotes);
		// stop the tone playing:
		noTone(6);
	}
}

void setup() {
	pinMode(BoardPin, OUTPUT);
	pinMode(LED_Sensor_POS, LOW);
	Serial.begin(115200);
	playMelody();
}

String sendText(int Pin, boolean State) {
	String OutputText = "";
	if (Pin == 13 && State == true)
		OutputText = "BoardLED on";
	if (Pin == 13 && State == false)
		OutputText = "BoardLED off";
	if (Pin == 12 && State == true)
		OutputText = "GrueneLED on";
	if (Pin == 12 && State == false)
		OutputText = "GrueneLED off";
	if (Pin == 11 && State == true)
		OutputText = "GelbeLED on";
	if (Pin == 11 && State == false)
		OutputText = "GelbeLED off";
	if (Pin == 10 && State == true)
		OutputText = "RoteLED on";
	if (Pin == 10 && State == false)
		OutputText = "RoteLED off";
	return OutputText;
}



void blinkBoardLED(int delayTime) {
	digitalWrite(BoardPin, HIGH);
	Serial.println(sendText(BoardPin, true));
	delay(delayTime);
	digitalWrite(BoardPin, LOW);
	Serial.println(sendText(BoardPin, false));
}


void delayBlink(int OnPin, int OffPin1, int OffPin2) {
	Serial.println(sendText(OnPin, true));
	analogWrite(OnPin, 255);
	analogWrite(OffPin1, 0);
	analogWrite(OffPin2, 0);
	delay(5000);
	Serial.println(sendText(OnPin, false));
	analogWrite(OnPin, 0);
}


void fadeLED(int Pin) {
	Serial.println(sendText(Pin, true));
	for (int fadeValue = 0; fadeValue <= 255; fadeValue += 10) {
		// sets the value (range from 0 to 255):
		analogWrite(Pin, fadeValue);
		// wait for 30 milliseconds to see the dimming effect
		delay(20);
	}

	Serial.println(sendText(Pin, false));
	// fade out from max to min in increments of 5 points:
	for (int fadeValue = 255; fadeValue >= 0; fadeValue -= 10) {
		// sets the value (range from 0 to 255):
		analogWrite(Pin, fadeValue);
		// wait for 30 milliseconds to see the dimming effect
		delay(20);
		analogWrite(Pin, 0);
	}
}


int measureDarkness() {
	pinMode(LED_Sensor_NEG, OUTPUT);
	digitalWrite(LED_Sensor_NEG, HIGH);

	long Darkness = 0;
	int OutputVal = 0;
	pinMode(LED_Sensor_NEG, INPUT);
	digitalWrite(LED_Sensor_NEG, LOW);
	while ((digitalRead(LED_Sensor_NEG) != 0) && Darkness < 80000) {
		Darkness++;
	}
	OutputVal = Darkness / 80;
	return OutputVal;
}


void loop() {
	if (Serial.available()) {
		;
	}

	Value = Serial.read();
	blinkBoardLED(50);

	if (Value == '0')
		blinkBoardLED(5000);
	if (Value == '1')
		delayBlink(GPin, YPin, RPin);
	if (Value == '2')
		delayBlink(YPin, GPin, RPin);
	if (Value == '3')
		delayBlink(RPin, YPin, GPin);
	if (Value == '4')
		playMelody();
	if (Value != '1')
		fadeLED(GPin);
	if (Value != '2')
		fadeLED(YPin);
	if (Value != '3')
		fadeLED(RPin);

	Serial.println(measureDarkness());
}


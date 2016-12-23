#include "Arduino.h"
/*
 Arduino PPM Decoder 1.2
 This sketch decodes PPM signals from an RC receiver. Input should be in the form of a PPM Stream
 Outputs are scaled to 1-100 and then printed to the Serial Monitor

 For details on using this sketch see link below
 http://projectsfromtech.blogspot.com/2013/11/arduino-ppm-decoder-decoding-rc.html
 For details on building a PPM Encoder to create an Arduino readable PPM stream see link below
 http://projectsfromtech.blogspot.com/2013/11/homemade-ppm-encoder.html


 Last edited: 11/16/2013
 Matthew
 http://projectsfromtech.blogspot.com/

 */
#include <Servo.h>

const int NumberOfChannels = 8;          // Change to match number of channels
const byte InputPin = 2;  // Pin associated with interrupt 2 on Arduino Mega2560
const int FrameSpace = 2000;   // 8 ms

volatile long Current_Time;
volatile long Last_Spike;

volatile byte Current_Channel = 0;
volatile int Spike_Length[NumberOfChannels + 1];
volatile int LastChannelFlag = false;

int lastSpeed = 0;
int Speed = 0;
int ESC_value = 1060;
Servo firstESC, secondESC, thirdESC, fourESC;

void Spike() {
	Current_Time = micros();
	Spike_Length[Current_Channel] = Current_Time - Last_Spike;
	if (Spike_Length[Current_Channel] > FrameSpace)
		Current_Channel = 0; // oops, you were actually in the frame space --- need to add correction

	Last_Spike = Current_Time; // Set the current time as the previous time to the next one.
	Current_Channel = Current_Channel + 1;       // Reading the next channel now

	if (Current_Channel == NumberOfChannels) // Special case. Must wait for it to fall manually.
			{
		LastChannelFlag = true;
	}
}
void initESC() {
	Speed = 0;
	ESC_value = 1060;
	firstESC.attach(9);    // attached to pin 9 I just do this with 1 Servo
	secondESC.attach(10);
	thirdESC.attach(11);
	fourESC.attach(12);
}

void LastChannel() {

	while (digitalRead(InputPin) == HIGH)
		;
	Current_Time = micros();  // Now it has fallen
	Spike_Length[Current_Channel] = Current_Time - Last_Spike;
	Current_Channel = 0;
	Last_Spike = Current_Time;
	LastChannelFlag = false;
}

void Display() {
	for (byte x = 1; x <= 8; x++) {
		Serial.print("Ch. ");
		Serial.print(x);
		Serial.print(": ");
		Serial.print(Spike_Length[x]);
		Serial.print(": ");
		if (Spike_Length[x] < 1155 && x == 3) {
			Spike_Length[x] = 1140;
		}
		if (x == 3) {
			Speed = map(Spike_Length[x], 1140, 1900, 0, 100);
			Serial.print(Speed);
		};
		Serial.print(map(Spike_Length[x], 1140, 1900, 0, 100)); // Mapping values may need to be changed depending on receiver
		Serial.print("      ");
	}
	Serial.print(Speed);
	Serial.println();
}
void setup() {
	delay(100);
	attachInterrupt(1, Spike, RISING);
	Last_Spike = micros();
	Serial.begin(115200);
	Serial.println("Setup");
	initESC();
}

void loop() {
	firstESC.writeMicroseconds(ESC_value);
	secondESC.writeMicroseconds(ESC_value);
	thirdESC.writeMicroseconds(ESC_value);
	fourESC.writeMicroseconds(ESC_value);
	lastSpeed = Speed;
	if (LastChannelFlag == true)   // If we are on the last Spike
	{
		LastChannel();               // Get the last pulse value
		Display();                 // Display all channels on the Serial Monitor
	}

	/*if (Serial.available()) {
	 Speed = Serial.parseInt();    // Parse an Integer from Serial

	 }*/
	if (Speed == 0)
		ESC_value = 1060;
	else
		ESC_value = map(Speed, 1, 100, 1130, 1860);

}  //end loop()

//Stores the length of the spike in Spike_Length. Framespace length in stored in SpikeLength[0]
// end Spike()

// Prints values to Serial Monitor

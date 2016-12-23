#include "Arduino.h"
void setup() {

	Serial.begin(115200);

}

void loop() {

	Serial.println("0");
	delay(100);

	Serial.println("1");
	delay(100);
	Serial.println("30");
	delay(100);
}

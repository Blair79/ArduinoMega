#include "Arduino.h"
#include <Servo.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_LSM303_U.h>
#include <Adafruit_BMP085_U.h>
#include <Adafruit_L3GD20_U.h>
#include <Adafruit_Simple_AHRS.h>
#include "Kalman.h"

const int NumberOfChannels = 8;          // Change to match number of channels
const byte InputPin = 2;  // Pin associated with interrupt 2 on Arduino Mega2560
const int FrameSpace = 6000;   // 8 ms

volatile long Current_Time;
volatile long Last_Spike;

volatile byte Current_Channel = 0;
volatile int Spike_Length[NumberOfChannels + 1];
volatile int LastChannelFlag = false;
//----------------
int lastSpeed = 0;
int Speed = 0;
int Gier = 0;
int Nick = 0;
int Roll = 0;
int ESC_value = 1060;
int firstESC_value = 1060;
int secondESC_value = 1060;
int thirdESC_value = 1060;
int fourESC_value = 1060;
Servo firstESC, secondESC, thirdESC, fourESC;


Adafruit_LSM303_Accel_Unified acc = Adafruit_LSM303_Accel_Unified(30301);
Adafruit_LSM303_Mag_Unified mag = Adafruit_LSM303_Mag_Unified(30302);
Adafruit_Simple_AHRS          dof(&acc, &mag);
Adafruit_BMP085_Unified bmp = Adafruit_BMP085_Unified(18001);
Adafruit_L3GD20_Unified gyr = Adafruit_L3GD20_Unified(20);

Kalman kalmanX; // Create the Kalman instances
Kalman kalmanY;

/* IMU Data */
double accX, accY, accZ;
double gyroX, gyroY, gyroZ;
int tempRaw;
long timer, timer2;

double gyroXangle, gyroYangle; // Angle calculate using the gyro only
double compRoll, compPitch; // Calculated angle using a complementary filter
double kalRoll, kalPitch; // Calculated angle using a Kalman filter

float seaLevelPressure = 1015.7; //SENSORS_PRESSURE_SEALEVELHPA;
float alt = 109, vario = 0;

void initSensors() {
	if (!acc.begin()) {
		/* There was a problem detecting the LSM303 ... check your connections */
		Serial.print(F("F:LSM303"));
		while (1)
			;
	}
	if (!mag.begin()) {
		/* There was a problem detecting the LSM303 ... check your connections */
		Serial.print("F:LSM303");
		while (1)
			;
	}
	if (!bmp.begin()) {
		/* There was a problem detecting the BMP180 ... check your connections */
		Serial.print("F:BMP180");
		while (1)
			;
	}
	if (!gyr.begin()) {
		/* There was a problem detecting the BMP180 ... check your connections */
		Serial.print("F:L3GD20");
		while (1)
			;
	}

	sensors_event_t acc_event;

	accX = acc_event.acceleration.x;
	accY = acc_event.acceleration.y;
	accZ = acc_event.acceleration.z;

#ifdef RESTRICT_PITCH // Eq. 25 and 26
	double roll = atan2(accY, accZ) * RAD_TO_DEG;
	double pitch = atan(-accX / sqrt(accY * accY + accZ * accZ)) * RAD_TO_DEG;
#else // Eq. 28 and 29
	double roll = atan(accY / sqrt(accX * accX + accZ * accZ)) * RAD_TO_DEG;
	double pitch = atan2(-accX, accZ) * RAD_TO_DEG;
#endif

	kalmanX.setAngle(roll); // Set starting angle
	kalmanY.setAngle(pitch);
	gyroXangle = roll;
	gyroYangle = pitch;
	compRoll = roll;
	compPitch = pitch;

	delay(500);
	timer = micros();
	timer2 = timer;
}

void initESC() {
	Speed = 0;
	ESC_value = 1060;
	firstESC.attach(9);    // attached to pin 9 I just do this with 1 Servo
	secondESC.attach(10);
	thirdESC.attach(11);
	fourESC.attach(12);
}

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
}  // end Spike()

void setup(void) {
	Last_Spike = micros();
	attachInterrupt(0, Spike, RISING);
	//---------------
	pinMode(13, OUTPUT);
	Serial.begin(115200);

	/* Initialise the sensors */
	initSensors();

	/* Initialise the ESC */
	initESC();
	delay(100);

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

// Prints values to Serial Monitor
void setPWM() {
	for (byte x = 1; x <= 8; x++) {
		/*Serial.print("Ch. ");
		 Serial.print(x);
		 Serial.print(": ");
		 Serial.print(Spike_Length[x]);
		 Serial.print(": ");*/
		if (Spike_Length[x] < 1174 && x == 3) {
			Spike_Length[x] = 1140;
		}
		if (x == 1)
			Gier = map(Spike_Length[x], 1140, 1860, -50, 50);
		if (x == 2)
			Nick = map(Spike_Length[x], 1140, 1860, -50, 50);
		if (x == 3)
			Speed = map(Spike_Length[x], 1140, 1860, 0, 100);
		if (x == 4)
			Roll = map(Spike_Length[x], 1140, 1860, -50, 50);
		/*Serial.print(map(Spike_Length[x], 1140, 1860, 0, 100));      // Mapping values may need to be changed depending on receiver
		 Serial.print("      ");*/
	}
	//Serial.println();
}

void loop(void) {

	firstESC.writeMicroseconds(firstESC_value);
	secondESC.writeMicroseconds(secondESC_value);
	thirdESC.writeMicroseconds(thirdESC_value);
	fourESC.writeMicroseconds(fourESC_value);

	if (LastChannelFlag == true)   // If we are on the last Spike
	{
		LastChannel();               // Get the last pulse value
		setPWM();                  // Display all channels on the Serial Monitor
	}

	if (Serial.available()) {
		Speed = Serial.parseInt();    // Parse an Integer from Serial
		if (Speed == 0) {
			ESC_value = 1060;
			firstESC_value = ESC_value;
			secondESC_value = ESC_value;
			thirdESC_value = ESC_value;
			fourESC_value = ESC_value;
		}
		if (Speed > 0 && Speed < 100)
			ESC_value = map(Speed, 1, 100, 1130, 1860);
	}

	if (Speed == 0) {
		ESC_value = 1060;
		firstESC_value = ESC_value;
		secondESC_value = ESC_value;
		thirdESC_value = ESC_value;
		fourESC_value = ESC_value;
	}
	if (Speed > 0 && Speed < 100) {
		ESC_value = map(Speed, 1, 100, 1130, 1860);
		if (Gier >= -50 && Gier <= 50) {
			//Serial.println(Gier);
		}

		if (Nick >= -50 && Nick <= 50) {
			//Serial.println(Gier);
		}

		if (Roll >= -50 && Roll <= 50) {
			//Serial.println(Gier);
		}
	}

	sensors_event_t acc_event;
	sensors_event_t mag_event;
	sensors_event_t bmp_event;
	sensors_event_t gyr_event;
	sensors_vec_t orientation;
	float temperature;
	//sensors_event_t event;
	sensor_t acc_sensor;
	sensor_t mag_sensor;
	sensor_t gyr_sensor;
	sensor_t bmp_sensor;
	//delay(500);
	acc.getSensor(&acc_sensor);
	acc.getEvent(&acc_event);
	mag.getSensor(&mag_sensor);
	mag.getEvent(&mag_event);
	gyr.getSensor(&gyr_sensor);
	gyr.getEvent(&gyr_event);
	bmp.getSensor(&bmp_sensor);
	bmp.getEvent(&bmp_event);
	bmp.getTemperature(&temperature);

	//Kalman Values
	accX = acc_event.acceleration.x;
	accY = acc_event.acceleration.y;
	accZ = acc_event.acceleration.z;
	tempRaw = (int) temperature;
	gyroX = gyr_event.gyro.x;
	gyroY = gyr_event.gyro.y;
	gyroZ = gyr_event.gyro.z;
	double dt = (double) (micros() - timer) / 1000000; // Calculate delta time
	timer = micros();

#ifdef RESTRICT_PITCH // Eq. 25 and 26
	double roll = atan2(accY, accZ) * RAD_TO_DEG;
	double pitch = atan(-accX / sqrt(accY * accY + accZ * accZ)) * RAD_TO_DEG;
#else // Eq. 28 and 29
	double roll = atan(accY / sqrt(accX * accX + accZ * accZ)) * RAD_TO_DEG;
	double pitch = atan2(-accX, accZ) * RAD_TO_DEG;
#endif

	double gyroXrate = gyroX / 131.0; // Convert to deg/s
	double gyroYrate = gyroY / 131.0; // Convert to deg/s

#ifdef RESTRICT_PITCH
	// This fixes the transition problem when the accelerometer angle jumps between -180 and 180 degrees
	if ((roll < -90 && kalRoll > 90) || (roll > 90 && kalRoll < -90)) {
		kalmanX.setAngle(roll);
		compRoll = roll;
		kalRoll = roll;
		gyroXangle = roll;
	}
	else
	kalRoll = kalmanX.getAngle(roll, gyroXrate, dt); // Calculate the angle using a Kalman filter

	if (abs(kalRoll) > 90)
	gyroYrate = -gyroYrate;// Invert rate, so it fits the restriced accelerometer reading
	kalPitch = kalmanY.getAngle(pitch, gyroYrate, dt);
#else
	// This fixes the transition problem when the accelerometer angle jumps between -180 and 180 degrees
	if ((pitch < -90 && kalPitch > 90) || (pitch > 90 && kalPitch < -90)) {
		kalmanY.setAngle(pitch);
		compPitch = pitch;
		kalPitch = pitch;
		gyroYangle = pitch;
	} else
		kalPitch = kalmanY.getAngle(pitch, gyroYrate, dt); // Calculate the angle using a Kalman filter

	if (abs(kalPitch) > 90)
		gyroXrate = -gyroXrate; // Invert rate, so it fits the restriced accelerometer reading
	kalRoll = kalmanX.getAngle(roll, gyroXrate, dt); // Calculate the angle using a Kalman filter
#endif

	gyroXangle += gyroXrate * dt; // Calculate gyro angle without any filter
	gyroYangle += gyroYrate * dt;
	//gyroXangle += kalmanX.getRate() * dt; // Calculate gyro angle using the unbiased rate
	//gyroYangle += kalmanY.getRate() * dt;

	compRoll = 0.93 * (compRoll + gyroXrate * dt) + 0.07 * roll; // Calculate the angle using a Complimentary filter
	compPitch = 0.93 * (compPitch + gyroYrate * dt) + 0.07 * pitch;

	// Reset the gyro angle when it has drifted too much
	if (gyroXangle < -180 || gyroXangle > 180)
		gyroXangle = kalRoll;
	if (gyroYangle < -180 || gyroYangle > 180)
		gyroYangle = kalPitch;

	//Serial.print(F("----------- ACCELEROMETER ----------"));
	Serial.print(acc_sensor.name);
	Serial.print(F("|"));
	Serial.print(acc_sensor.version);
	Serial.print(F("|"));
	Serial.print(acc_sensor.sensor_id);

	Serial.print(F("|"));
	Serial.print(acc_sensor.max_value);
	Serial.print(F("|"));
	Serial.print(acc_sensor.min_value);
	Serial.print(F("|"));
	Serial.print(acc_sensor.resolution);
	Serial.print(F("|"));
	Serial.print(acc_event.acceleration.x);
	Serial.print(F("|"));
	Serial.print(acc_event.acceleration.y);
	Serial.print(F("|"));
	Serial.print(acc_event.acceleration.z);

//	if (dof.accelGetOrientation(&acc_event, &orientation)) {
//		/* 'orientation' should have valid .roll and .pitch fields */
//		Serial.print(F("|"));
//		Serial.print(orientation.roll);
//		Serial.print(F("|"));
//		Serial.print(orientation.pitch);
//	}Fehlende Methode accelGetOrientation in ahrs
	 

	Serial.print(F("|"));
	Serial.print(mag_sensor.name);
	Serial.print(F("|"));
	Serial.print(mag_sensor.version);
	Serial.print(F("|"));
	Serial.print(mag_sensor.sensor_id);
	Serial.print(F("|"));
	Serial.print(mag_sensor.max_value); // Serial.print(F(" uT"));
	Serial.print(F("|"));
	Serial.print(mag_sensor.min_value); // Serial.print(F(" uT"));
	Serial.print(F("|"));
	Serial.print(mag_sensor.resolution); // Serial.print(F(" uT"));
	Serial.print(F("|"));
	Serial.print(mag_event.magnetic.x);
	Serial.print(F("|"));
	Serial.print(mag_event.magnetic.y);
	Serial.print(F("|"));
	Serial.print(mag_event.magnetic.z);
	/* Calculate the heading using the magnetometer */
//	if (dof.magGetOrientation(SENSOR_AXIS_Z, &mag_event, &orientation)) {
//		/* 'orientation' should have valid .heading data now */
//		Serial.print(F("|"));
//		Serial.print(orientation.heading);
//	}Fehlende Methode accelGetOrientation in ahrs

	/* Use the new fusionGetOrientation function to merge accel/mag data */
//	if (dof.fusionGetOrientation(&acc_event, &mag_event, &orientation)) {
//		/* 'orientation' should have valid .roll and .pitch fields */
//		Serial.print(F("|"));
//		Serial.print(orientation.roll);
//		Serial.print(F("|"));
//		Serial.print(orientation.pitch * -1);
//		Serial.print(F("|"));
//		Serial.print(360 - orientation.heading);
//	}Fehlende Methode accelGetOrientation in ahrs

	//Serial.print(F("------------- GYROSCOPE -----------"));
	Serial.print(F("|"));
	Serial.print(gyr_sensor.name);
	Serial.print(F("|"));
	Serial.print(gyr_sensor.version);
	Serial.print(F("|"));
	Serial.print(gyr_sensor.sensor_id);
	Serial.print(F("|"));
	Serial.print(gyr_sensor.max_value); // Serial.print(F(" rad/s"));
	Serial.print(F("|"));
	Serial.print(gyr_sensor.min_value); // Serial.print(F(" rad/s"));
	Serial.print(F("|"));
	Serial.print(gyr_sensor.resolution); // Serial.print(F(" rad/s"));
	/* Display the results (gyrocope values in rad/s) */
	Serial.print(F("|"));
	Serial.print(gyr_event.gyro.x);
	Serial.print(F("|"));
	Serial.print(gyr_event.gyro.y);
	Serial.print(F("|"));
	Serial.print(gyr_event.gyro.z);

	//Serial.print(F("-------- PRESSURE/ALTITUDE/Variometer ---------"));
	Serial.print(F("|"));
	Serial.print(bmp_sensor.name);
	Serial.print(F("|"));
	Serial.print(bmp_sensor.version);
	Serial.print(F("|"));
	Serial.print(bmp_sensor.sensor_id);
	Serial.print(F("|"));
	Serial.print(bmp_sensor.max_value); // Serial.print(F(" hPa"));
	Serial.print(F("|"));
	Serial.print(bmp_sensor.min_value); // Serial.print(F(" hPa"));
	Serial.print(F("|"));
	Serial.print(bmp_sensor.resolution); // Serial.print(F(" hPa"));

	/* Display the pressure sensor results (barometric pressure is measure in hPa) */
	if (bmp_event.pressure) {
		/* Display atmospheric pressure in hPa */
		Serial.print(F("|"));
		Serial.print(bmp_event.pressure);
		/* Display ambient temperature in C */
		//float temperature;
		//bmp.getTemperature(&temperature);
		Serial.print(F("|"));
		Serial.print(temperature);
		/* Then convert the atmospheric pressure, SLP and temp to altitude    */
		/* Update this next line with the current SLP for better results      */
		Serial.print(F("|"));
		float alt_neu = bmp.pressureToAltitude(seaLevelPressure,
				bmp_event.pressure, temperature);
		Serial.print(alt_neu);
		if (micros() - timer2 > 1000) {
			vario = (alt_neu - alt);
			timer2 = micros();
			alt = alt_neu;
		}
	}

	//Kalman-Filter Results
	Serial.print(F("|"));
	Serial.print(compRoll);
	Serial.print(F("|"));
	Serial.print(compPitch * -1);

	Serial.print(F("|"));
	Serial.print(kalRoll);
	Serial.print(F("|"));
	Serial.print(kalPitch * -1);

	Serial.print(F("|"));
	Serial.print(vario);

	//Schub
	Serial.print(F("|"));
	Serial.print(Speed);

	Serial.print("\n");
}

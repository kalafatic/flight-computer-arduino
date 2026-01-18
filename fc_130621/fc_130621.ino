#include <Nokia_5110.h>
#include "Arduino.h"
#include <Wire.h>
#include "MS5611.h"
#include <SoftwareSerial.h>
#include <SimpleKalmanFilter.h>

#define DISPLAY_RST 2
#define DISPLAY_CE 3
#define DISPLAY_DC 4
#define DISPLAY_DIN 5
#define DISPLAY_CLK 6

#define SOUND 7
#define LED_SOUND 8
#define LED_BLUETOOTH 9

/*  Arduino Nano Pin "D10" -> HC-06 Pin "RX"
 *  Arduino Nano Pin "D11" -> HC-06 Pin "TX"
 */
#define BLUETOOTH_RX 10 // digital 10 (Nano clone)
#define BLUETOOTH_TX 11 // digital 11 (Nano clone)

// inicializace Bluetooth modul SoftwareSerial
SoftwareSerial bluetooth(BLUETOOTH_TX, BLUETOOTH_RX);

Nokia_5110 lcd = Nokia_5110( DISPLAY_RST, DISPLAY_CE, DISPLAY_DC, DISPLAY_DIN,
		DISPLAY_CLK);

/*  Arduino Nano Pin "A5" -> MS5611 Pin "SCL"
 *  Arduino Nano Pin "A4" -> MS5611 Pin "SDA"
 *  Arduino Nano Pin "3.3V" -> MS5611 Pin "VIN"
 *  Arduino Nano Pin "GND" -> MS5611 Pin "GND"
 */
MS5611 ms5611;

double pressure;
double temperature;
double altitude;

int VARIO_BT_DELAY = 24;

/*
 SimpleKalmanFilter(e_mea, e_est, q);
 e_mea: Measurement Uncertainty
 e_est: Estimation Uncertainty
 q: Process Noise
 */
SimpleKalmanFilter pressureKalmanFilter(1, 1, 0.01);
long refresh_time;

void initBluetooth() {
	bluetooth.begin(9600);

	Serial.println("Bluetooth ON");
	digitalWrite(LED_BLUETOOTH, HIGH);
}

void initMS5611() {
	Serial.println("Initialize MS5611 Sensor");

	while (!ms5611.begin(MS5611_ULTRA_HIGH_RES)) {
		Serial.println("Could not find a valid MS5611 sensor, check wiring!");
		delay(VARIO_BT_DELAY);
	}
	Serial.println("MS5611 Sensor Initialized");

	// Check settings
	checkSettings();
}

void initLCD() {
	/**
	 * Note: if instead of text being shown on the display, all the segments are on, you may need to decrease contrast value.
	 */
	//lcd.setContrast(60); // 60 is the default value set by the driver
	lcd.print("Please Wait ...");
	delay(1000);
	lcd.clear();

	lcd.print("Hi there");
	lcd.println(":D");

	lcd.setCursor(0, 5);
	lcd.println("1 2 3 ...");

}
void checkSettings() {
	Serial.print("Oversampling: ");
	Serial.println(ms5611.getOversampling());
}

// The setup function is called once at startup of the sketch
void setup() {
	pinMode(LED_SOUND, OUTPUT);
	pinMode(LED_BLUETOOTH, OUTPUT);

	Serial.begin(9600);

	initLCD();

	// Initialize Bluetooth module
	initBluetooth();
	// Initialize MS5611 sensor
	initMS5611();
}

// The loop function is called in an endless loop
void loop() {
	receiveBT();

	lcd.clear();

	delay(VARIO_BT_DELAY);

	sendBT();
	digitalWrite(LED_SOUND, HIGH);

	delay(VARIO_BT_DELAY);

	lcd.print("PRS: ");
	lcd.print(pressure);

	lcd.setCursor(0, 2);
	lcd.print("TMP: ");
	lcd.println(temperature);

	lcd.setCursor(0, 4);
	lcd.print("ALT: ");
	lcd.println(altitude);

	tone(SOUND, 500, 50);

	float estimated_altitude = pressureKalmanFilter.updateEstimate(altitude);

	if (millis() > refresh_time) {
		Serial.print("altitude-");
		Serial.print(altitude, 6);
		Serial.print(", estimated_altitude-");
		Serial.print(estimated_altitude, 6);
		Serial.println();
		refresh_time = millis() + VARIO_BT_DELAY;
	}

	print();
}

void receiveBT() {
	byte bluetooth_data;

	if (bluetooth.available() > 0) {

		bluetooth_data = bluetooth.read();

		switch (bluetooth_data) {

		case '0':
			//      bluetooth.println("Vypni LED diodu.");
			Serial.println(" Vypni LED diodu.");
			break;
		case '1':
			//      bluetooth.println("Vypni LED diodu.");
			Serial.println(" Zapni LED diodu.");
			break;
		default:
			bluetooth.println("Neznamy prikaz.");
			break;
		}
	}
}

void sendBT() {
	pressure = ms5611.readPressure();
	temperature = ms5611.readTemperature();
	altitude = ms5611.getAltitude(pressure);

	// xtrack protocol
	bluetooth.print("_PRS ");
	bluetooth.print((long int) (pressure + 0.5), HEX);
	bluetooth.print("\n");

	// FC protocol
	bluetooth.print("_TMP ");
	bluetooth.print((long int) (temperature + 0.5), HEX);
	bluetooth.print("\n");
	// FC protocol
	bluetooth.print("_ALT ");
	bluetooth.print((long int) (altitude + 0.5), HEX);
	bluetooth.print("\n");
	bluetooth.flush();
}

void print() {
	Serial.println("--");

	Serial.print(" pressure = ");
	Serial.print(pressure);

	Serial.print(", pressure HEX = ");
	Serial.print((long int) (pressure + 0.5), HEX);

	Serial.print(", altitude = ");
	Serial.print(altitude, 2);

	Serial.print(", temperature = ");
	Serial.print(temperature);
	Serial.println(" *C");
}

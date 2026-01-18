#include <Nokia_5110.h>
#include "Arduino.h"
#include <Wire.h>
#include "MS5611.h"
#include <SoftwareSerial.h>
#include <SimpleKalmanFilter.h>

const int DISPLAY_RST_PIN = 2;
const int DISPLAY_CE_PIN = 3;
const int DISPLAY_DC_PIN = 4;
const int DISPLAY_DIN_PIN = 5;
const int DISPLAY_CLK_PIN = 6;

const int SOUND_PIN = 7;
const int LED_SOUND_PIN = 8;
const int LED_BLUETOOTH_PIN = 9;

/*  Arduino Nano Pin "D10" -> HC-06 Pin "RX"
 *  Arduino Nano Pin "D11" -> HC-06 Pin "TX"
 */
const int BLUETOOTH_RX_PIN = 10; // digital 10 (Nano clone)
const int BLUETOOTH_TX_PIN = 11; // digital 11 (Nano clone)

const int VARIO_BT_DELAY_MS = 24;

const long BLUETOOTH_SERIAL_SPEED = 9600;
const long COMPUTER_SERIAL_SPEED = 9600;

const int TONE_FREQUENCY_HZ = 500;
const int TONE_DURATION_MS = 50;

// inicializace Bluetooth modul SoftwareSerial
SoftwareSerial bluetooth(BLUETOOTH_TX_PIN, BLUETOOTH_RX_PIN);

Nokia_5110 lcd = Nokia_5110(DISPLAY_RST_PIN, DISPLAY_CE_PIN, DISPLAY_DC_PIN,
		DISPLAY_DIN_PIN, DISPLAY_CLK_PIN);

/*  Arduino Nano Pin "A5" -> MS5611 Pin "SCL"
 *  Arduino Nano Pin "A4" -> MS5611 Pin "SDA"
 *  Arduino Nano Pin "3.3V" -> MS5611 Pin "VIN"
 *  Arduino Nano Pin "GND" -> MS5611 Pin "GND"
 */
MS5611 ms5611;

double pressure;
double temperature;
double altitude;

/*
 SimpleKalmanFilter(e_mea, e_est, q);
 e_mea: Measurement Uncertainty
 e_est: Estimation Uncertainty
 q: Process Noise
 */
SimpleKalmanFilter pressureKalmanFilter(1, 1, 0.01);
long refresh_time;

void initBluetooth() {
	bluetooth.begin(BLUETOOTH_SERIAL_SPEED);

	Serial.println("Bluetooth ON");
	digitalWrite(LED_BLUETOOTH_PIN, HIGH);
}

void initMS5611() {
	Serial.println("Initialize MS5611 Sensor");

	while (!ms5611.begin()) {
		Serial.println("Could not find a valid MS5611 sensor, check wiring!");
		delay(VARIO_BT_DELAY_MS);
	}
	ms5611.setOversampling(OSR_ULTRA_HIGH);
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
	pinMode(LED_SOUND_PIN, OUTPUT);
	pinMode(LED_BLUETOOTH_PIN, OUTPUT);

	Serial.begin(COMPUTER_SERIAL_SPEED);

	initLCD();

	// Initialize Bluetooth module
	initBluetooth();
	// Initialize MS5611 sensor
	initMS5611();
}

void readSensorData() {
	ms5611.read();
	pressure = ms5611.getPressure();
	temperature = ms5611.getTemperature();
	altitude = ms5611.getAltitude(pressure);
}

void updateDisplay() {
	lcd.clear();

	lcd.print("PRS: ");
	lcd.print(pressure);

	lcd.setCursor(0, 2);
	lcd.print("TMP: ");
	lcd.println(temperature);

	lcd.setCursor(0, 4);
	lcd.print("ALT: ");
	lcd.println(altitude);
}

void handleVario() {
	digitalWrite(LED_SOUND_PIN, HIGH);
	tone(SOUND_PIN, TONE_FREQUENCY_HZ, TONE_DURATION_MS);
}

// The loop function is called in an endless loop
void loop() {
	receiveBT();
	readSensorData();

	delay(VARIO_BT_DELAY_MS);

	sendBT();
	updateDisplay();
	handleVario();

	delay(VARIO_BT_DELAY_MS);

	float estimated_altitude = pressureKalmanFilter.updateEstimate(altitude);

	if (millis() > refresh_time) {
		Serial.print("altitude-");
		Serial.print(altitude, 6);
		Serial.print(", estimated_altitude-");
		Serial.print(estimated_altitude, 6);
		Serial.println();
		refresh_time = millis() + VARIO_BT_DELAY_MS;
	}

	print();

	digitalWrite(LED_SOUND_PIN, LOW);
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
	// The custom Bluetooth protocol sends data in the following format:
	// _<ID> <VALUE_HEX>\n
	// where <ID> is a 3-letter identifier (PRS, TMP, ALT)
	// and <VALUE_HEX> is the rounded integer value in hexadecimal format.

	// Send pressure data (xtrack protocol)
	bluetooth.print("_PRS ");
	bluetooth.print((long int) round(pressure), HEX);
	bluetooth.print("\n");

	// Send temperature data (FC protocol)
	bluetooth.print("_TMP ");
	bluetooth.print((long int) round(temperature), HEX);
	bluetooth.print("\n");

	// Send altitude data (FC protocol)
	bluetooth.print("_ALT ");
	bluetooth.print((long int) round(altitude), HEX);
	bluetooth.print("\n");

	bluetooth.flush();
}

void print() {
	Serial.println("--");

	Serial.print(" pressure = ");
	Serial.print(pressure);

	Serial.print(", pressure HEX = ");
	Serial.print((long int) round(pressure), HEX);

	Serial.print(", altitude = ");
	Serial.print(altitude, 2);

	Serial.print(", temperature = ");
	Serial.print(temperature);
	Serial.println(" *C");
}

/*
 * TELEKOMMANDOS
 * S Zahl 	: 	Ändert Interval SignalProcessing-Thread
 * T Zahl	: 	Ändert Interval Telemetrie-Thread
 * Q		: 	postet ALLE Telemetrie-Daten
 * A		:	postet nur Accelerometer
 * G		:	postet nur Gyroskop
 * M		:	postet nur Magnetometer
 * H		:	postet nur Temperatur
 *
 */

/// Standardbibliotheken
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <ctype.h>
#include <math.h>
//#include <string.h> // Vorsicht mit Includes von C++ Bibliotheken! Verursacht Fehler!

/// RODOS & Board
#include "rodos.h"
#include "hal_gpio.h"

/// Makros
// Adresse für IMU Sensoren:
#define LSM9DS1_AG 0x6B // (0x6B) Accelerometer & Gyroscope
#define LSM9DS1_M 0x1E // (0x1E) Magnetometer

// feste Initialsierungen für Sensoren:
#define init_CTRL_REG1_G_addr 0x10
#define init_CTRL_REG1_G_val 0b10011000 // (0x98) ODR=238Hz, Cutoff=14Hz, Full Scale=2000
#define init_CTRL_REG6_XL_addr 0x20
#define init_CTRL_REG6_XL_val 0b10000011 // (0x83) Acceleration ODR=238Hz, Acceleration Full-scale=+-2g, BW w.r.t ODR, Acceleration Anti alias Filter, Bandwidth=50Hz
#define init_CTRL_REG1_M_addr 0x20
#define init_CTRL_REG1_M_val 0b00011100 // (0x1C) ODR=80Hz
#define init_CTRL_REG2_M_addr 0x21
#define init_CTRL_REG2_M_val 0b00000000 // (0x00) Magnetic Full-scale=+-4gauss
#define init_CTRL_REG3_M_addr 0x22
#define init_CTRL_REG3_M_val 0b00000000 // (0x00) Continuous Conversion Mode

// Gyroscope
#define ORIENT_CFG_G 0x13 // Angular rate sensor sign and orientation register
#define OUT_X_G 0x18 // + 0x19 !
#define OUT_Y_G 0x1A // + 0x1B !
#define OUT_Z_G 0x1C // + 0x1D !

// Accelerometer
#define OUT_X_XL 0x28 // + 0x29 !
#define OUT_Y_XL 0x2A // + 0x2B !
#define OUT_Z_XL 0x2C // + 0x2D !

// Magnetometer
#define OUT_X_L_M 0x28
#define OUT_Y_L_M 0x2A
#define OUT_Z_L_M 0x2C

// Temperature
#define OUT_TEMP_L 0x15

/// Strukturen
// Uart -> Interpreter/StateMachine
struct Telecommand {
	char id;
	float data;
};

// Interpreter/StateMachine -> Threads
struct Command {
	char id;
	float value;
};

struct Data {
	float x;
	float y;
	float z;
};

/*
 enum config_axes {
 X, Y, Z
 } config; // wird verwendet um IMU zu initialisieren
 */

/// Globale Variablen
/// Hardware
HAL_GPIO button(GPIO_000); // blauer Button
//	LEDs			grün	 orange		rot		   blau
HAL_GPIO LED[4] = { GPIO_060, GPIO_061, GPIO_062, GPIO_063 };

/*
 * grün = SignalProcessing
 * orange = Telemetry
 * rot = Telecommand
 * blau = Kalibrierung
 */

HAL_UART BT2UART(UART_IDX2); // UART (Tx=PD5, Rx=PD6)
HAL_SPI IMU(SPI_IDX1); // SPI: müsste so sein wie auf Folie (Aufbau) abgebildet

// Sensoren:
HAL_GPIO CS_AG(GPIO_006); // IMU Chip Pin für Gyro und Accelerometer
HAL_GPIO CS_M(GPIO_041); // IMU Chip Pin für Magnometer

// LSB: Mit Integerwert von jeweiligem Sensor multiplizieren um realen Sensorwert zu erhalten
const float LSB_G = 70e-3f;
const float LSB_A = 0.061e-3f;
const float LSB_M = 0.14e-3f;

/// Intertask
// UART -> Interpreter/StateMachine
Topic<Telecommand> TopicTelecommand(-1, "TopicTelecommand");

// Interpreter/StateMachine -> Counter-Thread
Topic<Command> TopicTelemetry(-1, "TopicTelemetry");

CommBuffer<Data> cbAcc;
CommBuffer<Data> cbMag;
CommBuffer<Data> cbGyr;
CommBuffer<float> cbTemp;

/// Statische Funktionen
// Schaltet LED an und nach Wartezeit wieder aus
static void ToggleLED(HAL_GPIO& led, uint32_t length_ms) {
	led.setPins(1);
	AT(NOW() + length_ms * MILLISECONDS);
	led.setPins(0);
}

static void changeLED(HAL_GPIO& led, bool on = true) {
	led.setPins(on);
}

// Schreibt einen String in UART:
static void writeUART(HAL_UART& uart, const char* string) {
	const int len = strlen(string);

	char *str = (char*) calloc(sizeof(char), len + 1);

	sprintf(str, "\n%s", string); // TODO \r evtl. noch anfügen!

	uart.write(str, len);

	free(str);
}

//*******************************************************************************

//*******************************************************************************

// Initialisiert Accelerometer & Gyroscope:
static void initAG(HAL_SPI& imu, HAL_GPIO& pin) {
	// Kommunikation initiieren:
	pin.setPins(0);

	// If write, bit 0 of the subAddress (MSB) should be 0;
	// If single byte, bit 1 of the subAddress should be 0
	uint8_t config_1[2] = { init_CTRL_REG1_G_addr & 0x3F, init_CTRL_REG1_G_val };
	uint8_t config_2[2] =
			{ init_CTRL_REG6_XL_addr & 0x3F, init_CTRL_REG6_XL_val };

	const int i = imu.write(config_1, 2);
	const int j = imu.write(config_2, 2);

	// Prüfen ob Fehler auftraten:
	if ((i < 0) || (j < 0))
		writeUART(BT2UART, "FEHLER INITIALISIERUNG!");

	// Kommunikation schließen:
	pin.setPins(1);
}

// Initialisiert Magnetometer:
static void initM(HAL_SPI& imu, HAL_GPIO& pin) {
	// Kommunikation initiieren:
	pin.setPins(0);

	// If write, bit 0 of the subAddress (MSB) should be 0;
	// If single byte, bit 1 of the subAddress should be 0
	uint8_t config_1[2] = { init_CTRL_REG1_M_addr & 0x3F, init_CTRL_REG1_M_val };
	uint8_t config_2[2] = { init_CTRL_REG2_M_addr & 0x3F, init_CTRL_REG2_M_val };
	uint8_t config_3[2] = { init_CTRL_REG3_M_addr & 0x3F, init_CTRL_REG3_M_val };

	const int i = imu.write(config_1, 2);
	const int j = imu.write(config_2, 2);
	const int k = imu.write(config_3, 2);

	// Prüfen ob Fehler auftraten:
	if ((i < 0) || (j < 0) || (k < 0))
		writeUART(BT2UART, "FEHLER INITIALISIERUNG!");

	// Kommunikation schließen:
	pin.setPins(1);
}

//*******************************************************************************

static void readSensor2Bytes(int16_t& _temp, HAL_GPIO& pin,
		uint8_t regAddress) {
	// Kommunikation öffnen:
	pin.setPins(0);

	uint8_t temp[3];
	uint8_t addr[1] = { 0x80 | (regAddress & 0x3F) };

	const int i = IMU.writeRead(addr, 1, temp, 3); // 2 Bytes lesen

	if (i < 0)
		writeUART(BT2UART, "FEHLER READING 2 BYTES!");

	int16_t temperature = (int16_t) ((temp[2] << 8) | temp[1]); // Temperatur ist ein 12-bit signed Integer

	_temp = temperature;

	// Kommunikation schließen:
	pin.setPins(1);
}

// Liest 6-Bytes von einem Sensor (außer Temperatur!) ein:
static void readSensor6Bytes(int16_t (&arr)[3], HAL_GPIO& pin,
		uint8_t regAddress) {
	// Kommunikation öffnen:
	pin.setPins(0);

	uint8_t temp[7];
	uint8_t addr[1] = { 0x80 | (regAddress & 0x3F) };

	const int i = IMU.writeRead(addr, 1, temp, 7); // wg. Duplex ein Byte mehr!

	if (i < 0)
		writeUART(BT2UART, "FEHLER READING 6 BYTES!");

	int16_t x, y, z; // kombiniert je zwei 8-Bit-Register zu 16-Bit Ganzzahl
	x = (int16_t) ((temp[2] << 8) | temp[1]);
	y = (int16_t) ((temp[4] << 8) | temp[3]);
	z = (int16_t) ((temp[6] << 8) | temp[5]);

	// Ganzzahlen noch mit jeweiligem LSB multiplizieren um Dezimalwert zu erhalten:
	arr[0] = x;
	arr[1] = y;
	arr[2] = z;

	// Kommunikation schließen:
	pin.setPins(1);
}

//*******************************************************************************

// Nimmt 100 Messungen im Abstand von 10ms auf und gibt gemittelte Werte zurück
static void calibrate(int64_t (&Offset)[3], HAL_GPIO& sensor,
		uint8_t regAddress, uint16_t length_ms = 50, uint32_t numberOfSamples =
				100) {
	int64_t data[3]; // summiert auf, muss daher größer als 16 Bits sein!
	int16_t temp[3];

	for (uint i = 0; i < numberOfSamples; i++) {
		readSensor6Bytes(temp, sensor, regAddress); // Hier mit LSB 1, damit die Werte ohne Nachkommastellen in Ganzzahl konvertiert werden können und nichts abgeschnitten wird

		data[0] += temp[0];
		data[1] += temp[1];
		data[2] += temp[2];

		AT(NOW() + length_ms * MILLISECONDS); // 100 * 10 ms = 1 s (Nach jeder Messung kurz warten, bis die nächste aufgenommen wird
	}

	// Offset zuweisen und zurückgeben:
	Offset[0] = (data[0]);
	Offset[1] = (data[1]);
	Offset[2] = (data[2]);
}

// Kalibriert Beschleunigungssensor
static void calibrateAcc(int16_t (&Offset_A)[3]) {
	// Vorgehensweise:
	// erst z-, dann x-, zuletzt y-Achse kalibrieren! Dies dauert jeweils 1 s, dazwischen eine weitere Sekunde warten um Board in Position zu bringen
	ToggleLED(LED[3], 250);

	int64_t P1[3] = { 0, 0, 0 }, P2[3] = { 0, 0, 0 }, P3[3] = { 0, 0, 0 };

	// ######################################################
	calibrate(P1, CS_AG, OUT_X_XL); // x = 0, y = 0, z = 1*g
	// ######################################################

	ToggleLED(LED[3], 500);
	AT(NOW() + 1 * SECONDS);
	ToggleLED(LED[3], 500);

	// ######################################################
	calibrate(P2, CS_AG, OUT_X_XL); // x = 1*g, y = 0, z = 0
	// ######################################################

	ToggleLED(LED[3], 500);
	AT(NOW() + 1 * SECONDS);
	ToggleLED(LED[3], 500);

	// ######################################################
	calibrate(P3, CS_AG, OUT_X_XL); // x = 0, y = 1*g, z = 0
	// ######################################################

	ToggleLED(LED[3], 500);
	AT(NOW() + 500 * MILLISECONDS);

	// Werte ausrechnen:
	// Dabei vernachlässigen das Nachkommastelle gerundet wird, da es sich um sehr große Zahlen handert, dürfte das Ergebnis nur sehr gering vom exakten Wert abweichen:
	Offset_A[0] = static_cast<int16_t>((P1[0] + P3[0]) / 200.0); // Offset x-Achse
	Offset_A[1] = static_cast<int16_t>((P1[1] + P2[1]) / 200.0); // Offset y-Achse
	Offset_A[2] = static_cast<int16_t>((P3[2] + P2[2]) / 200.0); // Offset z-Achse

	// Abschluss signalisieren:
	ToggleLED(LED[3], 500);
	AT(NOW() + 500 * MILLISECONDS);
	ToggleLED(LED[3], 500);
}

// Kalibriert Gyroskop
static void calibrateGyro(int16_t (&Offset_G)[3]) {
	// Vorgehensweise:
	// erst z-, dann x-, zuletzt y-Achse kalibrieren! Dies dauert jeweils 1 s, dazwischen eine weitere Sekunde warten um Board in Position zu bringen
	ToggleLED(LED[3], 500);

	int64_t P1[3] = { 0, 0, 0 }, P2[3] = { 0, 0, 0 }, P3[3] = { 0, 0, 0 };

	// ######################################################
	calibrate(P1, CS_AG, OUT_X_G, 20); // z-Achse
	// ######################################################

	ToggleLED(LED[3], 500);
	AT(NOW() + 1 * SECONDS);
	ToggleLED(LED[3], 500);

	// ######################################################
	calibrate(P2, CS_AG, OUT_X_G, 20); // x-Achse
	// ######################################################

	ToggleLED(LED[3], 500);
	AT(NOW() + 1 * SECONDS);
	ToggleLED(LED[3], 500);

	// ######################################################
	calibrate(P3, CS_AG, OUT_X_G, 20); // y-Achse
	// ######################################################

	ToggleLED(LED[3], 500);
	AT(NOW() + 500 * MILLISECONDS);

	// Werte ausrechnen:
	Offset_G[0] = P2[0] / 100.0; // Offset x-Achse
	Offset_G[1] = P3[1] / 100.0; // Offset y-Achse
	Offset_G[2] = P1[2] / 100.0; // Offset z-Achse

	// Abschluss signalisieren:
	ToggleLED(LED[3], 500);
	AT(NOW() + 500 * MILLISECONDS);
	ToggleLED(LED[3], 500);
}

// Kalibriert Magnetometer
static void calibrateMag(int16_t (&Offset_M)[3][2]) {
	// Vorgehensweise:
	// erst z-, dann x-, zuletzt y-Achse kalibrieren!
	ToggleLED(LED[3], 500);

	int16_t M_c[2] = {0, 0}; // speichert über Kalibrierung den gemessenen Min- ([0]) und Maxwert ([1])
	int64_t time; // speichert die Zeit bei Beginn, sodass genau 2 Sekunden kalibriert werden kann
	// ######################################################
	time = NOW();
	do {
		int16_t currVal; // immer der aktuell gemessene Wert

		// Messung abrufen:
		readSensor2Bytes(currVal, CS_M, OUT_Z_L_M);

		// prüfen ob der Wert gespeichert werden soll:
		if (currVal < M_c[0])
			M_c[0] = currVal;
		else if (currVal > M_c[1])
			M_c[1] = currVal;

	} while (NOW() < (time + 2 * SECONDS));

	// In M_c befinden sich jetzt der Min/Max-Wert für Z-Achse, diesen speichern:
	Offset_M[2][0] = M_c[0];
	Offset_M[2][1] = M_c[1];

	// Zwischenspeicher zurücksetzen:
	M_c[0] = 0;
	M_c[1] = 0;
	// ######################################################

	ToggleLED(LED[3], 500);
	AT(NOW() + 1 * SECONDS);
	ToggleLED(LED[3], 500);

	// ######################################################
	time = NOW();
	do {
		int16_t currVal; // immer der aktuell gemessene Wert

		// Messung abrufen:
		readSensor2Bytes(currVal, CS_M, OUT_X_L_M);

		// prüfen ob der Wert gespeichert werden soll:
		if (currVal < M_c[0])
			M_c[0] = currVal;
		else if (currVal > M_c[1])
			M_c[1] = currVal;

	} while (NOW() < (time + 2 * SECONDS));

	// In M_c befinden sich jetzt der Min/Max-Wert für X-Achse, diesen speichern:
	Offset_M[0][0] = M_c[0];
	Offset_M[0][1] = M_c[1];

	// Zwischenspeicher zurücksetzen:
	M_c[0] = 0;
	M_c[1] = 0;
	// ######################################################

	ToggleLED(LED[3], 500);
	AT(NOW() + 1 * SECONDS);
	ToggleLED(LED[3], 500);

	// ######################################################
	time = NOW();
	do {
		int16_t currVal; // immer der aktuell gemessene Wert

		// Messung abrufen:
		readSensor2Bytes(currVal, CS_M, OUT_Y_L_M);

		// prüfen ob der Wert gespeichert werden soll:
		if (currVal < M_c[0])
			M_c[0] = currVal;
		else if (currVal > M_c[1])
			M_c[1] = currVal;

	} while (NOW() < (time + 2 * SECONDS));

	// In M_c befinden sich jetzt der Min/Max-Wert für Y-Achse, diesen speichern:
	Offset_M[1][0] = M_c[0];
	Offset_M[1][1] = M_c[1];

	// Zwischenspeicher zurücksetzen:
	M_c[0] = 0;
	M_c[1] = 0;
	// ######################################################

	ToggleLED(LED[3], 500);
	AT(NOW() + 500 * MILLISECONDS);

	// Abschluss signalisieren:
	ToggleLED(LED[3], 500);
	AT(NOW() + 500 * MILLISECONDS);
	ToggleLED(LED[3], 500);
}

//*******************************************************************************

// Berechnet über Daten des Accelerometer die Orientierung
static void calcRP_Acc(float& pitch, float& roll, float x, float y, float z) {
	roll = atan2(y, z);
	pitch = atan2(-x, sqrt(y * y + z * z));
}
/*
 // Berechnet über Daten des Gyrometer die Orientierung
 static void calcY_Mag(float& yaw, float mx, float my, float mz) {
 yaw = atan2(my, mx);
 }
 */
static float calcYaw(float gz) {
	return gz * 0.05;
}

//*******************************************************************************

class SignalProcessing: public Thread, public SubscriberReceiver<Command> {
	uint interval; // Standard: 50 ms

	// Accelerometer
	int16_t Offset_A[3]; // Offset Beschleunigungssensor
	int16_t Value_A[3]; // Speichert zuletzt gemessene Beschleunigungsdaten pro Achse
	// Gyroscope
	int16_t Offset_G[3]; // Offset Gyroskop
	int16_t Value_G[3]; // Speichert zuletzt gemessene Daten des Gyroskops pro Achse
	// Magnetometer
	int16_t Offset_M[3][2]; // zwei Werte für jede Achse: Min[0]/Max[1] Wert der Kalibrierung!
	int16_t Value_M[3]; // Speichert zuletzt gemessene Daten des Magnetometers pro Achse

	float Value_Temp; // Speichert zuletzt gemessene Temperatur, ausnahmsweise in Float und nicht Ganzzahl speichern, da kein Offset und keine Kalibrierung nötig

	bool calibration_complete; // legt fest ob Kalibrierung der IMU erfolgt ist

public:
	SignalProcessing() :
			SubscriberReceiver<Command>(TopicTelemetry,
					"TopicIntervalSignalProcessingReceiver"), interval(100) { // abonniert Interval-Topic
		// Standart: 100 ms
		calibration_complete = false;
		Value_Temp = 0.0;
	}

	bool isCalibrated() {
		return this->calibration_complete;
	}

	void init(void) {
		// zugehörige LEDs auf Ausgang schalten:
		LED[0].init(1, 1, 1); // Aufgabenstellung (grün)
		LED[3].init(1, 1, 1); // Kalibrierung (blau)

		// Button auf Eingang schalten:
		button.init();

		// Initialisieren: IMU:
		IMU.init(1000000); // standard-baudrate == 1000000 // Wichtig! Muss vor Initialisierung der Sensoren ausgeführt werden!

		// Initialisieren: Beschleunigungssensor & Gyroskop:
		CS_AG.init(true, 1, 1); // Init Gyro & Accelerometer
		initAG(IMU, CS_AG); // Einstellungen in Register schreiben
		writeUART(BT2UART,
				"Initialisierung Beschleunigungssensor und Gyroskop abgeschlossen...");

		// Initialisieren: Magnetometer:
		CS_M.init(true, 1, 1); // Init Magnometer
		initM(IMU, CS_M); // Einstellungen in Register schreiben
		writeUART(BT2UART, "Initialisierung Magnetometer abgeschlossen...");

		// ALLE Variablen mit Null initialisieren:
		Offset_A[0] = 0;
		Offset_A[1] = 0;
		Offset_A[2] = 0;

		Offset_M[0][0] = 0;
		Offset_M[0][1] = 0;
		Offset_M[1][0] = 0;
		Offset_M[1][1] = 0;
		Offset_M[2][0] = 0;
		Offset_M[2][1] = 0;

		Offset_G[0] = 0;
		Offset_G[1] = 0;
		Offset_G[2] = 0;
	}

	void put(Command& data) {
		Command* _data = (Command*) &data;

		// Interval ändern:
		//if (_data->id == 'I')
		//	this->interval = _data->value;
		switch (_data->id) {
		case 'I':
			this->interval = (uint) (_data->value);
			break;
		}
	}

	void run(void) {
		while (1) {
			// ############################# KALIBRIERUNG #############################
			if (!calibration_complete) {
				while (1) {
					static uint8_t counter = 1;

					if (button.readPins() == 1) { // prüfen ob Button gedrückt wurde
						ToggleLED(LED[3], 400);
						AT(NOW() + 500 * MILLISECONDS); // kurz warten bevor kalibriert wird um nicht irgendwie nächste Schleifeniteration auch gleich zu aktivieren

						// Reihenfolge jeweils: x, y, z
						// Unterscheiden, in welcher Kalibrierungsphase wir sind (1-3)
						switch (counter) {
						case 1:
							// Accelerometer
							calibrateAcc(this->Offset_A);
							writeUART(BT2UART,
									"-- Kalibrierung Beschleunigungssensor abgeschlossen... --");
							{
								const char* cal_acc_msg =
										"Acc: Offset-X=%3.8f g, Offset-Y=%3.8f g, Offset-Z=%3.8f g\0";
								char cal_acc_str[79];
								sprintf(cal_acc_str, cal_acc_msg,
										static_cast<float>(this->Offset_A[0]
												* LSB_A),
										static_cast<float>(this->Offset_A[1]
												* LSB_A),
										static_cast<float>(this->Offset_A[2]
												* LSB_A));
								writeUART(BT2UART, cal_acc_str);
							}
							break;

						case 2:
							// Gyroskope
							calibrateGyro(this->Offset_G);
							writeUART(BT2UART,
									"-- Kalibrierung Gyroskop abgeschlossen... --");
							{
								const char* cal_gyro_msg =
										"Gyro: Offset-X=%3.8f dps, Offset-Y=%3.8f dps, Offset-Z=%3.8f dps\0";
								char cal_gyro_str[86];
								sprintf(cal_gyro_str, cal_gyro_msg,
										static_cast<float>(this->Offset_G[0]
												* LSB_G),
										static_cast<float>(this->Offset_G[1]
												* LSB_G),
										static_cast<float>(this->Offset_G[2]
												* LSB_G));
								writeUART(BT2UART, cal_gyro_str);
							}
							break;

						case 3:
							// Magnetometer
							calibrateMag(this->Offset_M);
							writeUART(BT2UART,
									"-- Kalibrierung Magnetometer abgeschlossen... --");
							{
								const char* cal_mag_msg =
										"Mag: Offset-X_max=%3.8f gauss, Offset-Y_max=%3.8f gauss, Offset-Z_max=%3.8f gauss\nOffset-X_min=%3.8f gauss, Offset-Y_min=%3.8f gauss, Offset-Z_min=%3.8f gauss\0";
								char cal_mag_str[201];
								sprintf(cal_mag_str, cal_mag_msg,
										static_cast<float>(this->Offset_M[0][1]
												* LSB_M),
										static_cast<float>(this->Offset_M[1][1]
												* LSB_M),
										static_cast<float>(this->Offset_M[2][1]
												* LSB_M),
										static_cast<float>(this->Offset_M[0][0]
												* LSB_M),
										static_cast<float>(this->Offset_M[1][0]
												* LSB_M),
										static_cast<float>(this->Offset_M[2][0]
												* LSB_M));
								writeUART(BT2UART, cal_mag_str);
							}
							break;
						}

						// Zähler inkrementieren: (ist zugleich auch Abbruchbedingung, da nach jeder Runde inkrementiert wird und Schleife bis counter < 4 geht
						counter++;

						if (counter == 4)
							break;

					} else {
						// wird der Button nicht gedrückt, durch schnelles Aufblinken der LED die Bereitschaft signalisieren, weiter zu machen:
						ToggleLED(LED[3], 150);
					}
				}

				writeUART(BT2UART,
						"## Kalibrierung erfolgreich abgeschlossen! ##");

				// Kalibrierung fertig, zur Signalisierung, einmal kurz mit allen LEDs aufleuchten:
				{
					for (int i = 0; i < 4; i++)
						LED[i].setPins(1);
					AT(NOW() + 1 * SECONDS);
					for (int i = 0; i < 4; i++)
						LED[i].setPins(0);
				}

				// anschließend alle Offsets in UART ausgeben:

				//, Offsets ausgeben:

				this->calibration_complete = true;

			}

				// ########################### NEUE SENSORDATEN ###########################
				// zugehörige LED toggeln:
				ToggleLED(LED[0], 500);

				{
					// Accelerometer:
					int16_t temp[3] = { 0, 0, 0 };

					readSensor6Bytes(temp, CS_AG, OUT_X_XL);

					Data data;
					data.x = int(temp[0] - this->Offset_A[0]) * LSB_A;
					data.y = int(temp[1] - this->Offset_A[1]) * LSB_A;
					data.z = int(temp[2] - this->Offset_A[2]) * LSB_A;

					cbAcc.put(data);
				}
				// ******************

				// Gyroscope:
				{
					int16_t temp[3] = { 0, 0, 0 };
					readSensor6Bytes(temp, CS_AG, OUT_X_G);

					Data data;
					data.x = int(temp[0] - this->Offset_G[0]) * LSB_G;
					data.y = int(temp[1] - this->Offset_G[1]) * LSB_G;
					data.z = int(temp[2] - this->Offset_G[2]) * LSB_G;

					cbGyr.put(data);
				}
				// ******************

				// Magnetometer:
				{
					int16_t temp[3] = { 0, 0, 0 };
					readSensor6Bytes(temp, CS_AG, OUT_X_L_M);

					Data data;
					data.x =
							((((temp[0] - this->Offset_M[0][0])
									/ (this->Offset_M[0][1]
											- this->Offset_M[0][0])) * 2) - 1)
									* LSB_M;
					data.y =
							((((temp[1] - this->Offset_M[1][0])
									/ (this->Offset_M[1][1]
											- this->Offset_M[1][0])) * 2) - 1)
									* LSB_M;
					data.z =
							((((temp[2] - this->Offset_M[2][0])
									/ (this->Offset_M[2][1]
											- this->Offset_M[2][0])) * 2) - 1)
									* LSB_M;

					cbMag.put(data);
				}
				// ******************

				// Temperatur:
				{
					int16_t _temp = 0;
					readSensor2Bytes(_temp, CS_AG, OUT_TEMP_L);

					float temp = (float)((float)(_temp / 16.0f) + 25.0f);

					cbTemp.put(temp);
				}



			// mit Interval suspenden:
			suspendCallerUntil(NOW()+ this->interval*MILLISECONDS);
		}
	}
}spT; // SignalProcessingThread

//*******************************************************************************

class telemetry: public Thread, public SubscriberReceiver<Command> {
	uint interval;

	// Beinhaltet die zuletzt über das Topic gesendeten Sensorwerte
	float val_acc[3];
	float val_gyro[3];
	float val_mag[3];
	float val_temp;

	enum tel {
		all, acc, gyro, mag, temp, orient
	} pt; // legt fest, welche Telemetriedaten in einem Interval gepostet werden sollen. Wird mit 0 (all) initialisiert!

public:
	telemetry() :
			SubscriberReceiver<Command>(TopicTelemetry,
					"TopicIntervalSignalProcessingReceiver"), interval(1500), pt(
					(tel) 0) {

		val_temp = 0.0;
	}

	void put(Command& data) {
		Command* _data = (Command*) &data;

		switch (_data->id) {
		case 'T':
			this->interval = _data->value;
			break;
		case 'Q':
		case 'A':
		case 'G':
		case 'M':
		case 'H':
		case 'O':
			this->pt = (tel) (_data->value);
			break;
		}
	}

	void run(void) {
		while (1) {
			if (spT.isCalibrated()) {
				// ########################### AUSGABE TELEMETRIE ###########################
				// zugehörige LED toggeln:
				ToggleLED(LED[1], 500);

				switch ((int) this->pt) {
				case 0: // bei all einfach alle Fälle durchlaufen
				case 1: {
					// nur Acc
					const char* acc_msg =
							"Acc: x=%3.8f g, y=%3.8f g, z=%3.8f g\0";

					char acc_str[58]; // 40

					// Neuste Messwerte abrufen:
					Data data;

					cbAcc.get(data);

					sprintf(acc_str, acc_msg, data.x, data.y, data.z);

					writeUART(BT2UART, acc_str);
				}
					if (((int) this->pt) != 0)
						break;
				case 2: {
					// nur Gyro
					const char* gyro_msg =
							"Gyro: x=%3.8f dps, y=%3.8f dps, z=%3.8f dps\0";

					char gyro_str[65];

					// Neuste Messwerte abrufen:
					Data data;

					cbGyr.get(data);

					sprintf(gyro_str, gyro_msg, data.x * 180.0 / M_PI,
							data.y * 180.0 / M_PI, data.z * 180.0 / M_PI);

					writeUART(BT2UART, gyro_str);
				}
					if (((int) this->pt) != 0)
						break;

				case 3: {
					// nur Mag
					const char* mag_msg =
							"Mag: x=%3.8f gauss, y=%3.8f gauss, z=%3.8f gauss\0";

					char mag_str[70];

					// Neuste Messwerte abrufen:
					Data data;

					cbMag.get(data);

					sprintf(mag_str, mag_msg, data.x, data.y, data.z);

					writeUART(BT2UART, mag_str);
				}
					if (((int) this->pt) != 0)
						break;

				case 4: {
					// nur Temp
					const char* temp_msg = "Temp: x=%3.5f C\0";

					char temp_str[20];

					// Neuste Messung abrufen:
					float temp;

					cbTemp.get(temp);

					sprintf(temp_str, temp_msg, temp);

					writeUART(BT2UART, temp_str);
				}
					if (((int) this->pt) != 0)
						break;

				case 5: {
					// nur Orientierung:
					const char* orient_msg =
							"roll=%4.4f, pitch=%4.4f, yaw=%4.4f\0";

					char orient_str[47];

					// Neuste Daten abrufen:

					Data acc;
					Data mag;
					Data gyr;
					float roll = 0.0, pitch = 0.0, yaw = 0.0;

					// Werte ausrechnen:
					cbAcc.get(acc);
					cbMag.get(mag);
					cbGyr.get(gyr);

					calcRP_Acc(pitch, roll, acc.x, acc.y, acc.z);
					//calcY_Mag(yaw, mag.z, mag.y, mag.z);
					yaw = calcYaw(gyr.z);

					// Änderungen berechnen:
					float dp = 0.0, dr = 0.0, dy = 0.0;

					dp = cosf(roll) * cosf(pitch) * gyr.y
							- sinf(roll) * cosf(pitch) * gyr.z;
					dr = cosf(pitch) * gyr.x + sinf(roll) * sinf(pitch) * gyr.y
							+ cosf(roll) * sinf(pitch) * gyr.z;
					dy = sinf(roll) * gyr.y + cosf(roll) * gyr.z;

					// Änderungen addieren:
					pitch += dp;
					roll += dr;
					yaw += dy;

					// Werte vom Bogenmaß in Grad umwandeln:
					roll *= 180.0 / M_PI;
					pitch *= 180.0 / M_PI;
					yaw *= 180.0 / M_PI;

					// ERSTMAL AUSPROBIEREN WAS RAUSKOMMT: VERMUTUNG: ICH MÜSSTE DIE WINKEL DER VORHERIGEN ITERATION NEHMEN & IRGENDWO DAZU SPEICHERN

					sprintf(orient_str, orient_msg, roll, pitch, yaw);

					writeUART(BT2UART, orient_str);

				}
					if (((int) this->pt) != 0)
						break;
				}

				// Zeilenumbruch, um Datensätze voneinander trennen zu können
				writeUART(BT2UART, "\n");
			}
			// mit Interval suspenden:
			suspendCallerUntil(NOW()+ this->interval * MILLISECONDS);
		}
	}

}telemetryT; // Telemetry-Thread

//*******************************************************************************

struct telecommand: public Thread, public SubscriberReceiver<Telecommand> {
	telecommand() :
			SubscriberReceiver<Telecommand>(TopicTelecommand,
					"TopicTelecommands") {
	}

	void put(Telecommand& data) {
		Telecommand* _data = (Telecommand*) &data;

		Command cmd;

		// zugehörige LED toggeln:
		ToggleLED(LED[2], 1000);

		switch (_data->id) {
		case 'S':
			// Interval von SignalProcess ändern: (via Topic)
			cmd.id = 'I';
			cmd.value = _data->data;

			TopicTelemetry.publish(cmd);
			break;
		case 'T':
			cmd.id = 'T';
			cmd.value = _data->data;

			TopicTelemetry.publish(cmd);
			break;
		case 'Q':
			// alle:
			cmd.id = 'Q';
			cmd.value = 0;

			TopicTelemetry.publish(cmd);
			break;
		case 'A':
			// nur Acc:
			cmd.id = 'A';
			cmd.value = 1;

			TopicTelemetry.publish(cmd);
			break;
		case 'G':
			// nur Gyro:
			cmd.id = 'G';
			cmd.value = 2;

			TopicTelemetry.publish(cmd);
			break;
		case 'M':
			// nur Mag:
			cmd.id = 'M';
			cmd.value = 3;

			TopicTelemetry.publish(cmd);
			break;
		case 'H':
			// nur Temperatur:
			cmd.id = 'H';
			cmd.value = 4;

			TopicTelemetry.publish(cmd);
			break;
		case 'O':
			// nur Orientierung:
			cmd.id = 'O';
			cmd.value = 5;

			TopicTelemetry.publish(cmd);
			break;
		}

		// mit zugehöriger LED toggeln:
		ToggleLED(LED[2], 500);
	}

	void run(void) {
		while (1) {
			// mit zugehöriger LED toggeln:
			ToggleLED(LED[2], 500);
			suspendCallerUntil(NOW()+ 500 * MILLISECONDS);
		}
	}
}tcT; // TelecommandoThread

//*******************************************************************************

// Validiert ein potenzielles Telekommando (StateMachine)
struct TCDecoder {
	// Liefert ID des in UART geschriebenen Strings
	char getId(const char *cmd) {
		return cmd[1];
	}

	// Liefert Data-Block des in UART geschriebenen Strings
	float getData(char* cmd, int length) {
		// Herausfinden, wie lange <data> ist:
		//const int len = strlen(cmd) - 2; // 3 Zeichen gehören nicht dazu (-1 weil nullbasiert!)
		char* ptr = &cmd[2];

		char _data[length];			// array mit Länge len

		//strncpy(_data, &cmd[3], length); // Nur Data kopieren
		for (int i = 0; i < length; i++)
			_data[i] = *ptr++;

		return atof(_data);			// in float konvertieren und zurückgeben
	}

	// prüft ob ein gültiges Telekommando übergeben wurde und gibt dann true zurück (sonst false)
	bool validate(char *str) {
		const uint8_t _len = strlen(str); // Gesamtlänge des Strings ermitteln

		if (_len < 4) // 4 Bytes sind mindestens für TC erforderlich!
			return false;

		// prüfen ob erstes und letztes Zeichen korrekt sind:
		if ((str[0] != '$') & (str[_len - 1] != '#')) {
			// Abbrechen
			return false;
		}

		// Prüfen ob ID ein Buchstabe ist: A-Z
		if (isalpha(str[1]) == 0) // prüft ob id ein Buchstabe ist oder nicht (==0)
			return false;

		// Prüfen ob data in Gleitkommazahl konvertiert werden kann:
		// Festlegung: atof gibt 0.0 zurück, falls keine Konvertierung vorgenommen werden konnte, daher dann mit dieser Zahl weiterarbeiten

		// ist bis hier kein Fehler aufgetreten, das Telekommando validieren:
		return true;
	}
} StateDecoder;

//*******************************************************************************

// Liest aus UART Interface und versucht zu parsen ob es sich um Telekommando handelt
struct uartWatcher: public Thread {
	HAL_UART* uart;

	uartWatcher(HAL_UART* stream) :
			uart(stream) {
	}

	void init(void) {
		uart->init(); // default 115200
		uart->config(UART_PARAMETER_ENABLE_DMA, 1); // Direct Memory Access aktivieren (DMA)
	}

	void run(void) {
		while (1) {
			// Liest aus UART und prüft ob Telekommando gefunden wurde:

			char str[50] = { '\0' };
			char* ptr = &str[0];
			int len = 0;
			bool read = false;
			uint64_t lasttime = 0, sum = 0;

			// liest nur wenn Daten zur Verfügung stehen...
			if (uart->isDataReady()) {
				// ... und prüft ob noch einzelne chars im Buffer sind SOWIE die Max. Zeit 5 Sekunden nicht überschritten wurden
				while ((uart->status(UART_STATUS_RX_BUF_LEVEL))
						&& (sum < 5000 * MILLISECONDS)) {
					lasttime = NOW(); // aktuelle Zeit speichern
					*ptr++ = uart->getcharNoWait();// Array schrittweise befüllen
					len++;
					read = true;// Sicherheitstoken damit nicht doppelte Ausgabe erfolgt

					sum += NOW() - lasttime;// Lesezeit herausfinden und aufaddieren
				}
			} else {
				uart->suspendUntilDataReady(); // wenn keine Daten mehr zur Verfügung stehen, suspenden
				suspendCallerUntil(NOW()+ 250 * MILLISECONDS);// kurz suspenden, damit auch andere Threads ausgeführt werden können
			}

			if (read) {
				// Hier: es wurde etwas eingelesen und kann verarbeitet werden

				if (sum >= 5000 * MILLISECONDS) {
					// Zeitüberschreitung:
					uart->write("\nMessage Timeout!\n", 18); // Zeitüberschreitung mitteilen
				} else if (StateDecoder.validate(str)) {
					// eigentlicher Fall: In der Zeit gelesen, dann jetzt: verarbeiten

					//uart->write("\nMessage Received!\n", 19);

					Telecommand tc;
					tc.id = StateDecoder.getId(str);
					tc.data = StateDecoder.getData(str, len - 3);

					TopicTelecommand.publish(tc, true); // neues Telekommando in Topic posten (Receiver verarbeitet es weiter)
				} else {
					uart->write("\nMessage Corrupted!\n", 20);
				}

				// für Debugging entkommentieren:
				//uart->write("\nAusgabe:\n", 10);
				//uart->write(str, len);
			}

			sum = 0; // Zeit zurücksetzen
			read = false; // Lesebestätigung zurücksetzen
		}
	}
};
uartWatcher reader = uartWatcher(&BT2UART);

//*******************************************************************************

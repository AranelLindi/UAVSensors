/*
 * TELEKOMMANDOS
 * S Zahl 	: 	Ändert Interval SignalProcessing-Thread
 * T Zahl	: 	Ändert Interval Telemetrie-Thread
 * Q		: 	postet ALLE Telemetrie-Daten
 * A		:	postet nur Accelerometer
 * G		:	postet nur Gyroskop
 * M		:	postet nur Magnetometer
 * H		:	postet nur Temperatur
 * O		:	postet nur Orientierungswinkel
 * F		: 	ändert den Wert des Filters alpha
 * W		:	Orientierungswinkel zurücksetzen (alle) // noch nicht implementiert
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

struct Euler {
	float roll;
	float pitch;
	float yaw;
};

/// Globale Variablen
/// Hardware
HAL_GPIO button(GPIO_000); // blauer Button
// Definition LEDs:
HAL_GPIO led_green(GPIO_060);
HAL_GPIO led_orange(GPIO_061);
HAL_GPIO led_red(GPIO_062);
HAL_GPIO led_blue(GPIO_063);

/*
 * grün = SignalProcessing
 * orange = Telemetry
 * rot = Telecommand
 * blau = Kalibrierung
 */

HAL_UART BT2UART(UART_IDX2); // UART (Tx=PD5, Rx=PD6)
HAL_SPI IMU(SPI_IDX1); // SPI: SDA: PB5, SCL: PB3, Sensoren auf BP4

// Sensoren:
HAL_GPIO CS_AG(GPIO_006); // IMU Chip Pin für Gyro und Accelerometer
HAL_GPIO CS_M(GPIO_041); // IMU Chip Pin für Magnometer

// LSB: Mit Integerwert von jeweiligem Sensor multiplizieren um realen Sensorwert zu erhalten
const float LSB_G = 70e-3f;
const float LSB_A = 0.061e-3f;
const float LSB_M = 0.14e-3f;

const float rad2deg = 180.0f / M_PI;
const float deg2rad = M_PI / 180.0f;

/// Intertask
// UART -> Interpreter/StateMachine
Topic<Telecommand> TopicTelecommand(-1, "TopicTelecommand");

// Interpreter/StateMachine -> Counter-Thread
Topic<Command> TopicTelemetry(-1, "TopicTelemetry");

CommBuffer<Data> cbAcc; // für Daten des Accelerometers
CommBuffer<Data> cbMag; // für Daten des Magnometers
CommBuffer<Data> cbGyr; // für Daten des Gyroskops
CommBuffer<Euler> cbOrie; // für Orientationswinkel
CommBuffer<float> cbTemp; // für Temperatur

/// Statische Funktionen
// Schaltet LED an und nach Wartezeit wieder aus
static void ToggleLED(HAL_GPIO& led, uint32_t length_ms) {
	led.setPins(1);
	AT(NOW() + length_ms * MILLISECONDS);
	led.setPins(0);
}

// Schreibt einen String in UART:
static void write2UART(const char* string) {
	const int len = strlen(string);
	BT2UART.write(string, len);
}

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
		write2UART("FEHLER INITIALISIERUNG!");

	// Kommunikation schließen:
	pin.setPins(1);
}

// Initialisiert Magnetometer:
static void initM(HAL_SPI& imu, HAL_GPIO& pin) {
	// Kommunikation initieren:
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
		write2UART("FEHLER INITIALISIERUNG!");

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
		write2UART("ERROR READING 3 BYTES!");

	int16_t temperature = (int16_t) ((temp[2] << 8) | temp[1]);

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
		write2UART("ERROR READING 7 BYTES!");

	int16_t x = 0, y = 0, z = 0; // kombiniert je zwei 8-Bit-Register zu 16-Bit Ganzzahl
	x = (int16_t) ((temp[2] << 8) | temp[1]);
	y = (int16_t) ((temp[4] << 8) | temp[3]);
	z = (int16_t) ((temp[6] << 8) | temp[5]);

	// Ganzzahlen später noch mit jeweiligem LSB multiplizieren um Dezimalwert zu erhalten:
	arr[0] = x;
	arr[1] = y;
	arr[2] = z;

	// Kommunikation schließen:
	pin.setPins(1);
}

//*******************************************************************************

// Liest Messdaten aus Sensor & Kalibriert diesen bzw. sendet Daten über Buffer
class SignalProcessing: public Thread, public SubscriberReceiver<Command> {
	uint interval; // Standard: 300 ms

	// Accelerometer
	float Offset_A[3]; // Offset Beschleunigungssensor
	float Value_A[3]; // Speichert zuletzt gemessene Beschleunigungsdaten pro Achse
	// Gyroscope
	float Offset_G[3]; // Offset Gyroskop
	float Value_G[3]; // Speichert zuletzt gemessene Daten des Gyroskops pro Achse
	// Magnetometer
	float Offset_M[3][2]; // zwei Werte für jede Achse: Min[0]/Max[1] Wert der Kalibrierung!
	float Value_M[3]; // Speichert zuletzt gemessene Daten des Magnetometers pro Achse

	float Value_Temp; // Speichert zuletzt gemessene Temperatur, ausnahmsweise in Float und nicht Ganzzahl speichern, da kein Offset und keine Kalibrierung nötig

	float roll_am, pitch_am, yaw_am; // Orientierungswinkel (Accelerometer & Magnometer)
	float roll_g, pitch_g, yaw_g; // Orientierungswinkel (Gyroskop)

	Data data_accel, data_gyro, data_magno; // für Übertragung in Topic (sicherer wenn in größerem Scope!)
	Euler data_angles;

	bool calibration_complete; // legt fest ob Kalibrierung der IMU erfolgt ist

	float alpha; // Filterkonstante (änderbar per TC!)

public:
	SignalProcessing() :
			SubscriberReceiver<Command>(TopicTelemetry,
					"TopicIntervalSignalProcessingReceiver"), interval(200) { // abonniert Interval-Topic
		// Standart: 300 ms
		calibration_complete = false;
		Value_Temp = 0.0;
		roll_am = 0.0, pitch_am = 0.0, yaw_am = 0.0, roll_g = 0.0, pitch_g =
				0.0, yaw_g = 0.0, alpha = 0.95; // Standardwert: 0.95
	}

	bool isCalibrated() {
		return this->calibration_complete;
	}

	void init(void) {
		// zugehörige LEDs auf Ausgang schalten:
		led_green.init(1, 1, 0); // Aufgabenstellung (grün)
		led_blue.init(1, 1, 0); // Kalibrierung (blau)

		// Button auf Eingang schalten:
		button.init();

		// Initialisieren: IMU:
		IMU.init(1000000); // standard-baudrate == 1000000 // Wichtig! Muss vor Initialisierung der Sensoren ausgeführt werden!

		// Initialisieren: Beschleunigungssensor & Gyroskop:
		CS_AG.init(true, 1, 1); // Init Gyro & Accelerometer
		initAG(IMU, CS_AG); // Einstellungen in Register schreiben
		write2UART(
				"Initialisierung Beschleunigungssensor und Gyroskop abgeschlossen...");

		// Initialisieren: Magnetometer:
		CS_M.init(true, 1, 1); // Init Magnometer
		initM(IMU, CS_M); // Einstellungen in Register schreiben
		write2UART("Initialisierung Magnetometer abgeschlossen...");

		// ALLE Variablen mit Null initialisieren:
		Offset_A[0] = 0.0, Offset_A[1] = 0.0, Offset_A[2] = 0.0;
		Offset_M[0][0] = 0.0, Offset_M[0][1] = 0.0;
		Offset_M[1][0] = 0.0, Offset_M[1][1] = 0.0;
		Offset_M[2][0] = 0.0, Offset_M[2][1] = 0.0;
		Offset_G[0] = 0.0, Offset_G[1] = 0.0, Offset_G[2] = 0.0;

		Value_A[0] = 0.0, Value_A[1] = 0.0, Value_A[2] = 0.0;
		Value_G[0] = 0.0, Value_G[1] = 0.0, Value_G[2] = 0.0;
		Value_M[0] = 0.0, Value_M[0] = 0.0, Value_M[2] = 0.0;
	}

	void put(Command& data) {
		Command* _data = (Command*) &data;

//		write2UART("Ich bin hier");

		// Interval ändern:
		switch (_data->id) {
		case 'I':
			this->interval = (uint) (_data->value);
			break;
		case 'C':
			// Festlegen ob Kalibrierung erfolgt ist oder neu gemacht werden soll:
			this->calibration_complete = (_data->value == 1.0 ? true : false);
			break;
		case 'n': /* Offset: Magnetometer: x_min */
			this->Offset_M[0][0] = _data->value;
			break;
		case 'p': /* Offset: Magnetometer: y_min */
			this->Offset_M[1][0] = _data->value;
			break;
		case 'q': /* Offset: Magnetometer: z_min */
			this->Offset_M[2][0] = _data->value;
			break;
		case 'r': /* Offset: Magnetometer: x_max */
			this->Offset_M[0][1] = _data->value;
			break;
		case 's': /* Offset: Magnetometer: y_max */
			this->Offset_M[1][1] = _data->value;
			break;
		case 't': /* Offset: Magnetometer: z_max */
			this->Offset_M[2][1] = _data->value;
			break;
		case 'u': /* Offset: Gyroskop: x */
			this->Offset_G[0] = _data->value;
			break;
		case 'v': /* Offset: Gyroskop: y */
			this->Offset_G[1] = _data->value;
			break;
		case 'w': /* Offset: Gyroskop: z */
			this->Offset_G[2] = _data->value;
			break;
		case 'x': /* Offset: Accelerometer: x */
			this->Offset_A[0] = _data->value;
			break;
		case 'y': /* Offset: Accelerometer: y */
			this->Offset_A[1] = _data->value;
			break;
		case 'z': /* Offset: Accelerometer: z */
			this->Offset_A[2] = _data->value;
			break;
		case 'Z': /* Standardeinstellungen Kalibrierung (Offsetfehler) */
			this->Offset_M[0][0] = -1.00758014;
			this->Offset_M[0][1] = 0;
			this->Offset_M[1][0] = 0;
			this->Offset_M[1][1] = 3.27418017;
			this->Offset_M[2][0] = 0;
			this->Offset_M[2][1] = 0.9354799;

			this->Offset_G[0] = 2.7104001 * deg2rad; // wurden noch in dps aufgezeichnet!
			this->Offset_G[1] = 1.56309998 * deg2rad;
			this->Offset_G[2] = -0.754599988 * deg2rad;

			this->Offset_A[0] = 0.0205;
			this->Offset_A[1] = -0.0035;
			this->Offset_A[2] = 0.02550;
			break;
		case 'F':
			this->alpha = _data->value;
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
						ToggleLED(led_blue, 500); // blau
						AT(NOW() + 500 * MILLISECONDS); // kurz warten bevor kalibriert wird um nicht irgendwie nächste Schleifeniteration auch gleich zu aktivieren

						// Reihenfolge jeweils: x, y, z
						// Unterscheiden, in welcher Kalibrierungsphase wir sind (1-3)
						switch (counter) {
						case 1: {
							// Accelerometer
							write2UART("Accelerometer wird kalibriert:\n");

							int16_t temp[3] = { 0, 0, 0 }; // Offset-Werte: summiert auf, muss daher größer als 16 Bits sein!
							int64_t P1[3] = { 0, 0, 0 }, P2[3] = { 0, 0, 0 },
									P3[3] = { 0, 0, 0 };

							// Vorgehensweise:
							// erst z-, dann x-, zuletzt y-Achse kalibrieren! Dies dauert jeweils 1 s, dazwischen eine weitere Sekunde warten um Board in Position zu bringen
							ToggleLED(led_blue, 250);

							// ######################################################
							// x = 0, y = 0, z = 1*g
							write2UART("z-Achse wird kalibriert...\n");
							AT(NOW() + 1 * SECONDS);
							for (uint i = 0; i < 100; i++) {
								readSensor6Bytes(temp, CS_AG, OUT_X_XL);
								P3[0] += temp[0]; // x-Achse (0g)
								P3[1] += temp[1]; // y-Achse (0g)
								P3[2] += temp[2]; // z-Achse (1g)
								AT(NOW() + 100 * MILLISECONDS); // Nach jeder Messung kurz warten, bis die nächste aufgenommen wird
							}
							// ######################################################

							ToggleLED(led_blue, 500);
							AT(NOW() + 1 * SECONDS);
							ToggleLED(led_blue, 500);

							// ######################################################
							// x = 1*g, y = 0, z = 0
							write2UART("x-Achse wird kalibriert...\n");
							AT(NOW() + 1 * SECONDS);
							for (uint i = 0; i < 100; i++) {
								readSensor6Bytes(temp, CS_AG, OUT_X_XL);
								P1[0] += temp[0]; // x-Achse (1g)
								P1[1] += temp[1]; // y-Achse (0g)
								P1[2] += temp[2]; // z-Achse (0g)
								AT(NOW() + 100 * MILLISECONDS); // Nach jeder Messung kurz warten, bis die nächste aufgenommen wird
							}
							// ######################################################

							ToggleLED(led_blue, 500);
							AT(NOW() + 1 * SECONDS);
							ToggleLED(led_blue, 500);

							// ######################################################
							// x = 0, y = 1*g, z = 0
							write2UART("y-Achse wird kalibriert...\n");
							AT(NOW() + 1 * SECONDS);
							for (uint i = 0; i < 100; i++) {
								readSensor6Bytes(temp, CS_AG, OUT_X_XL);
								P2[0] += temp[0]; // x-Achse (0g)
								P2[1] += temp[1]; // y-Achse (0g)
								P2[2] += temp[2]; // z-Achse (1g)
								AT(NOW() + 100 * MILLISECONDS); // Nach jeder Messung kurz warten, bis die nächste aufgenommen wird
							}
							// ######################################################

							ToggleLED(led_blue, 500);
							AT(NOW() + 1 * SECONDS);

							// Werte ausrechnen:
							// Dabei vernachlässigen das Nachkommastelle gerundet wird, da es sich um sehr große Zahlen handert, dürfte das Ergebnis nur sehr gering vom exakten Wert abweichen:
							Offset_A[0] = ((P2[0] + P3[0]) / 200.0) * LSB_A; // Offset x-Achse
							Offset_A[1] = ((P1[1] + P3[1]) / 200.0) * LSB_A; // Offset y-Achse
							Offset_A[2] = ((P1[2] + P2[2]) / 200.0) * LSB_A; // Offset z-Achse

							// Abschluss signalisieren:
							ToggleLED(led_blue, 500);
							AT(NOW() + 1 * SECONDS);
							ToggleLED(led_blue, 500);

							write2UART(
									"-- Kalibrierung Beschleunigungssensor abgeschlossen... --");
							{
								const char* cal_acc_msg =
										"Acc: Offset-X=%2.5f g, Offset-Y=%2.5f g, Offset-Z=%2.5f g\0";
								char cal_acc_str[67];
								sprintf(cal_acc_str, cal_acc_msg,
										this->Offset_A[0], this->Offset_A[1],
										this->Offset_A[2]);
								write2UART(cal_acc_str);
							}
						}
							break;

						case 2: {
							// Gyroskope
							write2UART("Gyroskop wird kalibriert:\n");

							double data[3] = { 0, 0, 0 }; // Offset-Werte: summiert auf, muss daher größer als 16 Bits sein!
							const unsigned int N = 100;

							// Vorgehensweise:
							// erst z-, dann x-, zuletzt y-Achse kalibrieren! Dies dauert jeweils 1 s, dazwischen eine weitere Sekunde warten um Board in Position zu bringen
							ToggleLED(led_blue, 500);

							//int64_t P1[3] = { 0, 0, 0 }, P2[3] = { 0, 0, 0 }, P3[3] = { 0, 0, 0 };

							// ######################################################
							// z-Achse
							write2UART("z-Achse wird kalibriert...\n");
							AT(NOW() + 1 * SECONDS);
							int16_t temp_z = 0;
							for (uint i = 0; i < N; i++) {
								readSensor2Bytes(temp_z, CS_AG, OUT_Z_G);
								data[2] += ((float) (temp_z * LSB_G)); // z-Achse
								AT(NOW() + 100 * MILLISECONDS); // Nach jeder Messung kurz warten, bis die nächste aufgenommen wird
							}
							// ######################################################

							ToggleLED(led_blue, 500);
							AT(NOW() + 1 * SECONDS);
							ToggleLED(led_blue, 500);

							// ######################################################
							// x-Achse
							write2UART("x-Achse wird kalibriert...\n");
							AT(NOW() + 1 * SECONDS);
							int16_t temp_x = 0;
							for (uint i = 0; i < N; i++) {
								readSensor2Bytes(temp_x, CS_AG, OUT_X_G);
								data[0] += ((float) (temp_x * LSB_G)); // x-Achse
								AT(NOW() + 100 * MILLISECONDS); // Nach jeder Messung kurz warten, bis die nächste aufgenommen wird
							}
							// ######################################################

							ToggleLED(led_blue, 500);
							AT(NOW() + 1 * SECONDS);
							ToggleLED(led_blue, 500);

							// ######################################################
							// y-Achse
							write2UART("y-Achse wird kalibriert...\n");
							AT(NOW() + 1 * SECONDS);
							int16_t temp_y = 0;
							for (uint i = 0; i < N; i++) {
								readSensor2Bytes(temp_y, CS_AG, OUT_Y_G);
								data[1] += ((float) (temp_y * LSB_G)); // y-Achse
								AT(NOW() + 100 * MILLISECONDS); // Nach jeder Messung kurz warten, bis die nächste aufgenommen wird
							}
							// ######################################################

							ToggleLED(led_blue, 500);
							AT(NOW() + 1 * SECONDS);

							// Werte ausrechnen:
							Offset_G[0] = (float) (data[0] / N) * deg2rad; // Offset x-Achse
							Offset_G[1] = (float) (data[1] / N) * deg2rad; // Offset y-Achse
							Offset_G[2] = (float) (data[2] / N) * deg2rad; // Offset z-Achse

							// Abschluss signalisieren:
							ToggleLED(led_blue, 500);
							AT(NOW() + 1 * SECONDS);
							ToggleLED(led_blue, 500);

							write2UART(
									"-- Kalibrierung Gyroskop abgeschlossen... --");
							{
								const char* cal_gyro_msg =
										"Gyro: Offset-X=%2.3f dps, Offset-Y=%2.3f dps, Offset-Z=%2.3f dps";
								char cal_gyro_str[68];
								sprintf(cal_gyro_str, cal_gyro_msg,
										this->Offset_G[0], this->Offset_G[1],
										this->Offset_G[2]);
								write2UART(cal_gyro_str);
							}
						}
							break;

						case 3: {
							// Magnetometer
							write2UART("Magnetometer wird kalibriert:\n");

							const unsigned int N = 100;

							// Vorgehensweise:
							// erst z-, dann x-, zuletzt y-Achse kalibrieren!
							ToggleLED(led_blue, 500);

							// ######################################################
							{
								write2UART("z-Achse wird kalibriert...\n");
								AT(NOW() + 1 * SECONDS);
								int16_t currVal; // immer der aktuell gemessene Wert
								float& M_z_min = this->Offset_M[2][0];
								float& M_z_max = this->Offset_M[2][1];

								int16_t M_c_Z[2] = { 0, 0 }; // speichert über Kalibrierung den gemessenen Min- ([0]) und Maxwert ([1])

								for (unsigned int i = 0; i < N; i++) {
									readSensor2Bytes(currVal, CS_M, OUT_Z_L_M);

									// prüfen ob der Wert gespeichert werden soll:
									if (currVal <= M_c_Z[0])
										M_c_Z[0] = currVal;

									if (currVal >= M_c_Z[1])
										M_c_Z[1] = currVal;
									AT(NOW() + 100 * MILLISECONDS);
								}

								// In M_c befinden sich jetzt der Min/Max-Wert für Z-Achse, diesen speichern:
								M_z_min = (float) (M_c_Z[0] * LSB_M);
								M_z_max = (float) (M_c_Z[1] * LSB_M);
							}
							// ######################################################

							ToggleLED(led_blue, 500);
							AT(NOW() + 1 * SECONDS);
							ToggleLED(led_blue, 500);

							// ######################################################
							{
								write2UART("x-Achse wird kalibriert...\n");
								AT(NOW() + 1 * SECONDS);
								int16_t currVal; // immer der aktuell gemessene Wert
								float& M_x_min = this->Offset_M[0][0];
								float& M_x_max = this->Offset_M[0][1];

								int16_t M_c_X[2] = { 0, 0 };

								for (unsigned int i = 0; i < N; i++) {
									// Messung abrufen:
									readSensor2Bytes(currVal, CS_M, OUT_X_L_M);

									// prüfen ob der Wert gespeichert werden soll:
									if (currVal <= M_c_X[0])
										M_c_X[0] = currVal;

									if (currVal >= M_c_X[1])
										M_c_X[1] = currVal;

									AT(NOW() + 100 * MILLISECONDS);
								}

								// In M_c befinden sich jetzt der Min/Max-Wert für X-Achse, diesen speichern:
								M_x_min = (float) (M_c_X[0] * LSB_M);
								M_x_max = (float) (M_c_X[1] * LSB_M);
							}
							// ######################################################

							ToggleLED(led_blue, 500);
							AT(NOW() + 1 * SECONDS);
							ToggleLED(led_blue, 500);

							// ######################################################
							{
								write2UART("y-Achse wird kalibriert...\n");
								AT(NOW() + 1 * SECONDS);
								int16_t currVal; // immer der aktuell gemessene Wert
								float& M_y_min = this->Offset_M[1][0];
								float& M_y_max = this->Offset_M[1][1];

								int16_t M_c_Y[2] = { 0, 0 };

								for (unsigned int i = 0; i < N; i++) {
									// Messung abrufen:
									readSensor2Bytes(currVal, CS_M, OUT_Y_L_M);

									// prüfen ob der Wert gespeichert werden soll:
									if (currVal <= M_c_Y[0])
										M_c_Y[0] = currVal;

									if (currVal >= M_c_Y[1])
										M_c_Y[1] = currVal;
									AT(NOW() + 100 * MILLISECONDS);
								}

								// In M_c befinden sich jetzt der Min/Max-Wert für Y-Achse, diesen speichern:
								M_y_min = (float) (M_c_Y[0] * LSB_M);
								M_y_max = (float) (M_c_Y[1] * LSB_M);
							}
							// ######################################################

							ToggleLED(led_blue, 500);
							AT(NOW() + 1 * SECONDS);

							// Abschluss signalisieren:
							ToggleLED(led_blue, 500);
							AT(NOW() + 1 * SECONDS);
							ToggleLED(led_blue, 500);

							write2UART(
									"-- Kalibrierung Magnetometer abgeschlossen... --");
							{
								const char* cal_mag_msg =
										"Mag: Offset-X_max=%4.5f gauss, Offset-Y_max=%4.5f gauss, Offset-Z_max=%4.5f gauss\nOffset-X_min=%4.5f gauss, Offset-Y_min=%4.5f gauss, Offset-Z_min=%4.5f gauss\0";
								char cal_mag_str[189];
								sprintf(cal_mag_str, cal_mag_msg,
										this->Offset_M[0][1],
										this->Offset_M[1][1],
										this->Offset_M[2][1],
										this->Offset_M[0][0],
										this->Offset_M[1][0],
										this->Offset_M[2][0]);
								write2UART(cal_mag_str);
							}
						}
							break;
						}

						// Zähler inkrementieren: (ist zugleich auch Abbruchbedingung, da nach jeder Runde inkrementiert wird und Schleife bis counter < 4 geht
						counter++;

						if (counter == 4)
							break;

					} else {
						// wird der Button nicht gedrückt, durch schnelles Aufblinken der LED die Bereitschaft signalisieren, weiter zu machen:
						ToggleLED(led_blue, 200);
						if (this->calibration_complete)
							break;
					}

					//if (this->calibration_complete)
					//	break;
				}

				write2UART("## Kalibrierung erfolgreich abgeschlossen! ##");

				this->calibration_complete = true;
				continue;
			}
			// ########################### NEUE SENSORDATEN ###########################
			// zugehörige LED toggeln:
			ToggleLED(led_green, 200);

			{
				// Accelerometer:
				int16_t temp[3] = { 0, 0, 0 };

				readSensor6Bytes(temp, CS_AG, OUT_X_XL);

				//Data data;
				data_accel.x = temp[0] * LSB_A - this->Offset_A[0];
				data_accel.y = temp[1] * LSB_A - this->Offset_A[1];
				data_accel.z = temp[2] * LSB_A - this->Offset_A[2];

				cbAcc.put(data_accel);
			}
			// ******************

			// ******************
			// Gyroscope:
			{
				int16_t temp[3] = { 0, 0, 0 };
				readSensor2Bytes(temp[0], CS_AG, OUT_X_G);
				readSensor2Bytes(temp[1], CS_AG, OUT_Y_G);
				readSensor2Bytes(temp[2], CS_AG, OUT_Z_G);
				//readSensor6Bytes(temp, CS_AG, OUT_X_G);

				//Data data;
				data_gyro.x = temp[0] * LSB_G * deg2rad - this->Offset_G[0]; // Einheit: rad/s
				data_gyro.y = temp[1] * LSB_G * deg2rad - this->Offset_G[1]; // Einheit: rad/s
				data_gyro.z = temp[2] * LSB_G * deg2rad - this->Offset_G[2]; // Einheit: rad/s

				cbGyr.put(data_gyro);
			}
			// ******************

			// Magnetometer:
			{
				int16_t temp[3] = { 0, 0, 0 };
				//readSensor6Bytes(temp, CS_M, OUT_X_L_M);
				readSensor2Bytes(temp[0], CS_M, OUT_X_L_M);
				readSensor2Bytes(temp[1], CS_M, OUT_Y_L_M);
				readSensor2Bytes(temp[2], CS_M, OUT_Z_L_M);

				// Referenzen für Übersichtlichkeit:
				const float& M_x_min = this->Offset_M[0][0];
				const float& M_x_max = this->Offset_M[0][1];

				const float& M_y_min = this->Offset_M[1][0];
				const float& M_y_max = this->Offset_M[1][1];

				const float& M_z_min = this->Offset_M[2][0];
				const float& M_z_max = this->Offset_M[2][1];

				//Data data;
				data_magno.x = ((((float) (temp[0] * LSB_M) - M_x_min)
						/ (M_x_max - M_x_min)) * 2) - 1;
				data_magno.y = ((((float) (temp[1] * LSB_M) - M_y_min)
						/ (M_y_max - M_y_min)) * 2) - 1;
				data_magno.z = ((((float) (temp[2] * LSB_M) - M_z_min)
						/ (M_z_max - M_z_min)) * 2) - 1;

			cbMag.put(data_magno);
			}
			// ******************

			// Temperatur:
			{
				int16_t _temp = 0;
				readSensor2Bytes(_temp, CS_AG, OUT_TEMP_L);

				float temp = ((_temp / 16.0f) + 25.0f);

				cbTemp.put(temp);
			}

			// Orientierungswinkel:
			{
				// ############################# BERECHNUNGEN ###############################
				// Änderungen für Winkel durch Gyro berechnen:
				float dp = 0.0, dr = 0.0, dy = 0.0;

				dp = data_gyro.y * cosf(roll_g) - data_gyro.z * sinf(roll_g);
				dr = data_gyro.x + data_gyro.y * sinf(roll_g) * tanf(pitch_g)
						+ data_gyro.z * cosf(roll_g) * tanf(pitch_g);
				dy = data_gyro.x * sinf(roll_g) / cosf(pitch_g)
						+ data_gyro.z * cosf(roll_g) / cosf(pitch_g);

				// Änderungen für Gyro-Winkel aufaddieren:
				pitch_g += dp * (float) (this->interval / 1000.0f); // Interval in Sekunden umrechnen und damit Multiplizieren
				roll_g += dr * (float) (this->interval / 1000.0f);
				yaw_g += dy * (float) (this->interval / 1000.0f);

				// Neue Winkel aus Accelerometer & Magnometer berechnen:
				// ROLL & PITCH
				pitch_am = atan2f(data_accel.x,
						sqrt(
								data_accel.y * data_accel.y
										+ data_accel.z * data_accel.z));
				roll_am = atan2f(data_accel.y,
						sqrt(
								data_accel.x * data_accel.x
										+ data_accel.z * data_accel.z));

				// Heading: YAW
				const float M_x_h = data_magno.x * cosf(pitch_am)
						+ data_magno.z * sinf(pitch_am);
				const float M_y_h = data_magno.x * sinf(roll_am)
						* sinf(pitch_am) + data_magno.y * cosf(roll_am)
						- data_magno.z * sinf(roll_am) * cosf(pitch_am);
				yaw_am = atan2f(M_y_h, M_x_h);

				// Winkel in richtiges Interval mappen:
				if (roll_am > 2 * M_PI)
					roll_am -= 2 * M_PI;
				if (pitch_am > 2 * M_PI)
					pitch_am -= 2* M_PI;
				if (yaw_am > 2* M_PI)
					yaw_am -= 2 * M_PI;

				if (roll_g > 2 * M_PI)
					roll_g -= 2*M_PI;
				if (pitch_g > 2 * M_PI)
					pitch_g -= 2 * M_PI;
				if (yaw_g > 2 * M_PI)
					yaw_g -= 2 * M_PI;


				// resultierende Berechnung:
				// Werte vom Bogenmaß in Grad umwandeln und gem. Filter aufaddieren
				const float roll_res = alpha * roll_g * rad2deg
						+ (1 - alpha) * roll_am * rad2deg;
				const float pitch_res = alpha * pitch_g * rad2deg
						+ (1 - alpha) * pitch_am * rad2deg;
				const float yaw_res = alpha * yaw_g * rad2deg
						+ (1 - alpha) * yaw_am * rad2deg; // Yaw wird nur von Accelerometer & Magnometer berechnet

						// Gem. Filterplan: Gefilterte Winkel dem Gyroskop rückmelden: (Wichtig: Ins Bogenmaß konvertieren!)
				roll_g = roll_res * deg2rad;
				pitch_g = pitch_res * deg2rad;
				yaw_g = yaw_res * deg2rad;

				// Winkel als Grad übertragen:
				data_angles.roll = roll_res;
				data_angles.pitch = pitch_res;
				data_angles.yaw = yaw_res;

				cbOrie.put(data_angles);
			}
			// mit Interval suspenden:
			suspendCallerUntil(NOW()+ this->interval*MILLISECONDS);
		}
	}
}spT; // SignalProcessingThread

//*******************************************************************************

// Sendet gemäß den Usereinstellungen, angeforderte Telemetry-Daten über UART
class telemetryThread: public Thread, public SubscriberReceiver<Command> {
	uint interval;

	// Beinhaltet die zuletzt über das Topic gesendeten Sensorwerte
	//float val_acc[3];
	//float val_gyro[3];
	//float val_mag[3];

	enum tel {
		all, acc, gyro, mag, temp, orient
	} pt; // legt fest, welche Telemetriedaten in einem Interval gepostet werden sollen. Wird mit 0 (all) initialisiert!

public:
	telemetryThread() :
			SubscriberReceiver<Command>(TopicTelemetry,
					"TopicIntervalSignalProcessingReceiver"), interval(1000), pt(
					(tel) 0) {
		// Orientierungswinkel initialisieren und auf Null setzen:
	}

	void init() {
		led_orange.init(1, 1, 0);
	}

	void put(Command& data) {
		Command* _data = (Command*) &data;

		// korrekt so, da ab Q alles gleich behandelt wird
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
				// Neue Daten abholen:
				Data acc_data, gyr_data, mag_data;
				Euler angles;
				float temperature;
				cbAcc.get(acc_data);
				cbGyr.get(gyr_data);
				cbMag.get(mag_data);
				cbOrie.get(angles);
				cbTemp.get(temperature);

				// ########################### AUSGABE TELEMETRIE ###########################
				// zugehörige LED toggeln:
				ToggleLED(led_orange, 200);				// orange

				switch ((int) this->pt) {
				case 0: // bei all einfach alle Fälle durchlaufen
				case 1: {
					// nur Acc
					const char* acc_msg = "$a%2.4f,%2.4f,%2.4f#";
					//"Acc: x=%2.4f g, y=%2.4f g, z=%2.4f g\0";

					char acc_str[26]; // 43

					sprintf(acc_str, acc_msg, acc_data.x, acc_data.y,
							acc_data.z);

					write2UART(acc_str);
				}
					if (((int) this->pt) != 0)
						break;
				case 2: {
					// nur Gyro
					const char* gyro_msg = "$g%3.4f,%3.4f,%3.4f#";
					//"Gyro: x=%5.5f dps, y=%5.5f dps, z=%5.5f dps\0";

					char gyro_str[29]; // 62

					sprintf(gyro_str, gyro_msg, gyr_data.x * rad2deg,
							gyr_data.y * rad2deg, gyr_data.z * rad2deg);

					write2UART(gyro_str);
				}
					if (((int) this->pt) != 0)
						break;

				case 3: {
					// nur Mag
					const char* mag_msg = "$m%2.4f,%2.4f,%2.4f#";
					//"Mag: x=%4.5f gauss, y=%4.5f gauss, z=%4.5f gauss\0";

					char mag_str[26]; // 64

					sprintf(mag_str, mag_msg, mag_data.x, mag_data.y,
							mag_data.z);

					write2UART(mag_str);
				}
					if (((int) this->pt) != 0)
						break;

				case 4: {
					// nur Temp
					const char* temp_msg = "$h%3.3f#\0";
					//"Temp: x=%2.2f C\0";

					char temp_str[11]; //16

					sprintf(temp_str, temp_msg, temperature);

					write2UART(temp_str);
				}
					if (((int) this->pt) != 0)
						break;

				case 5: {
					// nur Orientierung:
					const char* orient_msg = "$o%4.2f,%4.2f,%4.2f#";
					//"roll=%4.2f, pitch=%4.2f, yaw=%4.2f\0";

					char orient_str[26]; //41

					sprintf(orient_str, orient_msg, angles.roll, angles.pitch,
							angles.yaw);

					write2UART(orient_str);
				}
					if (((int) this->pt) != 0)
						break;
				}

				// Zeilenumbruch, um Datensätze voneinander trennen zu können
				write2UART("\n");
			}
			// mit Interval suspenden:
			suspendCallerUntil(NOW()+ this->interval * MILLISECONDS);
		}
	}

}
telemetryT; // Telemetry-Thread

//*******************************************************************************

// Empfängt Telecommandos aus UART und führt sie aus
struct telecommandThread: public Thread, public SubscriberReceiver<Telecommand> {
	Command cmd; // zur Übertragung in o.g. Topic

	telecommandThread() :
			SubscriberReceiver<Telecommand>(TopicTelecommand,
					"TopicTelecommands") {
	}

	void init() {
		led_red.init(1, 1, 0);
	}

	void put(Telecommand& data) {
		Telecommand* _data = (Telecommand*) &data;

		// zugehörige LED toggeln:
		ToggleLED(led_red, 1000); // rot

		switch (_data->id) {
		case 'S': {
			// Interval von SignalProcess ändern: (via Topic)
			cmd.id = 'I';
			cmd.value = _data->data;

			TopicTelemetry.publish(cmd);
		}
			break;
		case 'T': {
			cmd.id = 'T';
			cmd.value = _data->data;

			TopicTelemetry.publish(cmd);
		}
			break;
		case 'Q': {
			// alle:
			cmd.id = 'Q';
			cmd.value = 0;

			TopicTelemetry.publish(cmd);
		}
			break;
		case 'A': {
			// nur Acc:
			cmd.id = 'A';
			cmd.value = 1;

			TopicTelemetry.publish(cmd);
		}
			break;
		case 'G': {
			// nur Gyro:
			cmd.id = 'G';
			cmd.value = 2;

			TopicTelemetry.publish(cmd);
		}
			break;
		case 'M': {
			// nur Mag:
			cmd.id = 'M';
			cmd.value = 3;

			TopicTelemetry.publish(cmd);
		}
			break;
		case 'H': {
			// nur Temperatur:
			cmd.id = 'H';
			cmd.value = 4;

			TopicTelemetry.publish(cmd);
		}
			break;
		case 'O': {
			// nur Orientierung:
			cmd.id = 'O';
			cmd.value = 5;

			TopicTelemetry.publish(cmd);
		}
			break;
		case 'C': {
			// neu Kalibrieren:
			cmd.id = 'C';
			cmd.value = (data.data == 1.0 ? true : false);

			TopicTelemetry.publish(cmd);
		}
			break;
		case 'n': {
			cmd.id = 'n';
			cmd.value = data.data;

			TopicTelemetry.publish(cmd);
		}
			break;
		case 'p': {
			cmd.id = 'p';
			cmd.value = data.data;

			TopicTelemetry.publish(cmd);
		}
			break;
		case 'q': {
			cmd.id = 'q';
			cmd.value = data.data;

			TopicTelemetry.publish(cmd);
		}
			break;
		case 'r': {
			cmd.id = 'r';
			cmd.value = data.data;

			TopicTelemetry.publish(cmd);
		}
			break;
		case 's': {
			cmd.id = 's';
			cmd.value = data.data;

			TopicTelemetry.publish(cmd);
		}
			break;
		case 't': {
			cmd.id = 't';
			cmd.value = data.data;

			TopicTelemetry.publish(cmd);
		}
			break;
		case 'u': {
			cmd.id = 'u';
			cmd.value = data.data;

			TopicTelemetry.publish(cmd);
		}
			break;
		case 'v': {
			cmd.id = 'v';
			cmd.value = data.data;

			TopicTelemetry.publish(cmd);
		}
			break;
		case 'w': {
			cmd.id = 'w';
			cmd.value = data.data;

			TopicTelemetry.publish(cmd);
		}
			break;
		case 'x': { // x-Wert Accelerometer (Offset)
			cmd.id = 'x';
			cmd.value = data.data;

			TopicTelemetry.publish(cmd);
		}
			break;
		case 'y': { // y-Wert Accelerometer (Offset)
			cmd.id = 'y';
			cmd.value = data.data;

			TopicTelemetry.publish(cmd);
		}
			break;
		case 'z': { // z-Wert Accelerometer (Offset)
			cmd.id = 'z';
			cmd.value = data.data;

			TopicTelemetry.publish(cmd);
		}
			break;
		case 'F': { // Änderung Filterkonstante
			cmd.id = 'F';
			cmd.value = data.data;

			TopicTelemetry.publish(cmd);
		}
			break;
		case 'Z': { // Standardwerte für Kalibrierung (Offsetfehler) verwenden
			cmd.id = 'Z';
			cmd.value = 1;

			TopicTelemetry.publish(cmd);
		}
		}

		// mit zugehöriger LED toggeln:
		ToggleLED(led_red, 500); // rot
	}

	void run(void) {
		while (1) {
			// mit zugehöriger LED toggeln:
			//ToggleLED(LED[2], 500);
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

		char _data[length];		// array mit Länge len

		//strncpy(_data, &cmd[3], length); // Nur Data kopieren
		for (int i = 0; i < length; i++)
			_data[i] = *ptr++;

		return atof(_data);		// in float konvertieren und zurückgeben
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

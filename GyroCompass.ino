#include "NMEA2000_CAN.h"
#include <MadgwickAHRS.h>
#include <i2c_t3.h>
#include "MPU9250.h"
#include <SparkFunBME280.h>
#include <EEPROM.h>
#include "kalman.h"

#define EEPROM_ADR_CONFIG 0 // eeprom address for config params
#define P0 1005.3
bool SerialDebug = false;
#define intPin 12
#define stbyPin 14
volatile boolean flagDataReady;
volatile boolean flagCalibrate;
#define MAGIC 24234 
struct tConfig {
	uint16_t Magic; //test if eeprom initialized
	float magBias[3];
	float magScale[3];
	float accelBias[3];
	float gyroBias[3];
	float KpAcc;	// proportional gain governs rate of convergence to accelerometer
	float KiAcc;	// integral gain governs rate of convergence of gyroscope biases
	float KpMag;	// proportional gain governs rate of convergence to magnetometer
	float KiMag;	// integral gain governs rate of convergence of gyroscope biases
	float accelCutoff;
};
int SID = 1;

tConfig config;
const tConfig defConfig PROGMEM = {
	MAGIC,
	0.f,0.f,0.f,	// magBias
	1.f,1.f,1.f,	// magScale
	0.f,0.f,0.f,	// accelBias
	0.f,0.f,0.f,	// gyroBias
	1.0f,0.f,		// KpAcc, KiAcc
	5.0f,0.f,		// KpMag ,KiMag
	0.25f			// accelCutoff
};

//const tNMEA2000::tProductInformation GyroCompassProductInformation PROGMEM = {
//	1300,                        // N2kVersion
//	101,                         // Manufacturer's product code
//	"GyroCompass",    // Manufacturer's Model ID
//	"1.1.0.17 (2017-06-21)",     // Manufacturer's Software version code
//	"1.1.0.0 (2017-06-21)",      // Manufacturer's Model version
//	"00000002",                  // Manufacturer's Model serial code
//	0,                           // CertificationLevel
//	4                            // LoadEquivalency
//};
tN2kMsg N2kMsg;
MPU9250 IMU(0x68, 0);
float a12, a22, a31, a32, a33;            // rotation matrix coefficients for Euler angles and gravity components
float ax, ay, az, gx, gy, gz, hx, hy, hz, t, mx, my, mz;
int beginStatus;
float SelfTest[6];            // holds results of gyro and accelerometer self test
float magBias[3] = { 0, 0, 0 }, magScale[3] = { 0, 0, 0 };
float factoryMagScale[3];
double T, P;
float pitch, yaw, roll, rollrate, pitchrate, yawrate, heading;

float deltat = 0.0f, sum = 0.0f;        // integration interval for both filter schemes
uint32_t delt_t = 0; // used to control display output rate
uint32_t count = 0, sumCount = 0; // used to control display output rate
uint32_t lastUpdate = 0, firstUpdate = 0; // used to calculate integration interval
uint32_t Now = 0;        // used to calculate integration interval
float lin_ax, lin_ay, lin_az;             // linear acceleration (acceleration with gravity component subtracted)
float DECLINATION = 10.823; // magnetic declination @ annapolis 11d 9'

Madgwick ahrs;
BME280  bmp;

int16_t mmax[3] = { -9999 ,-9999 ,-9999 };
int16_t mmin[3] = { 9999 ,9999 ,9999 };

float mmaxf[3] = { -9999 ,-9999 ,-9999 };
float mminf[3] = { 9999 ,9999 ,9999 };

#define PROCESS_NOISE 0.00000001   // q process noise covariance
#define SENSOR_NOISE 0.01 // r measurement noise covariance
#define INITIAL_Q 500.0  // p estimation error covariance
Kalman Filter(PROCESS_NOISE, SENSOR_NOISE, INITIAL_Q, 10);
int srd = 9; // 5;
float mxa, mya, mza;
long slowloop = 0;
long veryslowloop = 0;
// List here messages your device will transmit.
const unsigned long TransmitMessages[] PROGMEM = { 127257L, 127251L, 130311L,0 };

void setup()
{
	pinMode(LED_BUILTIN, OUTPUT);
	pinMode(stbyPin, OUTPUT);
	digitalWrite(stbyPin, LOW);

	Blink(5, 300);
	Serial.begin(115200);
	delay(100);
	Serial.println("Start");
	pinMode(intPin, INPUT_PULLUP);

	attachInterrupt(digitalPinToInterrupt(intPin), myinthandler, RISING);
	Serial.println("Attach Interrupt");
	delay(100);
	Serial.println("IMU initialization");
	beginStatus = IMU.begin(ACCEL_RANGE_2G, GYRO_RANGE_250DPS);
	if (beginStatus < 0) {
		delay(1000);
		Serial.println("IMU initialization unsuccessful");
		Serial.println("try cycling power");
		delay(10000);
		beginStatus = IMU.begin(ACCEL_RANGE_4G, GYRO_RANGE_250DPS);
	}
	else Serial.println("IMU init success!");
	IMU.setFilt(DLPF_BANDWIDTH_5HZ, srd);
	IMU.getMagScale(factoryMagScale);
	ReadConfig();
	if (config.Magic != MAGIC) {
		//InitializeEEPROM();
		Serial.println(F("\nNo stored calibration\r\n  Press 'a' to run accel/gyro cal\n  Press 'c' to run mag cal"));
		delay(1000);
	}
	else
	{
		Serial.println(F("Loading stored calibration\n"));
		PrintConfig();
		IMU.setAccelBias(config.accelBias);
		IMU.setGyroBias(config.gyroBias);
		IMU.setMagBias(config.magBias);
		IMU.setMagScale(config.magScale);
	}
	//selfTest();

	Serial.println("BMP Start");
	bmp.settings.commInterface = I2C_MODE;
	bmp.settings.I2CAddress = 0x76;
	bmp.settings.runMode = 3;
	bmp.settings.filter = 0;
	bmp.settings.tempOverSample = 1;
	bmp.settings.pressOverSample = 1;
	//bmp.settings.disableHumidity = true;
	if (!bmp.begin()) {
		Serial.println(F("BMP init failed!"));
		while (1);
	}
	else Serial.println(F("BMP init success!"));
	delay(50);
	while (!flagDataReady) {}
	ahrs.begin(34);
	NMEA2000.SetProductInformation("01290517", // Manufacturer's Model serial code
		666, // Manufacturer's product code
		"GyroCompass",  // Manufacturer's Model ID
		"1.0.0.1 (2015-08-14)",  // Manufacturer's Software version code
		"1.0.0.0 (2015-08-14)" // Manufacturer's Model version
	);
	//// Det device information
	NMEA2000.SetDeviceInformation(290517, // Unique number. Use e.g. Serial number.
		140, // Device function=Temperature See codes on http://www.nmea.org/Assets/20120726%20nmea%202000%20class%20%26%20function%20codes%20v%202.00.pdf
		60, // Device class=Sensor Communication Interface. See codes on http://www.nmea.org/Assets/20120726%20nmea%202000%20class%20%26%20function%20codes%20v%202.00.pdf
		2040 // Just choosen free from code list on http://www.nmea.org/Assets/20121020%20nmea%202000%20registration%20list.pdf                               
	);
	NMEA2000.SetMode(tNMEA2000::N2km_NodeOnly, 22);
	NMEA2000.EnableForward(false);
	NMEA2000.ExtendTransmitMessages(TransmitMessages);
	NMEA2000.SetN2kCANMsgBufSize(5);
	NMEA2000.Open();
	delay(100);
	Serial.println(F("Starting\n a=Accl/Gyro cal.\n c=Start Mag cal.\n d=toggle output mag x,y,x\n h=toggle output heading.\n p=print config.\n t=Self Test.\n\n"));
}
bool SDEBUG = false; // debug print for spreadsheet
bool SPRINT = false; // send test output

int pointcount = 0; // data points for mag test

void loop() {
	char command = getCommand();
	switch (command)
	{
	case 'c': // start mag calibration		
		if (!flagCalibrate)
		{
			StartMagCal();
		}
		else
		{
			Serial.println(F("Calibration already running!"));
		}		
		break;
	case 'f': // finish mag calibration
		FinishMagCal();
		break;
	case 'h': //toggle debug
		SDEBUG = !SDEBUG;
		delay(100);
		break;
	case 's':
		SPRINT = !SPRINT;
		break;
	case 'p':
		PrintConfig();
		PrintIMUConfig();
		break;
	case 'a':
		Serial.println(F("Accel Calibration Starting - hold still"));
		CalibrateAG();
		break;
	case 't':
		selfTest();
		break;
	case 'l':
		printMag();
		break;
	case 'r':
		RemoveCal();
		break;
	default:
		break;
	}
	
	if (flagDataReady) { //wait for interrupt
		if (flagCalibrate)
		{
			magCalLoop();
		}
		else
		{
			IMU.getMotion9(&ax, &ay, &az, &gx, &gy, &gz, &mx, &my, &mz);
			ahrs.update(gx, gy, gz, ax, ay, az, mx, my, mz);
		}
		digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
		flagDataReady = false;
	}
	
	// display at 0.25s rate independent of data rates
	delt_t = millis() - count;
	if (delt_t > 250) { // update once per half-second independent of read rate
		Now = micros();
		deltat = ((Now - lastUpdate) / 1000000.0f); // set integration time by time elapsed since last filter update
		lastUpdate = Now;
		count = millis();

		if (!flagCalibrate)
		{
			roll = ahrs.getRoll();
			pitch = ahrs.getPitch();
			yaw = ahrs.getYaw();
			heading = ahrs.getYaw();
			//heading += DECLINATION;
			//heading = YawtoHeading(yaw);
			if (SDEBUG) {
				if (pointcount < 5000) {
					printMag();
					pointcount++;
				}
				else {
					SDEBUG = false;
					pointcount = 0;
				}
			}
			if (SPRINT) { printHeading(); }
			SetN2kAttitude(N2kMsg, SID, yaw*DEG_TO_RAD, pitch*DEG_TO_RAD, roll*DEG_TO_RAD);
			NMEA2000.SendMsg(N2kMsg);
			SetN2kRateOfTurn(N2kMsg, SID, gz);  // radians
			NMEA2000.SendMsg(N2kMsg);
			SID++; if (SID > 254) { SID = 1; }
		}
		slowloop++;
	}
	//1 s
	if (slowloop > 4) { slowloop = 0; SlowLoop(); }
	NMEA2000.ParseMessages();
}

void RemoveCal()
{
	Serial.println(F("Remove calibration"));
	for (int ii = 0; ii < 3; ii++) {
		config.accelBias[ii] = 0;
		config.gyroBias[ii] = 0;
		config.magBias[ii] = 0;
		config.magScale[ii] = 1;
	}
	UpdateConfig();
	IMU.setAccelBias(config.accelBias);
	IMU.setGyroBias(config.gyroBias);
	IMU.setMagBias(config.magBias);
	IMU.setMagScale(config.magScale);
}

void SlowLoop()
{
	if (flagCalibrate) {
		Serial.print('.');
	}
	// send magnetic heading
	SetN2kMagneticHeading(N2kMsg, SID, heading*DEG_TO_RAD, N2kDoubleNA, N2kDoubleNA);
	NMEA2000.SendMsg(N2kMsg);
	// Send Temperature & Pressure
	T = bmp.readTempC();
	P = bmp.readFloatPressure();
	SetN2kEnvironmentalParameters(N2kMsg, SID, N2kts_MainCabinTemperature, CToKelvin(T), N2khs_Undef, 0, P);
	NMEA2000.SendMsg(N2kMsg);
}

void myinthandler()
{
	flagDataReady = true;
}

void CalibrateAG()
{
	float x, y, z;
	float aa[3], ga[3];
	IMU.setAccelBias(aa); //zero current bias calibration 
	IMU.setGyroBias(ga);
	uint8_t i = 0;
	const uint8_t samples = 100;
	while (i < samples)
	{
		if (flagDataReady) { 
			IMU.getAccel(&x, &y, &z);
			aa[0] += x; aa[1] += y; aa[2] += z;
			IMU.getGyro(&x, &y, &z);
			ga[0] += x; ga[1] += y; ga[2] += z;
			delay(20);
			digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
			flagDataReady = false;
			i++;
		}
	}
	for (uint8_t ii = 0; ii < 3; ii++) {  // Get the average
		aa[ii] /= (float)samples;
		ga[ii] /= (float)samples;
	}
	aa[2] -= 10; // subtract Gravity from acc Z
	IMU.setAccelBias(aa);
	IMU.setGyroBias(ga);
	for (uint8_t ii = 0; ii < 3; ii++) {
		config.accelBias[ii] = aa[ii];
		config.gyroBias[ii] = ga[ii];
	}
	Serial.println("\nAverage");
	Serial.print("Accel X "); Serial.print(aa[0] );
	Serial.print(" Y "); Serial.print(aa[1]);
	Serial.print(" Z "); Serial.print(aa[2]);
	Serial.print(" Gyro X "); Serial.print(ga[0]);
	Serial.print(" Y "); Serial.print(ga[1]);
	Serial.print(" Z "); Serial.println(ga[2]);
	delay(500);
	Serial.println("\nWriting EEPROM");
	UpdateConfig();
	delay(2000);
	flagCalibrate = false;
}

void selfTest()
{
	IMU.MPU9250SelfTest(IMU.SelfTest);
	Serial.print("x-axis self test: acceleration trim within : ");
	Serial.print(IMU.SelfTest[0], 1); Serial.println("% of factory value");
	Serial.print("y-axis self test: acceleration trim within : ");
	Serial.print(IMU.SelfTest[1], 1); Serial.println("% of factory value");
	Serial.print("z-axis self test: acceleration trim within : ");
	Serial.print(IMU.SelfTest[2], 1); Serial.println("% of factory value");
	Serial.print("x-axis self test: gyration trim within : ");
	Serial.print(IMU.SelfTest[3], 1); Serial.println("% of factory value");
	Serial.print("y-axis self test: gyration trim within : ");
	Serial.print(IMU.SelfTest[4], 1); Serial.println("% of factory value");
	Serial.print("z-axis self test: gyration trim within : ");
	Serial.print(IMU.SelfTest[5], 1); Serial.println("% of factory value");
}

void PrintConfig()
{
	Serial.print("Config Accel bias\t");
	Serial.print(config.accelBias[0]); Serial.print("\t");
	Serial.print(config.accelBias[1]); Serial.print("\t");
	Serial.print(config.accelBias[2]); Serial.println("\t");
	Serial.print("Config Gyro bias\t");
	Serial.print(config.gyroBias[0]); Serial.print("\t");
	Serial.print(config.gyroBias[1]); Serial.print("\t");
	Serial.print(config.gyroBias[2]); Serial.println("\t");
	Serial.print("Config Mag bias \t");
	Serial.print(config.magBias[0]); Serial.print("\t");
	Serial.print(config.magBias[1]); Serial.print("\t");
	Serial.print(config.magBias[2]); Serial.println("\t");
	Serial.print("Config Mag scale\t");
	Serial.print(config.magScale[0]); Serial.print("\t");
	Serial.print(config.magScale[1]); Serial.print("\t");
	Serial.print(config.magScale[2]); Serial.println("\t");
}

void PrintIMUConfig()
{
	float buff[3];
	IMU.getAccelBias(buff);
	Serial.println("");
	Serial.print("IMU Accel bias\t");
	Serial.print(buff[0],2); Serial.print("\t");
	Serial.print(buff[1],2); Serial.print("\t");
	Serial.print(buff[2],2); Serial.println("\t");
	IMU.getGyroBias(buff);
	Serial.print("IMU Gyro bias\t");
	Serial.print(buff[0],2); Serial.print("\t");
	Serial.print(buff[1],2); Serial.print("\t");
	Serial.print(buff[2],2); Serial.println("\t");
	IMU.getMagBias(buff);
	Serial.print("IMU Mag bias\t");
	Serial.print(buff[0],2); Serial.print("\t");
	Serial.print(buff[1],2); Serial.print("\t");
	Serial.print(buff[2],2); Serial.println("\t");
	IMU.getMagScale(buff);
	Serial.print("Mag scale\t");
	Serial.print(buff[0],4); Serial.print("\t");
	Serial.print(buff[1],4); Serial.print("\t");
	Serial.print(buff[2],4); Serial.println("\t");
}

float YawtoHeading(float yaw)
{
	float hdm;
	hdm = yaw - 90;               // converts yaw to heading magnetic
	if (yaw < 90 && yaw >= -179.99) {
		hdm = yaw + 270;	
	}
	return hdm;
}

void printData() {
	Serial.println("");
	if (SerialDebug) {
		Serial.print("gx\t"); Serial.print(gx*RAD_TO_DEG, 2); Serial.println(" deg/s");
		Serial.print("gy\t"); Serial.print(gy*RAD_TO_DEG, 2); Serial.println(" deg/s");
		Serial.print("ax\t"); Serial.println((ax), 2);
		Serial.print("ay\t"); Serial.println((ay), 2);
		Serial.print("az\t"); Serial.println((az), 2);
		Serial.print("mx\t"); Serial.print(mx); Serial.println(" milliGauss");
		Serial.print("my\t"); Serial.print(my); Serial.println(" milliGauss");
		Serial.print("mz\t"); Serial.print(mz); Serial.println(" milliGauss");	
		//Serial.print("q0 = "); Serial.print(*(getQ()));
		//Serial.print(" qx = "); Serial.print(*(getQ() + 1));
		//Serial.print(" qy = "); Serial.print(*(getQ() + 2));
		//Serial.print(" qz = "); Serial.println(*(getQ() + 3));
	}

	Serial.print(F("Pressure: "));
	Serial.print(P/100, 1);
	Serial.println(" P");

	Serial.print(F("Temp: "));
	Serial.print(T, 2);
	Serial.println(" *C");

	Serial.print("Pitch:\t"); Serial.println(pitch, 2);
	Serial.print("Roll:\t"); Serial.println(roll, 2);
	Serial.print("Yaw:\t");	Serial.println(yaw, 2);
	Serial.print("Heading\t"); Serial.print(heading); Serial.println(" deg");
	Serial.print("RateOfTurn\t"); Serial.print(gz*RAD_TO_DEG, 2); Serial.println(" deg/s");

	if (SerialDebug) {
		Serial.print("rate = "); Serial.print((float)sumCount / sum, 2); Serial.println(" Hz");
	}
}

char getCommand()
{
	char c = '\0';
	if (Serial.available())
	{
		c = Serial.read();
	}
	return c;
}

// tbd mag cal circle detection
//int magCalCircles = 3;
//void magCalLoop()
//{
//	// collect data points
//	float x, y, z;
//	IMU.getMag(&x, &y, &z);
//	magcalMinMax(x, y, z); //accumulate min max 
//    //detect circles
//	//if(completed) magCalCalc
//}

void StartMagCal() 
{
	flagCalibrate = true;
	IMU.startMagCal();
	Serial.println(F("Begin calibration, press 'f' to finish"));
}

void magCalLoop()
{
	IMU.updateMagCal();
}

void FinishMagCal()
{
	Serial.println("\nCal complete");
	IMU.stopMagCal();
	Serial.print("Computed new values from: ");
	Serial.print(IMU.getCalCount());
	Serial.println(" samples");
	// read new values
	IMU.getMagBias(magBias);
	IMU.getMagScale(magScale);
	//update config
	for (uint8_t i = 0; i < 3; i++) {
		config.magBias[i] = magBias[i];
		config.magScale[i] = magScale[i];
	}
	IMU.getMinMax(mmin, mmax);
	Serial.println("\nMag min max");
	Serial.print("\tX "); Serial.print(mmin[0]); Serial.print("\t"); Serial.println(mmax[0]);
	Serial.print("\tY "); Serial.print(mmin[1]); Serial.print("\t"); Serial.println(mmax[1]);
	Serial.print("\tZ "); Serial.print(mmin[2]); Serial.print("\t"); Serial.println(mmax[2]);
	Serial.println();
	PrintConfig();
	PrintIMUConfig();
	Serial.println("\nWriting EEPROM");
	UpdateConfig();
	flagCalibrate = false;
}



// LED blinker
// count flashes in duration ms
void Blink(int count, unsigned long duration)
{
	unsigned long d = duration / count;
	for (int counter = 0; counter < count; counter++) {
		digitalWrite(LED_BUILTIN, HIGH);
		delay(d / 2);
		digitalWrite(LED_BUILTIN, LOW);
		delay(d / 2);
	}
}

//Load From EEPROM 
void ReadConfig()
{
	EEPROM.get(EEPROM_ADR_CONFIG, config);
}

//Write to EEPROM - Teensy non-volatile area size is 2048 bytes  100,000 cycles
void UpdateConfig()
{
	Blink(5, 2000);
	Serial.print("Updating config");
	config.Magic = MAGIC;
	EEPROM.put(EEPROM_ADR_CONFIG, config);
}

void InitializeEEPROM()
{
	Serial.print("Initialize EEPROM");
	config = defConfig;
	UpdateConfig();
}

void printJSON()
{
	String s = String("{\"time\":") + millis()
		+ String(",\"pitch\":") + pitch
		+ String(",\"roll\":") + roll
		+ String(",\"yaw\":") + yaw
		+ String(",\"gz\":") + gz
		+ String(",\"t_celsius\":") + T
		+ String(",\"P\":") + P
		+ "}";
	Serial.println(s);
}

void dumpData() {
	Serial.print(pitch, 2); Serial.print("\t");
	Serial.print(roll, 2);Serial.print("\t");
	Serial.print(yaw, 2); Serial.print("\t");
	Serial.print(heading,2);
	Serial.println();
}

void printHeading() {
	Serial.print(heading, 2);
	Serial.println();
}

void printMag()
{
	//int16_t d[3];
	//IMU.getMagCounts(&d[0], &d[1], &d[2]);
	//Serial.print(""); Serial.print(d[0]);
	//Serial.print("\t"); Serial.print(d[1]);
	//Serial.print("\t"); Serial.println(d[2]);
	float d[3];
	IMU.getMag(&d[0], &d[1], &d[2]);
	Serial.print(""); Serial.print(d[0],3);
	Serial.print("\t"); Serial.print(d[1],3);
	Serial.print("\t"); Serial.println(d[2],3);
}
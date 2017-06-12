#include <i2c_t3.h>
#include <NMEA2000_CAN.h>
#include "N2kMessages.h"
#include "AHRS.h"
#include "quaternionFilters.h"
#include "MPU9250.h"
#include <SparkFunBME280.h>
#include <EEPROM.h>


#define EEPROM_ADR_CONFIG 0 // eeprom address for config params
#define P0 1005.3
bool SerialDebug = false;
uint8_t magDataUpdate = false;
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

MPU9250 IMU(0x68, 0);
float a12, a22, a31, a32, a33;            // rotation matrix coefficients for Euler angles and gravity components
float ax, ay, az, gx, gy, gz, hx, hy, hz, t, mx, my, mz;
int beginStatus;
float SelfTest[6];            // holds results of gyro and accelerometer self test
float magBias[3] = { 0, 0, 0 }, magScale[3] = { 0, 0, 0 };
double T, P;
float pitch, yaw, roll, rollrate, pitchrate, yawrate;

float deltat = 0.0f, sum = 0.0f;        // integration interval for both filter schemes
uint32_t delt_t = 0; // used to control display output rate
uint32_t count = 0, sumCount = 0; // used to control display output rate
uint32_t lastUpdate = 0, firstUpdate = 0; // used to calculate integration interval
uint32_t Now = 0;        // used to calculate integration interval
float lin_ax, lin_ay, lin_az;             // linear acceleration (acceleration with gravity component subtracted)
float DECLINATION = -10.823; // magnetic declination @ annapolis 11d 9'
AHRS ahrs;
BME280  bmp;

float mmax[3] = { -9999 ,-9999 ,-9999 };
float mmin[3] = { 9999 ,9999 ,9999 };

//float ax_scale = 1.0, ay_scale = 1.0, az_scale = 1.0;
//float mx_scale = 1, my_scale = 1, mz_scale = 1;
//float mx_bias = 0, my_bias = 0, mz_bias = 0;
int srd = 9; // 5;
float mxa, mya, mza;
long slowloop = 0;
long veryslowloop = 0;
// List here messages your device will transmit.
const unsigned long TransmitMessages[] PROGMEM = { 127257L, 127251L, 130311L,0 };
// 127258
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
	PrintIMUConfig();
	ReadConfig();
	if (config.Magic != MAGIC) {
		//InitializeEEPROM();
		Serial.println(F("No stored calibration\r\nPress 'a' to run accel/gyro cal"));
		delay(1000);
	}
	else
	{
		Serial.println(F("Setting stored calibration"));
		IMU.setAccelBias(config.accelBias);
		IMU.setGyroBias(config.gyroBias);
		IMU.setMagBias(config.magBias);
	}


	IMU.setFilt(DLPF_BANDWIDTH_5HZ, srd);
	//selfTest();
	PrintConfig();

	float bias[3];
	IMU.getAccelBias(bias);
	Serial.print("getAccelBias:");
	Serial.println(bias[0]);
	delay(100);
	Serial.println("BMP Start");
	bmp.settings.commInterface = I2C_MODE;
	bmp.settings.I2CAddress = 0x76;
	bmp.settings.runMode = 3;
	bmp.settings.filter = 0;
	bmp.settings.tempOverSample = 1;
	bmp.settings.pressOverSample = 1;
	//bmp.settings.disableHumidity = true;
	if (!bmp.begin()) {
		Serial.println("BMP init failed!");
		while (1);
	}
	else Serial.println(F("BMP init success!"));
	delay(50);
	while (!flagDataReady) {}
	IMU.getMotion9(&ax, &ay, &az, &gx, &gy, &gz, &mx, &my, &mz);
	//ahrs.SetCoeff(config.KpAcc, config.KiAcc, config.KpMag, config.KiMag, config.accelCutoff);
	//ahrs.MargAHRSinit(ax, ay, az, mx, my, mz);

	NMEA2000.SetProductInformation("01290517", // Manufacturer's Model serial code
		666, // Manufacturer's product code
		"GyroCompass",  // Manufacturer's Model ID
		"1.0.0.1 (2015-08-14)",  // Manufacturer's Software version code
		"1.0.0.0 (2015-08-14)" // Manufacturer's Model version
	);
	//// Det device information
	//NMEA2000.SetHeartbeatInterval(config.HeartbeatInterval);
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
}

void loop() {
	
	char command = getCommand();
	switch (command)
	{
	case 'p':
		PrintConfig();
		PrintIMUConfig();
		delay(5000);
		break;
	case 's':
		float bias[3];
		IMU.getAccelBias(bias);
		Serial.print("Accel X "); Serial.print(bias[0]);
		Serial.print(" Y "); Serial.print(bias[1]);
		Serial.print(" Z "); Serial.print(bias[2]);
		//	Serial.print(" Gyro X "); Serial.print(IMU.gyroXOffset);
		//	Serial.print(" Y "); Serial.print(IMU.gyroYOffset);
		//	Serial.print(" Z "); Serial.println(IMU.gyroZOffset);
		delay(2000);
		break;
	case 'c':
		//calibrate();
		SerialDebug = false;
		flagCalibrate = false;
		break;
	case 'd':
		SerialDebug = true;
		Serial.print("srd: ");
		Serial.println(srd);
		break;
	case 'a':
		Serial.println(F("Accel Calibration Starting - hold still"));
		CalibrateAG();
		delay(3000);
		break;
	case 't':
		selfTest();
		delay(5000);
		break;
	case 'm':
		if (!flagCalibrate)
		{
			CalibrateMag();
		}
		else
		{
			Serial.println(F("Calibration already running!"));
		}
		break;
	case 'r':
		for (int ii = 0; ii < 3; ii++) {
			config.accelBias[ii] = 1;
			config.gyroBias[ii] = 1;
		}
		UpdateConfig();
		IMU.setAccelBias(config.accelBias);
		IMU.setGyroBias(config.gyroBias);
		IMU.setMagBias(config.magBias);
		break;
	case 'h':
		// SendHeartbeat();
		break;
	default:
		break;
	}


	if (flagDataReady) { //wait for interrupt
						 // get the accel (m/s/s), gyro (rad/s), and magnetometer (uT)
		IMU.getMotion9(&ax, &ay, &az, &gx, &gy, &gz, &mx, &my, &mz);
		magDataUpdate = true;
		T = bmp.readTempC();
		P = bmp.readFloatPressure();
		//ahrs.MargAHRSupdate(gx, gy, -gz, -ax, -ay, az, mx, my, mz, config.accelCutoff, magDataUpdate, deltat);
		flagDataReady = false;
	}
	
	Now = micros();
	deltat = ((Now - lastUpdate) / 1000000.0f); // set integration time by time elapsed since last filter update
	lastUpdate = Now;

	sum += deltat; // sum for averaging filter update rate
	sumCount++;

	// Sensors x (y)-axis of the accelerometer is aligned with the y (x)-axis of the magnetometer;
	// the magnetometer z-axis (+ down) is opposite to z-axis (+ up) of accelerometer and gyro!
	// We have to make some allowance for this orientationmismatch in feeding the output to the quaternion filter.
	// For the MPU-9250, we have chosen a magnetic rotation that keeps the sensor forward along the x-axis just like
	// in the LSM9DS0 sensor. This rotation can be modified to allow any convenient orientation convention.
	MadgwickQuaternionUpdate(ax, ay, az, gx*PI / 180.0f, gy*PI / 180.0f, gz*PI / 180.0f, my, mx, mz, deltat);

	// Serial print and/or display at 0.5 s rate independent of data rates
	delt_t = millis() - count;
	if (delt_t > 250) { // update once per half-second independent of read rate
		a12 = 2.0f * (*(getQ() + 1) **(getQ() + 2) + *getQ() * *(getQ() + 3));
		a22 = *getQ() * *getQ() + *(getQ() + 1) * *(getQ() + 1) - *(getQ() + 2)  * *(getQ() + 2) - *(getQ() + 3) *  *(getQ() + 3);
		a31 = 2.0f * (*getQ() **(getQ() + 1) + *(getQ() + 2)  * *(getQ() + 3));
		a32 = 2.0f * (*(getQ() + 1) *  *(getQ() + 3) - *getQ() * *(getQ() + 2));
		a33 = *getQ() * *getQ() - *(getQ() + 1) * *(getQ() + 1) - *(getQ() + 2)  * *(getQ() + 2) + *(getQ() + 3) *  *(getQ() + 3);
		pitch = -asinf(a32);
		roll = atan2f(a31, a33);
		yaw = atan2f(a12, a22);
		pitch *= 180.0f / PI;
		yaw *= 180.0f / PI;
		yaw += DECLINATION; // Declination 
							//yaw = yaw + 180;
		if (yaw < 0) yaw += 360.0f; // Ensure yaw stays between 0 and 360
		roll *= 180.0f / PI;
		lin_ax = ax + a31;
		lin_ay = ay + a32;
		lin_az = az - a33;

		// print the data
		printData();
		//PrintAHRS();
		// delay a frame
		//delay(50);
		Now = micros();
		deltat = ((Now - lastUpdate) / 1000000.0f); // set integration time by time elapsed since last filter update
		lastUpdate = Now;

		sum += deltat; // sum for averaging filter update rate
					   //sumCount++;

		
		count = millis();
		sumCount = 0;
		sum = 0;
		tN2kMsg N2kMsg;
		
		SetN2kAttitude(N2kMsg, SID, yaw*DEG_TO_RAD, pitch*DEG_TO_RAD, roll*DEG_TO_RAD);
		NMEA2000.SendMsg(N2kMsg);
		SetN2kRateOfTurn(N2kMsg, SID, gz);  // radians
		NMEA2000.SendMsg(N2kMsg);
		slowloop++;
		SID++; if (SID > 254) { SID = 1; }
	}
	//2.5 s
	if (slowloop > 9) { slowloop = 0; SlowLoop(); }
	NMEA2000.ParseMessages();
}

void SlowLoop()
{
	digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
	//Serial.println("SendN2kTemperature");
	SendN2kTemperature();
	veryslowloop++;
	if (veryslowloop > 9) { veryslowloop = 0; NMEA2000.SendIsoAddressClaim(); }
}

void SendN2kTemperature() {
	tN2kMsg N2kMsg;
		//SetN2kTemperature(N2kMsg, 1, 1, N2kts_MainCabinTemperature, T);
		//NMEA2000.SendMsg(N2kMsg);
	SetN2kEnvironmentalParameters(N2kMsg, SID, N2kts_MainCabinTemperature, CToKelvin(T), N2khs_Undef,0, P);
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
	UpdateConfig();
	delay(5);
	Serial.println("\nAverage");
	Serial.print("Accel X "); Serial.print(aa[0] );
	Serial.print(" Y "); Serial.print(aa[1]);
	Serial.print(" Z "); Serial.print(aa[2]);
	Serial.print(" Gyro X "); Serial.print(ga[0]);
	Serial.print(" Y "); Serial.print(ga[1]);
	Serial.print(" Z "); Serial.println(ga[2]);
	delay(5000);

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
	ReadConfig();
	Serial.println("Config Accel bias");
	Serial.print(config.accelBias[0]); Serial.print("\t");
	Serial.print(config.accelBias[1]); Serial.print("\t");
	Serial.print(config.accelBias[2]); Serial.println("\t");
	Serial.println("Config Gyro bias");
	Serial.print(config.gyroBias[0]); Serial.print("\t");
	Serial.print(config.gyroBias[1]); Serial.print("\t");
	Serial.print(config.gyroBias[2]); Serial.println("\t");
	Serial.println("Config Mag bias");
	Serial.print(config.magBias[0]); Serial.print("\t");
	Serial.print(config.magBias[1]); Serial.print("\t");
	Serial.print(config.magBias[2]); Serial.println("\t");
	Serial.println("Config Mag scale");
	Serial.print(config.magScale[0]); Serial.print("\t");
	Serial.print(config.magScale[1]); Serial.print("\t");
	Serial.print(config.magScale[2]); Serial.println("\t");
}

void PrintIMUConfig()
{
	float buff[3];
	IMU.getAccelBias(buff);
	Serial.println("");
	Serial.println("IMU Accel bias");
	Serial.print(buff[0],2); Serial.print("\t");
	Serial.print(buff[1],2); Serial.print("\t");
	Serial.print(buff[2],2); Serial.println("\t");
	IMU.getGyroBias(buff);
	Serial.println("IMU Gyro bias");
	Serial.print(buff[0],2); Serial.print("\t");
	Serial.print(buff[1],2); Serial.print("\t");
	Serial.print(buff[2],2); Serial.println("\t");
	IMU.getMagBias(buff);
	Serial.println("IMU Mag bias");
	Serial.print(buff[0],2); Serial.print("\t");
	Serial.print(buff[1],2); Serial.print("\t");
	Serial.print(buff[2],2); Serial.println("\t");
	IMU.getMagScale(buff);
	Serial.println("Mag scale");
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

float BearingDegrees(float x, float y)
{
	float bearingRadians = atan2f(y, x); // get bearing in radians
	float bearingDegrees = bearingRadians * (180.0 / M_PI); // convert to degrees
	bearingDegrees = (bearingDegrees > 0.0 ? bearingDegrees : (360.0 + bearingDegrees)); // correct discontinuity
	return bearingDegrees;
}
float Compass_Heading(float mx, float my, float mz)
{
	float MAG_X;
	float MAG_Y;
	float cos_roll;
	float sin_roll;
	float cos_pitch;
	float sin_pitch;

	cos_roll = cos(roll);
	sin_roll = sin(roll);
	cos_pitch = cos(pitch);
	sin_pitch = sin(pitch);

	// Tilt compensated Magnetic field X:
	MAG_X = mx*cos_pitch + my*sin_roll*sin_pitch + mz*cos_roll*sin_pitch;
	// Tilt compensated Magnetic field Y:
	MAG_Y = my*cos_roll - mz*sin_roll;
	// Magnetic Heading
	return atan2(-MAG_Y, MAG_X) * 180 / PI + 180;
}

void PrintAHRS()
{
	Serial.print("ax\t"); Serial.println((ahrs.sensors.accel500Hz[0]), 2);
	Serial.print("ay\t"); Serial.println((ahrs.sensors.accel500Hz[1]), 2);
	Serial.print("az\t"); Serial.println((ahrs.sensors.accel500Hz[2]), 2);

	Serial.print("gx\t"); Serial.println((ahrs.sensors.gyro500Hz[0]), 2);
	Serial.print("gy\t"); Serial.println((ahrs.sensors.gyro500Hz[1]), 2);
	Serial.print("gz\t"); Serial.println((ahrs.sensors.gyro500Hz[2]), 2);

	Serial.print("y\t"); Serial.println((ahrs.sensors.attitude500Hz[0]), 2);
	Serial.print("p\t"); Serial.println((ahrs.sensors.attitude500Hz[1]), 2);
	Serial.print("r\t"); Serial.println((ahrs.sensors.attitude500Hz[2]), 2);

	Serial.print("head\t"); Serial.println((ahrs.heading.mag), 2);

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
		Serial.print("q0 = "); Serial.print(*(getQ()));
		Serial.print(" qx = "); Serial.print(*(getQ() + 1));
		Serial.print(" qy = "); Serial.print(*(getQ() + 2));
		Serial.print(" qz = "); Serial.println(*(getQ() + 3));
	}

	Serial.print(F("Pressure: "));
	Serial.print(P/100, 1);
	Serial.println(" P");

	Serial.print(F("Temp: "));
	Serial.print(T, 2);
	Serial.println(" *C");

	Serial.print("Yaw:\t");	Serial.println(yaw, 2);
	Serial.print("Pitch:\t"); Serial.println(pitch, 2);
	Serial.print("Roll:\t"); Serial.println(roll, 2);
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


void magcalMinMax(float mx, float my, float mz)
{
	if (mx > mmax[0]) { mmax[0] = mx; }
	if (my > mmax[1]) { mmax[1] = my; }
	if (mz > mmax[2]) { mmax[2] = mz; }
	if (mx < mmin[0]) { mmin[0] = mx; }
	if (my < mmin[1]) { mmin[1] = my; }
	if (mz < mmin[2]) { mmin[2] = mz; }
}


// tbd mag cal circle detection
int magCalCircles = 3;

void StartMagCal() 
{
	flagCalibrate = true;
	magCalCircles = 0;
	float d[3];	IMU.setMagBias(d); //remove existing cal
}

void magCalLoop()
{
	// collect data points
	float x, y, z;
	IMU.getMag(&x, &y, &z);
	magcalMinMax(x, y, z); //accumulate min max 

	//detect circles
	
	//
	//if(completed) magCalCalc
	//IMU.setMagBias(magBias);
}

void magCalCalc()
{
	// Get hard iron correction
	magBias[0] = (mmax[0] + mmin[0]) / 2;
	magBias[1] = (mmax[1] + mmin[1]) / 2;
	magBias[2] = (mmax[2] + mmin[2]) / 2;

	// Get soft iron correction estimate
	magScale[0] = (mmax[0] - mmin[0]) / 2;
	magScale[1] = (mmax[1] - mmin[1]) / 2;
	magScale[2] = (mmax[2] - mmin[2]) / 2;

	float avg_rad = magScale[0] + magScale[1] + magScale[2];
	avg_rad /= 3.0;
	// center of points
	mxa = avg_rad / magScale[0];
	mya = avg_rad / magScale[1];
	mza = avg_rad / magScale[2];
}

void CalibrateMag()
{
	//reset bias to zero
	float d[3];
	IMU.setMagBias(d);
	magScale[0]=1; magScale[1] = 1; magScale[2] = 1;

	flagCalibrate = true;
	float x, y, z;	int i = 0;
	int samples = 100;
	while (i < samples)
	{
		if (flagDataReady) {
			IMU.getMag(&x, &y, &z);
			magcalMinMax(x, y, z);
			delay(150);
			digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
			flagDataReady = false;
			i++;
		}
	}
	magCalCalc();

	IMU.setMagBias(magBias);
	//update config
	for (uint8_t ii = 0; ii < 3; ii++) {
		config.magBias[ii] = magBias[ii];
		config.magScale[ii] = magScale[ii];
	}

	UpdateConfig(); //save to eeprom
		Serial.println("Mag");
		Serial.print("X "); Serial.print(mmin[0]); Serial.print(" - "); Serial.println(mmax[0]);
		Serial.print("Y "); Serial.print(mmin[1]); Serial.print(" - "); Serial.println(mmax[1]);
		Serial.print("Z "); Serial.print(mmin[2]); Serial.print(" - "); Serial.println(mmax[2]);
		Serial.println("Bias");
		Serial.print("X "); Serial.println(magBias[0]);
		Serial.print("Y "); Serial.println(magBias[1]);
		Serial.print("Z "); Serial.println(magBias[2]);

		Serial.println("Scale");
		Serial.print("X "); Serial.println(magScale[0]);
		Serial.print("Y "); Serial.println(magScale[1]);
		Serial.print("Z "); Serial.println(magScale[2]);

		Serial.println("Center");
		Serial.print("X "); Serial.println(mxa);
		Serial.print("Y "); Serial.println(mya);
		Serial.print("Z "); Serial.println(mza);
	
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

float rad2deg(float rad)
{
	float deg = 0;
	deg = rad * (180 / M_PI);
	return deg;
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
	delay(5000);
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
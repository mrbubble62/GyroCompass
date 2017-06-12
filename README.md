# GyroCompass
## NMEA2000 style CAN Gyro Sensor

Built with vMicro on VS2015 for Teensy 3.2

### Parts list:

Teensy 3.1/3.2 board

GY-91 Attitude Sensor MPU9250+BMP280 10DOF Compass Acceleration Gyro Module

DC-DC 9V/12V/24V to 5V

MCP2562 CAN Transceiver

### Working:
PGN 127257  Attitude 
	Yaw, Pitch, Roll

PGN 127251	RateOfTurn
	radians/s

PGN 130311 Environmental Parameters 
	Barometric Pressure
	Temperature


### Serial Data

Pressure: 1014.5 P
Temp: 34.42 *C
Yaw:	344.96
Pitch:	-2.89
Roll:	-14.13
RateOfTurn	0.12 deg/s

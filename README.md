# GyroCompass
## NMEA2000 style CAN Gyro Sensor

Built with vMicro on VS2015 for Teensy 3.2

### Parts list:

Teensy 3.1/3.2 board

GY-91 Attitude Sensor MPU9250+BMP280 10DOF Magnatometer Acceleration Gyro Barometer Module

DC-DC 9V/12V/24V to 5V

MCP2562 CAN Transceiver

### Working:
PGN 127257  Attitude 
	Yaw, Pitch, Roll

PGN 127250 Magnetic Heading

PGN 127251	RateOfTurn
	radians/s

PGN 130311 Environmental Parameters 
	Barometric Pressure
	Temperature

## Configuration
Connect USB Serial

### Serial Send
"a" = Calibrate accel and gyro and save to eeprom (keep level and still)

"m" = Calibrate magnatometer and save to eeprom
	turn magnatometer through all positions for ~60 seconds

"d" = dump 5000 magnatometer x,y,z points to check calibration (plot xy,xz,zy on scatter graph)

"p" = print stored and current calibration

"s" = send heading continuously for ArduinoCompass test

"t" = Selftest


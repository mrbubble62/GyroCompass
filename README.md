# GyroCompass
## NMEA2000 style CAN Gyro Sensor

### Parts list:

Teensy 3.2 board

GY-91 Attitude Sensor MPU9250+BMP280 10DOF Magnatometer Acceleration Gyro Barometer Module

DC-DC 9V/12V/24V to 5V

MCP2562 CAN Transceiver

M12H5J-12 IP67 Waterproof Connector

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

"c" = Start magnatometer calibration, turn through all positions

"f" = Finish magnatometer calibration and save to eeprom

"d" = Dump 5000 magnatometer x,y,z points to check calibration (plot xy,xz,zy on scatter graph)

"p" = Print stored and current calibration

"s" = Send heading continuously for ArduinoCompass test

"t" = Selftest

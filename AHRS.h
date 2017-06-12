// AHRS.h

#ifndef _AHRS_h
#define _AHRS_h

#if defined(ARDUINO) && ARDUINO >= 100
	#include "arduino.h"
#else
	#include "WProgram.h"
#endif

//=====================================================================================================
// AHRS.h
// S.O.H. Madgwick
// 25th August 2010
//
// 1 June 2012 Modified by J. Ihlein
// 27 Aug  2012 Extensively modified to include G.K. Egan's accel confidence calculations and
//                                                          calculation efficiency updates
//=====================================================================================================
//
// See AHRS.c file for description.
//
//=====================================================================================================

//----------------------------------------------------------------------------------------------------
// Variable declaration
#define ROLL     0
#define PITCH    1
#define YAW      2

typedef struct sensors_t
{
	float    accel500Hz[3];
	float    attitude500Hz[3];
	float    gyro500Hz[3];
} sensors_t;

typedef struct heading_t
{
	float    mag;
	float    tru;
} heading_t;

 

float Constrain(float input, float minValue, float maxValue);

class AHRS {
public:
	AHRS();
	void SetCoeff(float KpAcc, float KiAcc, float KpMag, float KiMag, float accelCutoff);
	float accConfidenceDecay;
	heading_t heading;
	sensors_t sensors;
	float KpAcc;
	float KiAcc;
	float KpMag; 
	float KiMag;
	float accelCutoff;
	void MargAHRSupdate(float gx, float gy, float gz,
		float ax, float ay, float az,
		float mx, float my, float mz,
		float accelCutoff, uint8_t magDataUpdate, float dt);

	void MargAHRSinit(float ax, float ay, float az, float mx, float my, float mz);

private:
	float _KpAcc, _KiAcc, _KpMag, _KiMag, _accelCutoff;
	float q0q0, q0q1, q0q2, q0q3; // auxiliary variables to reduce number of repeated operations
	float q1q1, q1q2, q1q3;
	float q2q2, q2q3;
	float q3q3;
	float q0, q1, q2, q3;  // quaternion elements representing the estimated orientation								 
};
#endif


#include "MPU9250_Impact.h"
#include <MPU9250.h>
#include <cmath>
#include <queue>

#define VEC_SIZE (std::size_t)3
#define EXTREME_ACCEL (float)EXTREME_G*9.81
#define HIGH_G 4
#define EXTREME_G 7


float ImpactDetector::calcAbsAccel(AccelVector vec) {
	return sqrt(
		(vec[0] * vec[0]) + (vec[1] * vec[1]) + (vec[2] * vec[2])
	);
}

float ImpactDetector::calcAbsGyro(GyroVector vec) {
	return sqrt(
		(vec[0] * vec[0]) + (vec[1] * vec[1]) + (vec[2] * vec[2])
	);
}

float ImpactDetector::calcAbsGravityForce(float absAccel) {
	return abs(absAccel / this->G);
}


ImpactDetector::ImpactDetector() : MPU9250(Wire, 0x68)
{
	this->_tick = 0;
	this->_impactAccel = 0.0f;
	this->_potImpact = false;
	this->_pollRate = 100;
	this->_duration = 1000;

	this->_lowGThreshold = 0.1f;
	this->_highGThreshold = 2.5f;
	this->_extremeGThreshold = 4.0f;

	this->_lastAccelVector = { 0.0f, 0.0f, 0.0f };
	this->_curAccelVector = { 0.0f, 0.0f, 0.0f };
	this->_lastGyroVector = { 0.0f, 0.0f, 0.0f };
	this->_curGyroVector = { 0.0f, 0.0f, 0.0f };
	this->_lastImpact = { 0.0f, 0.0f, 0.0f };

	this->_lastAbsAccel = 0.0f;
	this->_curAbsAccel = 0.0f;
	this->_lastAbsGyro = 0.0f;
	this->_curAbsGyro = 0.0f;
	this->_lastAbsGravityForce = 0.0f;
	this->_curAbsGravityForce = 0.0f;

}

ImpactDetector::ImpactDetector(int pollRate, int duration, float lowGThreshold, float highGThreshold, float extremeGThreshold)
	: MPU9250(Wire, 0x68), _highGThreshold(highGThreshold), _extremeGThreshold(extremeGThreshold)
{
	if (pollRate > 1000) {
		this->_pollRate = 1000;
	}
	else if (pollRate > duration / 2) {
		this->_pollRate = duration / 2;
	}
	else {
		this->_pollRate = pollRate;
	}

	if (duration > 10000) {
		this->_duration = 10000;
	}
	else if (duration < 2 * pollRate) {
		this->_duration = 2 * pollRate;
	}
	else {
		this->_duration = duration;
	}

	if (lowGThreshold > 1) {
		this->_lowGThreshold = 1;
	}
	else {
		this->_lowGThreshold = lowGThreshold;
	}

	this->_tick = 0;
	this->_impactAccel = 0.0f;
	this->_potImpact = false;

	this->_lastAccelVector = { 0.0f, 0.0f, 0.0f };
	this->_curAccelVector = { 0.0f, 0.0f, 0.0f };
	this->_lastGyroVector = { 0.0f, 0.0f, 0.0f };
	this->_curGyroVector = { 0.0f, 0.0f, 0.0f };
	this->_lastImpact = { 0.0f, 0.0f, 0.0f };

	this->_lastAbsAccel = 0.0f;
	this->_curAbsAccel = 0.0f;
	this->_lastAbsGyro = 0.0f;
	this->_curAbsGyro = 0.0f;
	this->_lastAbsGravityForce = 0.0f;
	this->_curAbsGravityForce = 0.0f;
}

ImpactDetector::ImpactDetector(int pollRate, int duration, float lowGThreshold, float highGThreshold, float extremeGThreshold, uint8_t address, uint8_t sda, uint8_t scl)
	: MPU9250(Wire, address, sda, scl), _highGThreshold(highGThreshold), _extremeGThreshold(extremeGThreshold)
{
	if (pollRate > 1000) {
		this->_pollRate = 1000;
	}
	else if (pollRate > duration / 2) {
		this->_pollRate = duration / 2;
	}
	else {
		this->_pollRate = pollRate;
	}

	if (duration > 10000) {
		this->_duration = 10000;
	}
	else if (duration < 2 * pollRate) {
		this->_duration = 2 * pollRate;
	}
	else {
		this->_duration = duration;
	}

	if (lowGThreshold > 1) {
		this->_lowGThreshold = 1;
	}
	else {
		this->_lowGThreshold = lowGThreshold;
	}

	this->_tick = 0;
	this->_impactAccel = 0.0f;
	this->_potImpact = false;

	this->_lastAccelVector = { 0.0f, 0.0f, 0.0f };
	this->_curAccelVector = { 0.0f, 0.0f, 0.0f };
	this->_lastGyroVector = { 0.0f, 0.0f, 0.0f };
	this->_curGyroVector = { 0.0f, 0.0f, 0.0f };
	this->_lastImpact = { 0.0f, 0.0f, 0.0f };

	this->_lastAbsAccel = 0.0f;
	this->_curAbsAccel = 0.0f;
	this->_lastAbsGyro = 0.0f;
	this->_curAbsGyro = 0.0f;
	this->_lastAbsGravityForce = 0.0f;
	this->_curAbsGravityForce = 0.0f;
}

boolean ImpactDetector::detector()
{
	long cur_tick = millis();
	this->readSensor();

	AccelVector accelVector = { 0.0f, 0.0f, 0.0f };
	accelVector[0] = this->getAccelX_mss();
	accelVector[1] = this->getAccelY_mss();
	accelVector[2] = this->getAccelZ_mss();
	float absAccel = this->calcAbsAccel(accelVector);
	float absG = this->calcAbsGravityForce(absAccel);

	GyroVector gyroVector = { 0.0f, 0.0f, 0.0f };
	gyroVector[0] = this->getGyroX_rads();
	gyroVector[1] = this->getGyroY_rads();
	gyroVector[2] = this->getGyroZ_rads();
	float absGyro = this->calcAbsGyro(gyroVector);


	// potential Impact detected
	if ((this->_potImpact == true) && (cur_tick - this->_tick > this->_duration)) {
		if (absG < this->_lowGThreshold) {
			this->_potImpact = false;
			this->_lastImpact[0] = this->_curAbsGravityForce;
			this->_lastImpact[1] = this->_curAbsAccel;
			this->_lastImpact[2] = this->_curAbsGyro;
			return true;
		}
		else if (absG < this->_highGThreshold) {
			this->_potImpact = false;
		}
	}

	else if ((cur_tick - this->_tick) > this->_pollRate) {
		this->_tick = cur_tick;

		this->_lastAccelVector = this->_curAccelVector;
		this->_curAccelVector = accelVector;
		this->_lastAbsAccel = this->_curAbsAccel;
		this->_curAbsAccel = absAccel;
		this->_lastAbsGravityForce = this->_curAbsGravityForce;
		this->_curAbsGravityForce = absG;
		this->_lastGyroVector = this->_curGyroVector;
		this->_curGyroVector = gyroVector;
		this->_lastAbsGyro = this->_curAbsGyro;
		this->_curAbsGyro = absGyro;

		if (absG >= this->_extremeGThreshold) {
			this->_potImpact = true;

			this->_lastImpact[0] = absG;
			this->_lastImpact[1] = absAccel;
			this->_lastImpact[2] = absGyro;
			return true;
		}
		else if (absG > this->_highGThreshold) {
			this->_potImpact = true;
		}
	}
	return false;
}

void ImpactDetector::setHighGThreshold(float a) {
	if ((this->_lowGThreshold < a) && (this->_extremeGThreshold > a)) {
		this->_highGThreshold = a;
	}
}

float ImpactDetector::getHightGThreshold(void) {
	return this->_highGThreshold;
}

void ImpactDetector::setExtremGThreshold(float a) {
	if (a > this->_highGThreshold) {
		this->_extremeGThreshold = a;
	}
}

float ImpactDetector::getExtremGThreshold(void) {
	return this->_extremeGThreshold;
}

void ImpactDetector::setLowGThreshold(float a) {
	if (a < this->_highGThreshold) {
		this->_lowGThreshold = a;
	}
}

std::array<float, 3> ImpactDetector::getCurrentValues() {

	return  std::array<float, 3>({ this->_curAbsGravityForce, this->_curAbsAccel, this->_curAbsGyro });
}


std::array<float, 3> ImpactDetector::getLastValues() {

	return std::array<float, 3>({ this->_lastAbsGravityForce, this->_lastAbsAccel, this->_lastAbsGyro });
}


std::array<float, 3> ImpactDetector::getLastImpact() {

	return  this->_lastImpact;
}
#ifndef MPU9250_IMPACT_H
#define MPU9250_IMPACT_H
#include <MPU9250.h>
#include <queue>
#include <array>

#define IMPACT 1;
#define NO_IMPACT 0;

#define AccelVector std::array<float,3>
#define GyroVector std::array<float,3>
#define LastImpact std::array<float,3>


class ImpactDetector : public MPU9250
{
	// public methods
public:
	ImpactDetector();
	ImpactDetector(int pollRate, int duration, float lowGThreshold, float highGThreshold, float extremeGThreshold);
	ImpactDetector(int pollRate, int duration, float lowGThreshold, float highGThreshold, float extremeGThreshold, uint8_t address, uint8_t sda, uint8_t scl);

	bool detector();
	std::array<float, 3> getCurrentValues();
	std::array<float, 3> getLastValues();
	std::array<float, 3> getLastImpact();

	void setHighGThreshold(float);
	float getHightGThreshold(void);

	void setExtremGThreshold(float);
	float getExtremGThreshold(void);

	void setLowGThreshold(float);



	// private methods
private:
	float calcAbsAccel(AccelVector);
	float calcAbsGyro(GyroVector);
	float calcAbsGravityForce(float);

	// private member attrib
private:
	int _tick;
	float _impactAccel;
	bool _potImpact;
	int _pollRate;
	int _duration;

	float _lowGThreshold;
	float _highGThreshold;
	float _extremeGThreshold;

	AccelVector _lastAccelVector;
	AccelVector _curAccelVector;
	GyroVector _lastGyroVector;
	GyroVector _curGyroVector;
	LastImpact _lastImpact;

	float _lastAbsAccel;
	float _curAbsAccel;
	float _lastAbsGyro;
	float _curAbsGyro;
	float _lastAbsGravityForce;
	float _curAbsGravityForce;

	float _lastImpactAbsForce;
	float _lastImpactAbsAccel;
	float _lastImpactAbsGyro;
};

#endif
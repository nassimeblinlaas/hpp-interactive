#ifndef ARMCONFIGURATIONFORCE_H
#define ARMCONFIGURATIONFORCE_H

#include <math.h>

class ArmConfigurationForce
{
protected:
	float position[3];
	float quaternion[4];
	float force[3];
public:
	ArmConfigurationForce();
	void setPosition(float x, float y, float z);
	void setQuaternion(float x1, float x2, float x3, float x4);
	void setForce(float x, float y, float z);


	float getPositionByIndex(int index);
	float getQuaternionByIndex(int index);
	const float(&getForce())[3];

	float getDisparity(ArmConfigurationForce a);

	bool operator==(ArmConfigurationForce &a);
	ArmConfigurationForce& operator=(ArmConfigurationForce &a);
};

#endif

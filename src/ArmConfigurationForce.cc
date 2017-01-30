#include <hpp/interactive/ArmConfigurationForce.hh>

ArmConfigurationForce::ArmConfigurationForce()
{
	for (int i = 0; i < 3; i++)
	{
		position[i] = 0.0;
		force[i] = 0.0;
		quaternion[i] = 0.0;
	}
	quaternion[3] = 0.0;
}

void ArmConfigurationForce::setForce(float x, float y, float z)
{
	force[0] = x;
	force[1] = y;
	force[2] = z;
}

void ArmConfigurationForce::setPosition(float x, float y, float z)
{
	position[0] = x;
	position[1] = y;
	position[2] = z;
}

void ArmConfigurationForce::setQuaternion(float x1, float x2, float x3, float x4)
{
	quaternion[0] = x1;
	quaternion[1] = x2;
	quaternion[2] = x3;
	quaternion[3] = x4;
}

float ArmConfigurationForce::getForceByIndex(int index)
{
	return force[index];
}

float ArmConfigurationForce::getPositionByIndex(int index)
{
	return position[index];
}

float ArmConfigurationForce::getQuaternionByIndex(int index)
{
	return quaternion[index];
}

bool ArmConfigurationForce::operator==(ArmConfigurationForce &a)
{

	// if position change is smaller than a coefficient, position doesn't change
	float pos_change = sqrt(pow((position[0] - a.position[0]), 2) + pow((position[1] - a.position[1]), 2) + pow((position[2] - a.position[2]), 2));

	if (pos_change <= 0.001)
		return true;
	else
		return false;
}

ArmConfigurationForce& ArmConfigurationForce::operator=(ArmConfigurationForce &a)
{
	for (int i = 0; i < 3; i++)
	{
		position[i] = a.position[i];
		force[i] = a.force[i];
		quaternion[i] = a.quaternion[i];
	}
	quaternion[3] = a.quaternion[3];
	
	return *this;
}

float ArmConfigurationForce::getDisparity(ArmConfigurationForce a)
{
	return sqrt(pow((position[0] - a.position[0]), 2) + pow((position[1] - a.position[1]), 2) + pow((position[2] - a.position[2]), 2));
}

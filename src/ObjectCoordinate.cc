#include <hpp/interactive/ObjectCoordinate.hh>

ObjectCoordinate::ObjectCoordinate()
{
	position[0] = 0.0;
	position[1] = 0.0;
	position[2] = 0.0;
	quaternion[0] = 0.0;
	quaternion[1] = 0.0;
	quaternion[2] = 0.0;
	quaternion[3] = 0.0;
}

void ObjectCoordinate::setPosition(float x, float y, float z)
{
	position[0] = x;
	position[1] = y;
	position[2] = z;
}

void ObjectCoordinate::setQuaternion(float x, float y, float z, float w)
{
	quaternion[0] = w;
	quaternion[1] = x;
	quaternion[2] = y;
	quaternion[3] = z;
}

float* ObjectCoordinate::getPosition()
{
	return position;
}

float* ObjectCoordinate::getQuaternion()
{
	return quaternion;
}


ObjectCoordinate& ObjectCoordinate::operator= (ObjectCoordinate& a)
{
	position[0] = a.position[0];
	position[1] = a.position[1];
	position[2] = a.position[2];
	quaternion[0] = a.quaternion[0];
	quaternion[1] = a.quaternion[1];
	quaternion[2] = a.quaternion[2];
	quaternion[3] = a.quaternion[3];
	return *this;
}

bool ObjectCoordinate::operator== (ObjectCoordinate& a)
{
	for (int i = 0; i < 7; i++)
	{
		if (i <= 2 && position[i] != a.position[i])
			return false;
		if (i > 2 && quaternion[i-3] != a.quaternion[i-3] )
			return false;
	}
	return true;
}

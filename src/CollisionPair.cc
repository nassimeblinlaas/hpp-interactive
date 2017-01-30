#include <hpp/interactive/CollisionPair.hh>

CollsionPair::CollsionPair()
{
	type = 0;
	pos_on_obj1[0] = 0.0;
	pos_on_obj1[1] = 0.0;
	pos_on_obj1[2] = 0.0;
	pos_on_obj2[0] = 0.0;
	pos_on_obj2[1] = 0.0;
	pos_on_obj2[2] = 0.0;
	normal_on_obj1[0] = 0.0;
	normal_on_obj1[1] = 0.0;
	normal_on_obj1[2] = 0.0;
	normal_on_obj2[0] = 0.0;
	normal_on_obj2[1] = 0.0;
	normal_on_obj2[2] = 0.0;
}


void CollsionPair::setType(int type)
{
	this->type = type;
}
void CollsionPair::setPositionOnObj1(float x, float y, float z)
{
	pos_on_obj1[0] = x;
	pos_on_obj1[1] = y;
	pos_on_obj1[2] = z;
}

void CollsionPair::setPositionOnObj2(float x, float y, float z)
{
	pos_on_obj2[0] = x;
	pos_on_obj2[1] = y;
	pos_on_obj2[2] = z;
}

void CollsionPair::setNormalOnObj1(float x, float y, float z)
{
	normal_on_obj1[0] = x;
	normal_on_obj1[1] = y;
	normal_on_obj1[2] = z;
}

void CollsionPair::setNormalOnObj2(float x, float y, float z)
{
	normal_on_obj2[0] = x;
	normal_on_obj2[1] = y;
	normal_on_obj2[2] = z;
}

int CollsionPair::getType()
{
	return type;
}

float* CollsionPair::getPositionOnObj1()
{
	return pos_on_obj1;
}

float* CollsionPair::getPositionOnObj2()
{
	return pos_on_obj2;
}

float* CollsionPair::getNormalOnObj1()
{
	return normal_on_obj1;
}

float* CollsionPair::getNormalOnObj2()
{
	return normal_on_obj2;
}


CollsionPair& CollsionPair::operator= (CollsionPair& a)
{
	type = a.type;
	pos_on_obj1[0] = a.pos_on_obj1[0];
	pos_on_obj1[1] = a.pos_on_obj1[1];
	pos_on_obj1[2] = a.pos_on_obj1[2];
	pos_on_obj2[0] = a.pos_on_obj2[0];
	pos_on_obj2[1] = a.pos_on_obj2[1];
	pos_on_obj2[2] = a.pos_on_obj2[2];
	normal_on_obj1[0] = a.normal_on_obj1[0];
	normal_on_obj1[1] = a.normal_on_obj1[1];
	normal_on_obj1[2] = a.normal_on_obj1[2];
	normal_on_obj2[0] = a.normal_on_obj2[0];
	normal_on_obj2[1] = a.normal_on_obj2[1];
	normal_on_obj2[2] = a.normal_on_obj2[2];
	return *this;
}

bool CollsionPair::operator== (CollsionPair& a)
{
	for (int i = 0; i < 3; i++)
	{
		if (pos_on_obj1[i] == a.pos_on_obj1[i] && pos_on_obj2[i] == a.pos_on_obj2[i]
		&& normal_on_obj1[i] == a.normal_on_obj1[i] && normal_on_obj2[i] == a.normal_on_obj2[i])
			continue;
		else
			return false;
	}

	if (type != a.type)
		return false;

	return true;
}

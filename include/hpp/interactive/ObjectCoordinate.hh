#ifndef OBJECT_COORDINATE_H
#define OBJECT_COORDINATE_H

class ObjectCoordinate
{
protected:
	float position[3];
	float quaternion[4];

public:
	ObjectCoordinate();
	void setPosition(float x, float y, float z);
	void setQuaternion(float x, float y, float z, float w);

	float* getPosition();
	float* getQuaternion();


	ObjectCoordinate& operator= (ObjectCoordinate& a);
	bool operator== (ObjectCoordinate& a);

};

#endif
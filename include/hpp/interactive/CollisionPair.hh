#ifndef COLLISION_PAIR_H
#define COLLISION_PAIR_H

class CollsionPair
{
protected:
	int type;
	float pos_on_obj1[3];
	float pos_on_obj2[3];
	float normal_on_obj1[3];
	float normal_on_obj2[3];

public:
	CollsionPair();
	void setType(int type);
	void setPositionOnObj1(float x, float y, float z);
	void setPositionOnObj2(float x, float y, float z);
	void setNormalOnObj1(float x, float y, float z);
	void setNormalOnObj2(float x, float y, float z);

	int getType();
	float* getPositionOnObj1();
	float* getPositionOnObj2();
	float* getNormalOnObj1();
	float* getNormalOnObj2();


	CollsionPair& operator= (CollsionPair& a);
	bool operator== (CollsionPair& a);
};

#endif
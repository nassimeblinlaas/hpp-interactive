#ifndef DATA_TRANSIMITE_H
#define DATA_TRANSIMITE_H

#include <hpp/interactive/ObjectCoordinate.hh>
#include <hpp/interactive/ArmConfigurationForce.hh>
#include <hpp/interactive/CollisionPair.hh>

class Data
{
protected:
	ObjectCoordinate objCoordinate;
	ArmConfigurationForce armCF;
	CollsionPair collisionPointArray[512];
	int arraySize;

public:
	Data();
	
	void setObjCoordinate(ObjectCoordinate objCoordinate);
	void setArmConfigurationForce(ArmConfigurationForce armCF);
	void addNewCollsionPair(CollsionPair c_pair);

	ObjectCoordinate getObjCooridnate();
	ArmConfigurationForce getArmCF();
	CollsionPair getCollisionPairByIndex(int index);
	int getCollisionArraySize();

};

#endif

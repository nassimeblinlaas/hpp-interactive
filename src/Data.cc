#include <hpp/interactive/Data.hh>

Data::Data()
{
	arraySize = 0;
}

void Data::setObjCoordinate(ObjectCoordinate objCoordinate)
{
	this->objCoordinate = objCoordinate;
}

void Data::setArmConfigurationForce(ArmConfigurationForce armCF)
{
	this->armCF = armCF;
}

void Data::addNewCollsionPair(CollsionPair c_pair)
{
	collisionPointArray[arraySize++] = c_pair;
}

ObjectCoordinate Data::getObjCooridnate()
{
	return objCoordinate;
}

ArmConfigurationForce Data::getArmCF()
{
	return armCF;
}

CollsionPair Data::getCollisionPairByIndex(int index)
{
	if (index >= arraySize)
	{
		CollsionPair zero;
		return zero;
	}
	else
	{
		return collisionPointArray[index];
	}
}

int Data::getCollisionArraySize()
{
	return arraySize;
}

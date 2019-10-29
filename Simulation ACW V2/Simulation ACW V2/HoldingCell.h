#pragma once

#include "RigidBody.h"
#include <vector>

class HoldingCell
{
private:
	std::vector<RigidBody *> mRigidBodies;
	Vector3F mLocation;
	Vector3F mSize;

public:
	HoldingCell(Vector3F pLocation, Vector3F pSize);
	~HoldingCell();

	void addRigidBody(RigidBody * pRigidBody);

	std::vector<RigidBody *> update();

	int getNumberRigidBody()
	{
		return mRigidBodies.size();
	}

	Vector3F getLocation()
	{
		return mLocation;
	}
};


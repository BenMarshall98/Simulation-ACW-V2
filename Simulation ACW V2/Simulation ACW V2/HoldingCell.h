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

	void update(std::vector<RigidBody *> & pToReassign);

	int getNumberRigidBody()
	{
		return mRigidBodies.size();
	}

	Vector3F getLocation()
	{
		return mLocation;
	}
};


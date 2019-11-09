#pragma once

#include "RigidBody.h"
#include <vector>

class HoldingCell
{
private:
	std::vector<RigidBody *> mRigidBodies;
	glm::vec3 mLocation;
	glm::vec3 mSize;

public:
	HoldingCell(glm::vec3 pLocation, glm::vec3 pSize);
	~HoldingCell();

	void addRigidBody(RigidBody * pRigidBody);

	void update(std::vector<RigidBody *> & pToReassign);

	int getNumberRigidBody()
	{
		return mRigidBodies.size();
	}

	glm::vec3 getLocation()
	{
		return mLocation;
	}
};


#include "HoldingCell.h"



HoldingCell::HoldingCell(Vector3F pLocation, Vector3F pSize) :
	mLocation(pLocation), mSize(pSize)
{
}

HoldingCell::~HoldingCell()
{
}

void HoldingCell::addRigidBody(RigidBody* pRigidBody)
{
	mRigidBodies.push_back(pRigidBody);
}

std::vector<RigidBody *> HoldingCell::update()
{
	std::vector<RigidBody *> toReassign;

	for (int i = 0; i < mRigidBodies.size(); i++)
	{
		Vector3F pos = mRigidBodies[i]->getPos();

		if (pos.getX() > mLocation.getX() + mSize.getX() ||
			pos.getX() < mLocation.getX() - mSize.getX() ||
			pos.getY() > mLocation.getY() + mSize.getY() ||
			pos.getY() < mLocation.getY() - mSize.getY() ||
			pos.getZ() > mLocation.getZ() + mSize.getZ() ||
			pos.getZ() < mLocation.getZ() - mSize.getZ())
		{
			toReassign.push_back(mRigidBodies[i]);
			mRigidBodies.erase(mRigidBodies.begin() + i);
			i--;
		}
	}

	return toReassign;
}
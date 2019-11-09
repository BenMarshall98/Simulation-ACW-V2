#include "HoldingCell.h"



HoldingCell::HoldingCell(const Vector3F pLocation, const Vector3F pSize) :
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

void HoldingCell::update(std::vector<RigidBody *> & pToReassign)
{
	for (auto i = 0u; i < mRigidBodies.size(); i++)
	{
		auto pos = mRigidBodies[i]->getPos();

		if (pos.getX() > mLocation.getX() + mSize.getX() ||
			pos.getX() < mLocation.getX() - mSize.getX() ||
			pos.getY() > mLocation.getY() + mSize.getY() ||
			pos.getY() < mLocation.getY() - mSize.getY() ||
			pos.getZ() > mLocation.getZ() + mSize.getZ() ||
			pos.getZ() < mLocation.getZ() - mSize.getZ())
		{
			pToReassign.push_back(mRigidBodies[i]);
			mRigidBodies.erase(mRigidBodies.begin() + i);
			mRigidBodies.shrink_to_fit();
			i--;
		}
	}

}
#include "HoldingCell.h"



HoldingCell::HoldingCell(const glm::vec3 pLocation, const glm::vec3 pSize) :
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

		if (pos.x > mLocation.x + mSize.x ||
			pos.x < mLocation.x - mSize.x ||
			pos.y > mLocation.y + mSize.y ||
			pos.y < mLocation.y - mSize.y ||
			pos.z > mLocation.z + mSize.z ||
			pos.z < mLocation.z - mSize.z)
		{
			pToReassign.push_back(mRigidBodies[i]);
			mRigidBodies.erase(mRigidBodies.begin() + i);
			mRigidBodies.shrink_to_fit();
			i--;
		}
	}

}
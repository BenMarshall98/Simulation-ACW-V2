#include "HoldingContainer.h"
#include "Sphere.h"
#include "Game.h"

HoldingContainer::HoldingContainer(Octree * pOctree, Vector3F pLocation, Vector3F pSize) :
	mOctree(pOctree), mLocation(pLocation), mSize(pSize)
{
	Vector3F location = Vector3F(pLocation.getX(), pLocation.getY() - pSize.getY() * 2, pLocation.getZ());
	Vector3F size = Vector3F(1, 1, 1);

	mHoldingCells.emplace_back(new HoldingCell(location, size));

	for (auto i = -pSize.getY(); i <= pSize.getY(); i++)
	{
		for (auto j = -pSize.getX(); j <= pSize.getX(); j++)
		{
			for (auto k = -pSize.getZ(); k <= pSize.getZ(); k++)
			{
				if (j == 0 && k == 0 && i == -pSize.getY())
				{
					continue;
				}

				location = Vector3F(pLocation.getX() + j * 2, pLocation.getY() + i * 2, pLocation.getZ() + k * 2);

				mHoldingCells.emplace_back(new HoldingCell(location, size));
			}
		}
	}
}

HoldingContainer::~HoldingContainer()
{
}

void HoldingContainer::addRigidBody()
{
	for (int i = 0; i < mHoldingCells.size(); i++)
	{
		if (mHoldingCells[i]->getNumberRigidBody() == 0)
		{
			//TODO: Look at random velocity
			Vector3F velocity = Vector3F(0, 0, 0);
			Sphere * sphere = new Sphere(Game::getSphereSize(), 0.02, mHoldingCells[i]->getLocation(),
				Vector3F(0, 1, 0), velocity);

			mHoldingCells[i]->addRigidBody(sphere);
			mOctree->AddRigidBody(sphere);
			return;
		}
	}

	mOverflow++;
}

void HoldingContainer::update()
{
	std::vector<RigidBody *> toReassign;
	for (int i = 0; i < mHoldingCells.size(); i++)
	{
		mHoldingCells[i]->update(toReassign);

		for (int j = 0; j < toReassign.size(); j++)
		{
			for (int k = 0; k < mHoldingCells.size(); k++)
			{
				Vector3F location = mHoldingCells[k]->getLocation();
				Vector3F pos = toReassign[j]->getPos();

				if (pos.getX() <= location.getX() + 1 &&
					pos.getX() >= location.getX() - 1 &&
					pos.getY() <= location.getY() + 1 &&
					pos.getY() >= location.getY() - 1 &&
					pos.getZ() <= location.getZ() + 1 &&
					pos.getZ() >= location.getZ() - 1)
				{
					mHoldingCells[k]->addRigidBody(toReassign[j]);
					break;
				}
			}
		}

		toReassign.clear();
		toReassign.shrink_to_fit();
	}

	for (int i = 0; i < mOverflow; i++)
	{
		for (int k = 0; k < mHoldingCells.size(); k++)
		{
			if (mHoldingCells[k]->getNumberRigidBody() == 0)
			{
				//TODO: Look at random velocity
				Vector3F velocity = Vector3F(0, 0, 0);
				Sphere * sphere = new Sphere(Game::getSphereSize(), 0.02, mHoldingCells[k]->getLocation(),
					Vector3F(0, 1, 0), velocity);

				mHoldingCells[k]->addRigidBody(sphere);
				mOctree->AddRigidBody(sphere);

				mOverflow--;
				i--;
				break;
			}
		}
	}
}

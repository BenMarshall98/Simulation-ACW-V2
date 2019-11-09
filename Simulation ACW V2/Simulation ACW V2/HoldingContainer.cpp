#include "HoldingContainer.h"
#include "Sphere.h"
#include "Game.h"

HoldingContainer::HoldingContainer(Octree * pOctree, const Vector3F pLocation, const Vector3F pSize) :
	mOctree(pOctree), mLocation(pLocation), mSize(pSize), mOverflow(0)
{
	auto location = Vector3F(pLocation.getX(), pLocation.getY() - pSize.getY() * 2, pLocation.getZ());
	const auto size = Vector3F(1, 1, 1);

	mHoldingCells.emplace_back(new HoldingCell(location, size));

	for (auto i = -pSize.getY(); i <= pSize.getY(); ++i)
	{
		for (auto j = -pSize.getX(); j <= pSize.getX(); ++j)
		{
			for (auto k = -pSize.getZ(); k <= pSize.getZ(); ++k)
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
			const auto velocity = Vector3F(0, 0, 0);
			const auto sphere = new Sphere(Game::getSphereSize(), 0.02, mHoldingCells[i]->getLocation(),
				Vector3F(0, 1, 1), velocity);

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
	for (auto& holdingCell : mHoldingCells)
	{
		holdingCell->update(toReassign);

		for (auto& j : toReassign)
		{
			for (auto& k : mHoldingCells)
			{
				auto location = k->getLocation();
				auto pos = j->getPos();

				if (pos.getX() <= location.getX() + 1 &&
					pos.getX() >= location.getX() - 1 &&
					pos.getY() <= location.getY() + 1 &&
					pos.getY() >= location.getY() - 1 &&
					pos.getZ() <= location.getZ() + 1 &&
					pos.getZ() >= location.getZ() - 1)
				{
					k->addRigidBody(j);
					break;
				}
			}
		}

		toReassign.clear();
		toReassign.shrink_to_fit();
	}

	for (auto i = 0; i < mOverflow; i++)
	{
		for (auto& holdingCell : mHoldingCells)
		{
			if (holdingCell->getNumberRigidBody() == 0)
			{
				//TODO: Look at random velocity
				const auto velocity = Vector3F(0, 0, 0);
				const auto sphere = new Sphere(Game::getSphereSize(), 0.02, holdingCell->getLocation(),
					Vector3F(0, 1, 1), velocity);

				holdingCell->addRigidBody(sphere);
				mOctree->AddRigidBody(sphere);

				mOverflow--;
				i--;
				break;
			}
		}
	}
}

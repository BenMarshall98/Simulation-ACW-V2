#include "HoldingContainer.h"
#include "Sphere.h"
#include "Game.h"
#include "Cuboid.h"

HoldingContainer::HoldingContainer(Octree * pOctree, const glm::vec3 pLocation, const glm::vec3 pSize) :
	mOctree(pOctree), mLocation(pLocation), mSize(pSize), mSphereOverflow(0), mCubeOverflow(0)
{
	mNumber = std::uniform_real_distribution<float>(0.0f, 1.0f);
	
	auto location = glm::vec3(pLocation.x, pLocation.y - pSize.y * 2, pLocation.z);
	const auto size = glm::vec3(1, 1, 1);

	mHoldingCells.emplace_back(new HoldingCell(location, size));

	for (auto i = -pSize.y; i <= pSize.y; ++i)
	{
		for (auto j = -pSize.x; j <= pSize.x; ++j)
		{
			for (auto k = -pSize.z; k <= pSize.z; ++k)
			{
				if (j == 0 && k == 0 && i == -pSize.y)
				{
					continue;
				}

				location = glm::vec3(pLocation.x + j * 2, pLocation.y + i * 2, pLocation.z + k * 2);

				mHoldingCells.emplace_back(new HoldingCell(location, size));
			}
		}
	}
}

HoldingContainer::~HoldingContainer()
{
	for (auto cell : mHoldingCells)
	{
		delete cell;
	}
}

void HoldingContainer::addSphere()
{
	for (auto i = 0u; i < mHoldingCells.size(); i++)
	{
		if (mHoldingCells[i]->getNumberRigidBody() == 0)
		{
			const auto color = glm::vec3(mNumber(mEngine), mNumber(mEngine), mNumber(mEngine));
			const auto velocity = glm::vec3(mNumber(mEngine), 0, mNumber(mEngine));
			auto angularVelocity = glm::vec3(mNumber(mEngine), mNumber(mEngine), mNumber(mEngine));

			angularVelocity /= 100.0f;
			
			const auto sphere = new Sphere(Game::getSphereSize(), 0.02, mHoldingCells[i]->getLocation(),
				angularVelocity, velocity, color);

			mHoldingCells[i]->addRigidBody(sphere);
			mOctree->addRigidBody(sphere);
			return;
		}
	}

	mSphereOverflow++;
}

void HoldingContainer::addCube()
{
	for (auto i = 0u; i < mHoldingCells.size(); i++)
	{
		if (mHoldingCells[i]->getNumberRigidBody() == 0)
		{
			const auto velocity = glm::vec3(0, 0, 0);
			const auto size = 0.25f;
			const auto cube = new Cuboid(glm::vec3(size, size, size), 0.02, mHoldingCells[i]->getLocation(),
				glm::vec3(0, 0.01, 0.01), velocity);

			mHoldingCells[i]->addRigidBody(cube);
			mOctree->addRigidBody(cube);
			return;
		}
	}

	mCubeOverflow++;
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

				if (pos.x <= location.x + 1 &&
					pos.x >= location.x - 1 &&
					pos.y <= location.y + 1 &&
					pos.y >= location.y - 1 &&
					pos.z <= location.z + 1 &&
					pos.z >= location.z - 1)
				{
					k->addRigidBody(j);
					break;
				}
			}
		}

		toReassign.clear();
		toReassign.shrink_to_fit();
	}

	for (auto i = 0; i < mSphereOverflow; i++)
	{
		for (auto& holdingCell : mHoldingCells)
		{
			if (holdingCell->getNumberRigidBody() == 0)
			{
				const auto color = glm::vec3(mNumber(mEngine), mNumber(mEngine), mNumber(mEngine));
				const auto velocity = glm::vec3(mNumber(mEngine), 0, mNumber(mEngine));
				auto angularVelocity = glm::vec3(mNumber(mEngine), mNumber(mEngine), mNumber(mEngine));

				angularVelocity /= 100.f;
				
				//TODO: Look at random velocity
				const auto sphere = new Sphere(Game::getSphereSize(), 0.02, holdingCell->getLocation(),
					angularVelocity, velocity, color);

				holdingCell->addRigidBody(sphere);
				mOctree->addRigidBody(sphere);

				mSphereOverflow--;
				i--;
				break;
			}
		}
	}

	for (auto i = 0; i < mCubeOverflow; i++)
	{
		for (auto& holdingCell : mHoldingCells)
		{
			if (holdingCell->getNumberRigidBody() == 0)
			{
				//TODO: Look at random velocity
				const auto velocity = glm::vec3(0, 0, 0);
				const auto size = Game::getSphereSize();
				const auto sphere = new Cuboid(glm::vec3(size, size, size), 0.02, holdingCell->getLocation(),
					glm::vec3(0, 0.01, 0.01), velocity);

				holdingCell->addRigidBody(sphere);
				mOctree->addRigidBody(sphere);

				mCubeOverflow--;
				i--;
				break;
			}
		}
	}
}

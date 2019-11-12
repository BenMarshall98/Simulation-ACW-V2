#pragma once
#include "Octree.h"
#include "HoldingCell.h"
#include <random>
#include <functional>

class HoldingContainer
{
	Octree * mOctree;
	glm::vec3 mLocation;
	glm::vec3 mSize;
	int mSphereOverflow;
	int mCubeOverflow;
	std::vector<HoldingCell *> mHoldingCells;

public:
	HoldingContainer(Octree * pOctree, glm::vec3 pLocation, glm::vec3 pSize);
	~HoldingContainer();

	void addSphere();
	void addCube();

	void update();
};


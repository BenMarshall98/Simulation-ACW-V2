#pragma once
#include "Octree.h"
#include "HoldingCell.h"
#include <random>
#include <functional>

class HoldingContainer
{
private:
	Octree * mOctree;
	Vector3F mLocation;
	Vector3F mSize;
	int mOverflow;
	std::vector<HoldingCell *> mHoldingCells;

public:
	HoldingContainer(Octree * pOctree, Vector3F pLocation, Vector3F pSize);
	~HoldingContainer();

	void addRigidBody();

	void update();
};


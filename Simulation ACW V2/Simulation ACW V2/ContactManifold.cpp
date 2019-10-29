#include "ContactManifold.h"

ContactManifold::ContactManifold() : mPoints{}, mNumOfPoints(0)
{
}

ContactManifold::~ContactManifold() = default;

void ContactManifold::add(const ManifoldPoint & pPoint)
{
	mPoints[mNumOfPoints] = pPoint;
	++mNumOfPoints;
}

void ContactManifold::clear()
{
	mNumOfPoints = 0;
}

int ContactManifold::getNumPoints() const
{
	return mNumOfPoints;
}

ManifoldPoint & ContactManifold::getPoint(const int pIndex)
{
	return mPoints[pIndex];
}
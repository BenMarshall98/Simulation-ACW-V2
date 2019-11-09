#include "ContactManifold.h"
#include "algorithm"

ContactManifold::ContactManifold() : mNumOfPoints(0)
{
}

ContactManifold::~ContactManifold() = default;

void ContactManifold::add(const ManifoldPoint & pPoint)
{
	mPoints.push_back(pPoint);
	++mNumOfPoints;
}

void ContactManifold::remove(const int pIndex)
{
	mPoints.erase(mPoints.begin() + pIndex);
	mNumOfPoints--;
}


void ContactManifold::clear()
{
	mPoints.clear();
	mPoints.shrink_to_fit();
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

void ContactManifold::sort()
{
	std::sort(mPoints.begin(), mPoints.end(), 
		[](const ManifoldPoint pPoint1, const ManifoldPoint pPoint2)
	{
		return pPoint1.mTime < pPoint2.mTime;
	});
}
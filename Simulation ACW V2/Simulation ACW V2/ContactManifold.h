#pragma once

#include "Vector3F.h"
#include <vector>

class RigidBody;

enum class CollisionType
{
	PENETRATION,
	COLLISION
};

struct ManifoldPoint
{
	RigidBody * mContactId1;
	RigidBody * mContactId2;
	Vector3F mContactPoint1;
	Vector3F mContactPoint2;
	Vector3F mContactNormal;
	float mTime;
	float mCollisionDepth;
	CollisionType mCollisionType;
};

class ContactManifold
{
public:
	ContactManifold();
	~ContactManifold();

	ContactManifold(const ContactManifold &) = delete;
	ContactManifold(ContactManifold &&) = delete;
	ContactManifold & operator= (const ContactManifold &) = delete;
	ContactManifold & operator= (ContactManifold &&) = delete;

	void add(const ManifoldPoint& pPoint);
	void remove(int pIndex);
	void clear();
	void sort();
	int getNumPoints() const;
	ManifoldPoint& getPoint(int pIndex);

private:
	std::vector<ManifoldPoint> mPoints;
	int mNumOfPoints;
};


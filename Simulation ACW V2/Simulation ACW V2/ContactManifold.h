#pragma once

#include "Vector3f.h"

class RigidBody;

struct ManifoldPoint
{
	RigidBody * mContactId1;
	RigidBody * mContactId2;
	Vector3F mContactNormal;
	float mTime;
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
	void clear();
	int getNumPoints() const;
	ManifoldPoint& getPoint(int pIndex);

private:
	ManifoldPoint mPoints[1000];
	int mNumOfPoints;
};


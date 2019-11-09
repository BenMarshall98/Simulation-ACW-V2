#pragma once

#include "glm/glm.hpp"
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
	glm::vec3 mContactPoint1;
	glm::vec3 mContactPoint2;
	glm::vec3 mContactNormal;
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


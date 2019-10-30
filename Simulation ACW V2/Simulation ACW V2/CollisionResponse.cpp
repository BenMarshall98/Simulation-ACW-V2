#include "CollisionResponse.h"
#include "Game.h"

void CollisionResponse::dynamicCollisionResponse(ManifoldPoint& pPoint, bool & moved1, bool & moved2)
{
	auto * rigidBody1 = pPoint.mContactId1;
	auto * rigidBody2 = pPoint.mContactId2;

	if (rigidBody1->getObjectType() == ObjectType::SPHERE && rigidBody2->getObjectType() == ObjectType::SPHERE)
	{
		respondCollisionSphereSphere(pPoint, rigidBody1, rigidBody2, moved1, moved1);
	}
	else if (rigidBody1->getObjectType() == ObjectType::SPHERE && rigidBody2->getObjectType() == ObjectType::PLANE)
	{
		respondCollisionSpherePlane(pPoint, rigidBody1, rigidBody2, moved1, moved2);
	}
	else if (rigidBody1->getObjectType() == ObjectType::PLANE && rigidBody2->getObjectType() == ObjectType::SPHERE)
	{
		respondCollisionSpherePlane(pPoint, rigidBody2, rigidBody1, moved2, moved1);
	}
	else if (rigidBody1->getObjectType() == ObjectType::SPHERE && rigidBody2->getObjectType() == ObjectType::PLANEHOLES)
	{
		respondCollisionSpherePlaneHoles(pPoint, rigidBody1, rigidBody2, moved1, moved2);
	}
	else if (rigidBody1->getObjectType() == ObjectType::PLANEHOLES && rigidBody2->getObjectType() == ObjectType::SPHERE)
	{
		respondCollisionSpherePlaneHoles(pPoint, rigidBody2, rigidBody1, moved2, moved1);
	}
}

void CollisionResponse::respondCollisionSphereSphere(ManifoldPoint& pPoint, RigidBody * pSphere1, RigidBody * pSphere2, bool & moved1, bool & moved2)
{
	Vector3F changePos1 = pSphere1->getNewPos() - pSphere1->getPos();
	Vector3F changePos2 = pSphere2->getNewPos() - pSphere2->getPos();

	Vector3F changeVel1 = pSphere1->getNewVel() - pSphere1->getVel();
	Vector3F changeVel2 = pSphere2->getNewVel() - pSphere2->getVel();

	if (changeVel1.dot(pPoint.mContactNormal) == 0 &&
		changeVel2.dot(pPoint.mContactNormal) == 0)
	{
		return; //Movement is perpendicular to the line of impact;
	}

	Vector3F initialVel1 = pSphere1->getVel() + (changeVel1 * pPoint.mTime);
	Vector3F initialVel2 = pSphere2->getVel() + (changeVel2 * pPoint.mTime);

	float sphereMass1 = pSphere1->getMass();
	float sphereMass2 = pSphere2->getMass();

	Vector3F velocityLine1 = (((sphereMass1 - 0.8 * sphereMass2) * initialVel1.dot(pPoint.mContactNormal) * pPoint.mContactNormal) + ((sphereMass2 + 0.8 * sphereMass2) * initialVel2.dot(pPoint.mContactNormal) * pPoint.mContactNormal)) / (sphereMass1 + sphereMass2);
	Vector3F velocityLine2 = (((sphereMass1 + 0.8 * sphereMass1) * initialVel1.dot(pPoint.mContactNormal) * pPoint.mContactNormal) + ((sphereMass2 - 0.8 * sphereMass1) * initialVel2.dot(pPoint.mContactNormal) * pPoint.mContactNormal)) / (sphereMass1 + sphereMass2);

	Vector3F newVel1 = initialVel1 - (initialVel1.dot(pPoint.mContactNormal) * pPoint.mContactNormal) + velocityLine1;
	Vector3F newVel2 = initialVel2 - (initialVel2.dot(pPoint.mContactNormal) * pPoint.mContactNormal) + velocityLine2;

	Vector3F newPos1 = pSphere1->getPos() + (changePos1 * pPoint.mTime);
	Vector3F newPos2 = pSphere2->getPos() + (changePos2 * pPoint.mTime);

	pSphere1->setNewPos(newPos1);
	pSphere1->setNewVel(newVel1);

	pSphere2->setNewPos(newPos2);
	pSphere2->setNewVel(newVel2);

	moved1 = true;
	moved2 = true;
}

void CollisionResponse::respondCollisionSpherePlane(ManifoldPoint& pPoint, RigidBody * pSphere, RigidBody * pPlane, bool & moved1, bool & moved2)
{

	Vector3F changePos = pSphere->getNewPos() - pSphere->getPos();
	Vector3F changeVel = pSphere->getNewVel() - pSphere->getVel();

	Vector3F newPos = pSphere->getPos() + (changePos * pPoint.mTime);

	Vector3F newVel = pSphere->getVel() + (changeVel * pPoint.mTime);

	newVel = newVel - (1 + 0.8) * (newVel.dot(pPoint.mContactNormal)) * pPoint.mContactNormal;

	pSphere->setNewPos(newPos);
	pSphere->setNewVel(newVel);
}

void CollisionResponse::respondCollisionSpherePlaneHoles(ManifoldPoint& pPoint, RigidBody * pSphere, RigidBody * pPlaneHoles, bool & moved1, bool & moved2)
{
	Vector3F changePos = pSphere->getNewPos() - pSphere->getPos();
	Vector3F changeVel = pSphere->getNewVel() - pSphere->getVel();

	Vector3F tempPos = pSphere->getPos() + (changePos * pPoint.mTime);

	Vector3F tempVel = pSphere->getVel() + (changeVel * pPoint.mTime);

	tempVel = tempVel - (1 + 0.8) * (tempVel.dot(pPoint.mContactNormal)) * pPoint.mContactNormal;

	State state = { tempPos, tempVel };

	float timeLeft = Game::getUpdateDt() - (Game::getUpdateDt() * pPoint.mTime);

	RigidBody::integrate(state, 0.0, timeLeft);

	pSphere->setNewPos(state.pos);
	pSphere->setNewVel(state.vel);
}


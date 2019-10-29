#include "CollisionResponse.h"
#include "Game.h"

void CollisionResponse::dynamicCollisionResponse(ManifoldPoint& pPoint)
{
	auto * rigidBody1 = pPoint.mContactId1;
	auto * rigidBody2 = pPoint.mContactId2;

	if (rigidBody1->getObjectType() == ObjectType::SPHERE && rigidBody2->getObjectType() == ObjectType::SPHERE)
	{
		auto * sphere1 = dynamic_cast<Sphere *>(rigidBody1);
		auto * sphere2 = dynamic_cast<Sphere *>(rigidBody2);

		respondCollisionSphereSphere(sphere1, sphere2, pPoint);
	}
	else if (rigidBody1->getObjectType() == ObjectType::SPHERE && rigidBody2->getObjectType() == ObjectType::PLANE)
	{
		auto * sphere = dynamic_cast<Sphere *>(rigidBody1);
		auto * plane = dynamic_cast<Plane *>(rigidBody2);

		respondCollisionSpherePlane(sphere, plane, pPoint);
	}
	else if (rigidBody1->getObjectType() == ObjectType::PLANE && rigidBody2->getObjectType() == ObjectType::SPHERE)
	{
		auto * sphere = dynamic_cast<Sphere *>(rigidBody2);
		auto * plane = dynamic_cast<Plane *>(rigidBody1);

		respondCollisionSpherePlane(sphere, plane, pPoint);
	}
	else if (rigidBody1->getObjectType() == ObjectType::SPHERE && rigidBody2->getObjectType() == ObjectType::PLANEHOLES)
	{
		auto * sphere = dynamic_cast<Sphere *>(rigidBody1);
		auto * planeHoles = dynamic_cast<PlaneHoles *>(rigidBody2);

		respondCollisionSpherePlaneHoles(sphere, planeHoles, pPoint);
	}
	else if (rigidBody1->getObjectType() == ObjectType::PLANEHOLES && rigidBody2->getObjectType() == ObjectType::SPHERE)
	{
		auto * sphere = dynamic_cast<Sphere *>(rigidBody2);
		auto * planeHoles = dynamic_cast<PlaneHoles *>(rigidBody1);

		respondCollisionSpherePlaneHoles(sphere, planeHoles, pPoint);
	}
}

void CollisionResponse::respondCollisionSphereSphere(Sphere* pSphere1, Sphere* pSphere2, ManifoldPoint& pPoint)
{
	if (pPoint.mTime = 0)
	{
		//TODO: Deal with non-moving object
		pSphere1->setNewPos(pSphere1->getPos());
		pSphere1->setNewVel(pSphere1->getVel());

		pSphere2->setNewPos(pSphere1->getPos());
		pSphere2->setNewVel(pSphere2->getVel());
	}
	else
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

		Vector3F tempVel1 = initialVel1 - (initialVel1.dot(pPoint.mContactNormal) * pPoint.mContactNormal) + velocityLine1;
		Vector3F tempVel2 = initialVel2 - (initialVel2.dot(pPoint.mContactNormal) * pPoint.mContactNormal) + velocityLine2;

		Vector3F tempPos1 = pSphere1->getPos() + (changePos1 * pPoint.mTime);
		Vector3F tempPos2 = pSphere2->getPos() + (changePos2 * pPoint.mTime);

		State state1 = { tempPos1, tempVel1 };
		State state2 = { tempPos2, tempVel2 };

		float timeLeft = Game::getUpdateDt() - (Game::getUpdateDt() * pPoint.mTime);

		RigidBody::integrate(state1, 0.0, timeLeft);
		RigidBody::integrate(state2, 0.0, timeLeft);

		pSphere1->setNewPos(state1.pos);
		pSphere1->setNewVel(state1.vel);

		pSphere2->setNewPos(state2.pos);
		pSphere2->setNewVel(state2.vel);
	}
}

void CollisionResponse::respondCollisionSpherePlane(Sphere* pSphere, Plane* pPlane, ManifoldPoint& pPoint)
{
	if (pPoint.mTime == 0)
	{
		//TODO: Deal with non-moving object
		pSphere->setNewPos(pSphere->getPos());
		pSphere->setNewVel(pSphere->getVel());
	}
	else
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
}

void CollisionResponse::respondCollisionSpherePlaneHoles(Sphere* pSphere, PlaneHoles* pPlaneHoles, ManifoldPoint& pPoint)
{
	if (pPoint.mTime == 0)
	{
		//TODO: Deal with non-moving object
		pSphere->setNewPos(pSphere->getPos());
		pSphere->setNewVel(pSphere->getVel());
	}
	else
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
}


#include "CollisionResponse.h"
#include "Game.h"

//https://www.scss.tcd.ie/~manzkem/CS7057/cs7057-1516-09-CollisionResponse-mm.pdf
//http://citeseerx.ist.psu.edu/viewdoc/download?doi=10.1.1.130.6905&rep=rep1&type=pdf

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
		auto tempRig = pPoint.mContactId1;
		pPoint.mContactId2 = pPoint.mContactId1;
		pPoint.mContactId1 = tempRig;

		auto tempVal = pPoint.mContactPoint1;
		pPoint.mContactPoint2 = pPoint.mContactPoint1;
		pPoint.mContactPoint1 = tempVal;
		
		respondCollisionSpherePlane(pPoint, rigidBody2, rigidBody1, moved2, moved1);
	}
	else if (rigidBody1->getObjectType() == ObjectType::SPHERE && rigidBody2->getObjectType() == ObjectType::PLANEHOLES)
	{
		respondCollisionSpherePlaneHoles(pPoint, rigidBody1, rigidBody2, moved1, moved2);
	}
	else if (rigidBody1->getObjectType() == ObjectType::PLANEHOLES && rigidBody2->getObjectType() == ObjectType::SPHERE)
	{
		auto tempRig = pPoint.mContactId1;
		pPoint.mContactId2 = pPoint.mContactId1;
		pPoint.mContactId1 = tempRig;

		auto tempVal = pPoint.mContactPoint1;
		pPoint.mContactPoint2 = pPoint.mContactPoint1;
		pPoint.mContactPoint1 = tempVal;
		
		respondCollisionSpherePlaneHoles(pPoint, rigidBody2, rigidBody1, moved2, moved1);
	}
	else if (rigidBody1->getObjectType() == ObjectType::SPHERE && rigidBody2->getObjectType() == ObjectType::BOWL)
	{
		respondCollisionSphereBowl(pPoint, rigidBody1, rigidBody2, moved1, moved2);
	}
	else if (rigidBody1->getObjectType() == ObjectType::BOWL && rigidBody2->getObjectType() == ObjectType::SPHERE)
	{
		auto tempRig = pPoint.mContactId1;
		pPoint.mContactId2 = pPoint.mContactId1;
		pPoint.mContactId1 = tempRig;

		auto tempVal = pPoint.mContactPoint1;
		pPoint.mContactPoint2 = pPoint.mContactPoint1;
		pPoint.mContactPoint1 = tempVal;
		respondCollisionSphereBowl(pPoint, rigidBody2, rigidBody1, moved2, moved1);
	}
}

void CollisionResponse::respondCollisionSphereSphere(ManifoldPoint& pPoint, RigidBody * pSphere1, RigidBody * pSphere2, bool & moved1, bool & moved2)
{
	float changeTime1 = pPoint.mTime - pSphere1->getCurrentUpdateTime();
	
	Vector3F tempPos1 = pSphere1->getPos().interpolate(pSphere1->getNewPos(), changeTime1);
	Vector3F tempVel1 = pSphere1->getVel().interpolate(pSphere1->getNewVel(), changeTime1);
	Vector3F tempAngVel1 = pSphere1->getAngularVelocity().interpolate(pSphere1->getNewAngularVelocity(), changeTime1);
	Matrix3F tempOrr1 = pSphere1->getOrientation().interpolate(pSphere1->getNewOrientation(), changeTime1);

	float changeTime2 = pPoint.mTime - pSphere2->getCurrentUpdateTime();

	Vector3F tempPos2 = pSphere2->getPos().interpolate(pSphere2->getNewPos(), changeTime2);
	Vector3F tempVel2 = pSphere2->getVel().interpolate(pSphere2->getNewVel(), changeTime2);
	Vector3F tempAngVel2 = pSphere2->getAngularVelocity().interpolate(pSphere2->getNewAngularVelocity(), changeTime2);
	Matrix3F tempOrr2 = pSphere2->getOrientation().interpolate(pSphere2->getNewOrientation(), changeTime2);

	if (pPoint.mCollisionType == CollisionType::PENETRATION)
	{
		tempPos1 = tempPos1 + pPoint.mContactNormal * 0.5f * pPoint.mCollisionDepth;
		tempPos2 = tempPos2 - pPoint.mContactNormal * 0.5f * pPoint.mCollisionDepth;
	}

	Vector3F sphereCenterToCollision1 = pPoint.mContactPoint1 - tempPos1;
	Vector3F sphereCenterToCollision2 = pPoint.mContactPoint2 - tempPos2;

	Vector3F tempSphereVel1 = tempVel1 + tempAngVel1.cross(sphereCenterToCollision1);
	Vector3F tempSphereVel2 = tempVel2 + tempAngVel2.cross(sphereCenterToCollision2);

	float relVel = pPoint.mContactNormal.dot(tempSphereVel1 - tempSphereVel2);

	Matrix3F sphereWorldTensor1 = tempOrr1 * pSphere1->getInverseImpulseTenser() * tempOrr1.transpose();
	Matrix3F sphereWorldTensor2 = tempOrr2 * pSphere2->getInverseImpulseTenser() * tempOrr2.transpose();

	float sphereInverseMass1 = 1.0f / pSphere1->getMass();
	float sphereInverseMass2 = 1.0f / pSphere2->getMass();

	float sphereImpulseMag1 = pPoint.mContactNormal.dot((sphereWorldTensor1 * sphereCenterToCollision1.cross(pPoint.mContactNormal)).cross(pPoint.mContactNormal));
	float sphereImpulseMag2 = pPoint.mContactNormal.dot((sphereWorldTensor2 * sphereCenterToCollision2.cross(pPoint.mContactNormal)).cross(pPoint.mContactNormal));

	float j = -(1.0f + 0.8f) * relVel / (sphereInverseMass1 + sphereInverseMass2 + sphereImpulseMag1 + sphereImpulseMag2);

	tempVel1 = tempVel1 + (j * sphereInverseMass1 * pPoint.mContactNormal);
	tempVel2 = tempVel2 - (j * sphereInverseMass2 * pPoint.mContactNormal);
	tempAngVel1 = tempAngVel1 + sphereCenterToCollision1.cross(j * pPoint.mContactNormal) * sphereWorldTensor1;
	tempAngVel2 = tempAngVel2 - sphereCenterToCollision2.cross(j * pPoint.mContactNormal) * sphereWorldTensor2;

	pSphere1->setNewPos(tempPos1);
	pSphere1->setNewVel(tempVel1);
	pSphere1->setNewAngularVel(tempAngVel1);
	pSphere1->setNewOrientation(tempOrr1);
						
	pSphere2->setNewPos(tempPos2);
	pSphere2->setNewVel(tempVel2);
	pSphere2->setNewAngularVel(tempAngVel2);
	pSphere2->setNewOrientation(tempOrr2);

	moved1 = true;
	moved2 = true;
}

void CollisionResponse::respondCollisionSphereBowl(ManifoldPoint& pPoint, RigidBody* pSphere, RigidBody* pBowl, bool& moved1, bool& moved2)
{
	float changeTime = pPoint.mTime - pSphere->getCurrentUpdateTime();

	Vector3F tempPos = pSphere->getPos().interpolate(pSphere->getNewPos(), changeTime);
	Vector3F tempVel = pSphere->getVel().interpolate(pSphere->getNewVel(), changeTime);
	Vector3F tempAngVel = pSphere->getAngularVelocity().interpolate(pSphere->getNewAngularVelocity(), changeTime);
	Matrix3F tempOrr = pSphere->getOrientation().interpolate(pSphere->getNewOrientation(), changeTime);

	if (pPoint.mCollisionType == CollisionType::PENETRATION)
	{
		tempPos = tempPos - pPoint.mContactNormal * pPoint.mCollisionDepth;
	}

	Matrix4F bowlMat = pBowl->getMatrix();
	const auto center = pBowl->getPos() * bowlMat;

	Vector3F tempBowlVel = Vector3F(0.0f, 0.0f, 0.0f);

	Vector3F sphereCenterToCollision = pPoint.mContactPoint1 - tempPos;
	Vector3F tempSphereVel = tempVel + tempAngVel.cross(sphereCenterToCollision);
	float relVel = pPoint.mContactNormal.dot(tempSphereVel - tempBowlVel);

	Matrix3F sphereWorldTensor = tempOrr * pSphere->getInverseImpulseTenser() * tempOrr.transpose();

	float sphereInverseMass = 1.0f / pSphere->getMass();

	float sphereImpulseMag = pPoint.mContactNormal.dot((sphereWorldTensor * sphereCenterToCollision.cross(pPoint.mContactNormal)).cross(pPoint.mContactNormal));

	float j = -(1.0f + 0.8f) * relVel / (sphereInverseMass + sphereImpulseMag);

	tempVel = tempVel + (j * sphereInverseMass * pPoint.mContactNormal);
	tempAngVel = tempAngVel + sphereCenterToCollision.cross(j * pPoint.mContactNormal) * sphereWorldTensor;

	pSphere->setNewPos(tempPos);
	pSphere->setNewVel(tempVel);
	pSphere->setNewOrientation(tempOrr);
	pSphere->setNewAngularVel(tempAngVel);

	moved1 = true;
	moved2 = false;
}

void CollisionResponse::respondCollisionSpherePlane(ManifoldPoint& pPoint, RigidBody * pSphere, RigidBody * pPlane, bool & moved1, bool & moved2)
{
	float changeTime = pPoint.mTime - pSphere->getCurrentUpdateTime();

	Vector3F tempPos = pSphere->getPos().interpolate(pSphere->getNewPos(), changeTime);
	Vector3F tempVel = pSphere->getVel().interpolate(pSphere->getNewVel(), changeTime);
	Vector3F tempAngVel = pSphere->getAngularVelocity().interpolate(pSphere->getNewAngularVelocity(), changeTime);
	Matrix3F tempOrr = pSphere->getOrientation().interpolate(pSphere->getNewOrientation(), changeTime);

	if (pPoint.mCollisionType == CollisionType::PENETRATION)
	{
		tempPos = tempPos - pPoint.mContactNormal * pPoint.mCollisionDepth;
	}

	Matrix4F planeMat = pPlane->getMatrix();
	const auto center = pPlane->getPos() * planeMat;

	Matrix4F newPlaneMat = pPlane->getNewMatrix();
	auto newCenter = pPlane->getPos() * newPlaneMat;

	Vector3F tempPlaneVel = (newCenter - center) / changeTime;

	Vector3F sphereCenterToCollision = pPoint.mContactPoint1 - tempPos;
	Vector3F tempSphereVel = tempVel + tempAngVel.cross(sphereCenterToCollision);
	float relVel = pPoint.mContactNormal.dot(tempSphereVel - tempPlaneVel);

	Matrix3F sphereWorldTensor = tempOrr * pSphere->getInverseImpulseTenser() * tempOrr.transpose();

	float sphereInverseMass = 1.0f / pSphere->getMass();

	float sphereImpulseMag = pPoint.mContactNormal.dot((sphereWorldTensor * sphereCenterToCollision.cross(pPoint.mContactNormal)).cross(pPoint.mContactNormal));
	
	float j = -(1.0f + 0.8f) * relVel / (sphereInverseMass + sphereImpulseMag);
	
	tempVel = tempVel + (j * sphereInverseMass * pPoint.mContactNormal);
	tempAngVel = tempAngVel + sphereCenterToCollision.cross(j * pPoint.mContactNormal) * sphereWorldTensor;

	pSphere->setNewPos(tempPos);
	pSphere->setNewVel(tempVel);
	pSphere->setNewOrientation(tempOrr);
	pSphere->setNewAngularVel(tempAngVel);

	moved1 = true;
	moved2 = false;
}

void CollisionResponse::respondCollisionSpherePlaneHoles(ManifoldPoint& pPoint, RigidBody * pSphere, RigidBody * pPlaneHoles, bool & moved1, bool & moved2)
{
	float changeTime = pPoint.mTime - pSphere->getCurrentUpdateTime();
	
	Vector3F tempPos = pSphere->getPos().interpolate(pSphere->getNewPos(), changeTime);
	Vector3F tempVel = pSphere->getVel().interpolate(pSphere->getNewVel(), changeTime);
	Vector3F tempAngVel = pSphere->getAngularVelocity().interpolate(pSphere->getNewAngularVelocity(), changeTime);
	Matrix3F tempOrr = pSphere->getOrientation().interpolate(pSphere->getNewOrientation(), changeTime);

	if (pPoint.mCollisionType == CollisionType::PENETRATION)
	{
		tempPos = tempPos - pPoint.mContactNormal * pPoint.mCollisionDepth;
	}

	Matrix4F planeMat = pPlaneHoles->getMatrix();
	const auto center = pPlaneHoles->getPos() * planeMat;
	
	Matrix4F newPlaneMat = pPlaneHoles->getNewMatrix();
	auto newCenter = pPlaneHoles->getPos() * newPlaneMat;

	Vector3F tempPlaneVel = (newCenter - center) / changeTime;

	Vector3F sphereCenterToCollision = pPoint.mContactPoint1 - tempPos;
	Vector3F tempSphereVel = tempVel + tempAngVel.cross(sphereCenterToCollision);
	float relVel = pPoint.mContactNormal.dot(tempSphereVel - tempPlaneVel);

	Matrix3F sphereWorldTensor = tempOrr * pSphere->getInverseImpulseTenser() * tempOrr.transpose();

	float sphereInverseMass = 1.0f / pSphere->getMass();

	float sphereImpulseMag = pPoint.mContactNormal.dot((sphereWorldTensor * sphereCenterToCollision.cross(pPoint.mContactNormal)).cross(pPoint.mContactNormal));
	
	float j = -(1.0f + 0.8f) * relVel / (sphereInverseMass + sphereImpulseMag);
	
	tempVel = tempVel + (j * sphereInverseMass * pPoint.mContactNormal);
	tempAngVel = tempAngVel + sphereCenterToCollision.cross(j * pPoint.mContactNormal) * sphereWorldTensor;

	pSphere->setNewPos(tempPos);
	pSphere->setNewVel(tempVel);
	pSphere->setNewOrientation(tempOrr);
	pSphere->setNewAngularVel(tempAngVel);

	moved1 = true;
	moved2 = false;
}


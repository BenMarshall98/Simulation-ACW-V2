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
		const auto tempRig = pPoint.mContactId1;
		pPoint.mContactId2 = pPoint.mContactId1;
		pPoint.mContactId1 = tempRig;

		const auto tempVal = pPoint.mContactPoint1;
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
		const auto tempRig = pPoint.mContactId1;
		pPoint.mContactId2 = pPoint.mContactId1;
		pPoint.mContactId1 = tempRig;

		const auto tempVal = pPoint.mContactPoint1;
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
		const auto tempRig = pPoint.mContactId1;
		pPoint.mContactId2 = pPoint.mContactId1;
		pPoint.mContactId1 = tempRig;

		const auto tempVal = pPoint.mContactPoint1;
		pPoint.mContactPoint2 = pPoint.mContactPoint1;
		pPoint.mContactPoint1 = tempVal;
		respondCollisionSphereBowl(pPoint, rigidBody2, rigidBody1, moved2, moved1);
	}
	else if (rigidBody1->getObjectType() == ObjectType::SPHERE && rigidBody2->getObjectType() == ObjectType::CUBOID)
	{
		respondCollisionSphereCuboid(pPoint, rigidBody1, rigidBody2, moved1, moved2);
	}
	else if (rigidBody1->getObjectType() == ObjectType::CUBOID && rigidBody2->getObjectType() == ObjectType::SPHERE)
	{
		const auto tempRig = pPoint.mContactId1;
		pPoint.mContactId2 = pPoint.mContactId1;
		pPoint.mContactId1 = tempRig;

		const auto tempVal = pPoint.mContactPoint1;
		pPoint.mContactPoint2 = pPoint.mContactPoint1;
		pPoint.mContactPoint1 = tempVal;
		respondCollisionSphereCuboid(pPoint, rigidBody2, rigidBody1, moved2, moved1);
	}
	else if (rigidBody1->getObjectType() == ObjectType::CUBOID && rigidBody2->getObjectType() == ObjectType::CUBOID)
	{
		respondCollisionCuboidCuboid(pPoint, rigidBody1, rigidBody2, moved1, moved2);
	}
	else if (rigidBody1->getObjectType() == ObjectType::CUBOID && rigidBody2->getObjectType() == ObjectType::BOWL)
	{
		respondCollisionCuboidBowl(pPoint, rigidBody1, rigidBody2, moved1, moved2);
	}
	else if (rigidBody1->getObjectType() == ObjectType::BOWL && rigidBody2->getObjectType() == ObjectType::CUBOID)
	{
		const auto tempRig = pPoint.mContactId1;
		pPoint.mContactId2 = pPoint.mContactId1;
		pPoint.mContactId1 = tempRig;

		const auto tempVal = pPoint.mContactPoint1;
		pPoint.mContactPoint2 = pPoint.mContactPoint1;
		pPoint.mContactPoint1 = tempVal;
		respondCollisionCuboidBowl(pPoint, rigidBody2, rigidBody1, moved2, moved1);
	}
	else if (rigidBody1->getObjectType() == ObjectType::CUBOID && rigidBody2->getObjectType() == ObjectType::PLANE)
	{
		respondCollisionCuboidPlane(pPoint, rigidBody1, rigidBody2, moved1, moved2);
	}
	else if (rigidBody1->getObjectType() == ObjectType::PLANE && rigidBody2->getObjectType() == ObjectType::CUBOID)
	{
		const auto tempRig = pPoint.mContactId1;
		pPoint.mContactId2 = pPoint.mContactId1;
		pPoint.mContactId1 = tempRig;

		const auto tempVal = pPoint.mContactPoint1;
		pPoint.mContactPoint2 = pPoint.mContactPoint1;
		pPoint.mContactPoint1 = tempVal;
		respondCollisionCuboidPlane(pPoint, rigidBody2, rigidBody1, moved2, moved1);
	}
	else if (rigidBody1->getObjectType() == ObjectType::CUBOID && rigidBody2->getObjectType() == ObjectType::PLANEHOLES)
	{
		respondCollisionCuboidPlaneHoles(pPoint, rigidBody1, rigidBody2, moved1, moved2);
	}
	else if (rigidBody1->getObjectType() == ObjectType::PLANEHOLES && rigidBody2->getObjectType() == ObjectType::CUBOID)
	{
		const auto tempRig = pPoint.mContactId1;
		pPoint.mContactId2 = pPoint.mContactId1;
		pPoint.mContactId1 = tempRig;

		const auto tempVal = pPoint.mContactPoint1;
		pPoint.mContactPoint2 = pPoint.mContactPoint1;
		pPoint.mContactPoint1 = tempVal;
		respondCollisionCuboidPlane(pPoint, rigidBody2, rigidBody1, moved2, moved1);
	}
}

void CollisionResponse::respondCollisionSphereSphere(ManifoldPoint& pPoint, RigidBody * pSphere1, RigidBody * pSphere2, bool & moved1, bool & moved2)
{
	const auto changeTime1 = pPoint.mTime - pSphere1->getCurrentUpdateTime();

	auto tempPos1 = pSphere1->getPos().interpolate(pSphere1->getNewPos(), changeTime1);
	auto tempVel1 = pSphere1->getVel().interpolate(pSphere1->getNewVel(), changeTime1);
	auto tempAngVel1 = pSphere1->getAngularVelocity().interpolate(pSphere1->getNewAngularVelocity(), changeTime1);
	const auto tempOrr1 = slerp(pSphere1->getOrientation(), pSphere1->getNewOrientation(), changeTime1);
	auto rot = toMat3(tempOrr1);
	const auto tempOrrMat1 = Matrix3F(rot[0][0], rot[0][1], rot[0][2],
		rot[1][0], rot[1][1], rot[1][2],
		rot[2][0], rot[2][1], rot[2][2]);

	const auto changeTime2 = pPoint.mTime - pSphere2->getCurrentUpdateTime();

	auto tempPos2 = pSphere2->getPos().interpolate(pSphere2->getNewPos(), changeTime2);
	auto tempVel2 = pSphere2->getVel().interpolate(pSphere2->getNewVel(), changeTime2);
	auto tempAngVel2 = pSphere2->getAngularVelocity().interpolate(pSphere2->getNewAngularVelocity(), changeTime2);
	const auto tempOrr2 = slerp(pSphere2->getOrientation(), pSphere2->getNewOrientation(), changeTime2);
	rot = toMat3(tempOrr1);
	const auto tempOrrMat2 = Matrix3F(rot[0][0], rot[0][1], rot[0][2],
		rot[1][0], rot[1][1], rot[1][2],
		rot[2][0], rot[2][1], rot[2][2]);

	if (pPoint.mCollisionType == CollisionType::PENETRATION)
	{
		tempPos1 = tempPos1 + pPoint.mContactNormal * 0.5f * pPoint.mCollisionDepth;
		tempPos2 = tempPos2 - pPoint.mContactNormal * 0.5f * pPoint.mCollisionDepth;
	}

	const auto sphereCenterToCollision1 = pPoint.mContactPoint1 - tempPos1;
	const auto sphereCenterToCollision2 = pPoint.mContactPoint2 - tempPos2;

	const auto tempSphereVel1 = tempVel1 + tempAngVel1.cross(sphereCenterToCollision1);
	const auto tempSphereVel2 = tempVel2 + tempAngVel2.cross(sphereCenterToCollision2);

	const auto relVel = pPoint.mContactNormal.dot(tempSphereVel1 - tempSphereVel2);

	const auto sphereWorldTensor1 = tempOrrMat1 * pSphere1->getInverseImpulseTenser() * tempOrrMat1.transpose();
	const auto sphereWorldTensor2 = tempOrrMat2 * pSphere2->getInverseImpulseTenser() * tempOrrMat2.transpose();

	const auto sphereInverseMass1 = 1.0f / pSphere1->getMass();
	const auto sphereInverseMass2 = 1.0f / pSphere2->getMass();

	const auto sphereImpulseMag1 = pPoint.mContactNormal.dot((sphereWorldTensor1 * sphereCenterToCollision1.cross(pPoint.mContactNormal)).cross(pPoint.mContactNormal));
	const auto sphereImpulseMag2 = pPoint.mContactNormal.dot((sphereWorldTensor2 * sphereCenterToCollision2.cross(pPoint.mContactNormal)).cross(pPoint.mContactNormal));

	const auto j = -(1.0f + 0.8f) * relVel / (sphereInverseMass1 + sphereInverseMass2 + sphereImpulseMag1 + sphereImpulseMag2);

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
	const auto changeTime = pPoint.mTime - pSphere->getCurrentUpdateTime();

	auto tempPos = pSphere->getPos().interpolate(pSphere->getNewPos(), changeTime);
	auto tempVel = pSphere->getVel().interpolate(pSphere->getNewVel(), changeTime);
	auto tempAngVel = pSphere->getAngularVelocity().interpolate(pSphere->getNewAngularVelocity(), changeTime);
	const auto tempOrr = slerp(pSphere->getOrientation(), pSphere->getNewOrientation(), changeTime);
	auto rot = glm::toMat3(tempOrr);
	const auto tempOrrMat = Matrix3F(rot[0][0], rot[0][1], rot[0][2],
		rot[1][0], rot[1][1], rot[1][2],
		rot[2][0], rot[2][1], rot[2][2]);
	
	if (pPoint.mCollisionType == CollisionType::PENETRATION)
	{
		tempPos = tempPos - pPoint.mContactNormal * pPoint.mCollisionDepth;
	}

	const auto bowlMat = pBowl->getMatrix();
	const auto center = pBowl->getPos() * bowlMat;

	const auto tempBowlVel = Vector3F(0.0f, 0.0f, 0.0f);

	const auto sphereCenterToCollision = pPoint.mContactPoint1 - tempPos;
	const auto tempSphereVel = tempVel + tempAngVel.cross(sphereCenterToCollision);
	const auto relVel = pPoint.mContactNormal.dot(tempSphereVel - tempBowlVel);

	const auto sphereWorldTensor = tempOrrMat * pSphere->getInverseImpulseTenser() * tempOrrMat.transpose();

	const auto sphereInverseMass = 1.0f / pSphere->getMass();

	const auto sphereImpulseMag = pPoint.mContactNormal.dot((sphereWorldTensor * sphereCenterToCollision.cross(pPoint.mContactNormal)).cross(pPoint.mContactNormal));

	const auto j = -(1.0f + 0.8f) * relVel / (sphereInverseMass + sphereImpulseMag);

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
	const auto changeTime = pPoint.mTime - pSphere->getCurrentUpdateTime();

	auto tempPos = pSphere->getPos().interpolate(pSphere->getNewPos(), changeTime);
	auto tempVel = pSphere->getVel().interpolate(pSphere->getNewVel(), changeTime);
	auto tempAngVel = pSphere->getAngularVelocity().interpolate(pSphere->getNewAngularVelocity(), changeTime);
	const auto tempOrr = slerp(pSphere->getOrientation(), pSphere->getNewOrientation(), changeTime);
	auto rot = toMat3(tempOrr);
	const auto tempOrrMat = Matrix3F(rot[0][0], rot[0][1], rot[0][2],
		rot[1][0], rot[1][1], rot[1][2],
		rot[2][0], rot[2][1], rot[2][2]);
	
	if (pPoint.mCollisionType == CollisionType::PENETRATION)
	{
		tempPos = tempPos + pPoint.mContactNormal * pPoint.mCollisionDepth;
	}

	const auto planeMat = pPlane->getMatrix();
	const auto center = pPlane->getPos() * planeMat;

	const auto newPlaneMat = pPlane->getNewMatrix();
	const auto newCenter = pPlane->getPos() * newPlaneMat;

	Vector3F tempPlaneVel;
	
	if (changeTime > 0.0f)
	{
		tempPlaneVel = (newCenter - center) / changeTime;
	}
	else
	{
		tempPlaneVel = Vector3F(0.0f, 0.0f, 0.0f);
	}

	const auto sphereCenterToCollision = pPoint.mContactPoint1 - tempPos;
	const auto tempSphereVel = tempVel + tempAngVel.cross(sphereCenterToCollision);
	const auto relVel = pPoint.mContactNormal.dot(tempSphereVel - tempPlaneVel);

	const auto sphereWorldTensor = tempOrrMat * pSphere->getInverseImpulseTenser() * tempOrrMat.transpose();

	const auto sphereInverseMass = 1.0f / pSphere->getMass();

	const auto sphereImpulseMag = pPoint.mContactNormal.dot((sphereWorldTensor * sphereCenterToCollision.cross(pPoint.mContactNormal)).cross(pPoint.mContactNormal));

	const auto j = -(1.0f + 0.8f) * relVel / (sphereInverseMass + sphereImpulseMag);
	
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
	const auto changeTime = pPoint.mTime - pSphere->getCurrentUpdateTime();

	auto tempPos = pSphere->getPos().interpolate(pSphere->getNewPos(), changeTime);
	auto tempVel = pSphere->getVel().interpolate(pSphere->getNewVel(), changeTime);
	auto tempAngVel = pSphere->getAngularVelocity().interpolate(pSphere->getNewAngularVelocity(), changeTime);
	const auto tempOrr = glm::slerp(pSphere->getOrientation(), pSphere->getNewOrientation(), changeTime);
	auto rot = glm::toMat3(tempOrr);
	const auto tempOrrMat = Matrix3F(rot[0][0], rot[0][1], rot[0][2],
		rot[1][0], rot[1][1], rot[1][2],
		rot[2][0], rot[2][1], rot[2][2]);
	
	if (pPoint.mCollisionType == CollisionType::PENETRATION)
	{
		tempPos = tempPos - pPoint.mContactNormal * pPoint.mCollisionDepth;
	}

	const auto planeMat = pPlaneHoles->getMatrix();
	const auto center = pPlaneHoles->getPos() * planeMat;

	const auto newPlaneMat = pPlaneHoles->getNewMatrix();
	const auto newCenter = pPlaneHoles->getPos() * newPlaneMat;

	const auto tempPlaneVel = (newCenter - center) / changeTime;

	const auto sphereCenterToCollision = pPoint.mContactPoint1 - tempPos;
	const auto tempSphereVel = tempVel + tempAngVel.cross(sphereCenterToCollision);
	const auto relVel = pPoint.mContactNormal.dot(tempSphereVel - tempPlaneVel);

	const auto sphereWorldTensor = tempOrrMat * pSphere->getInverseImpulseTenser() * tempOrrMat.transpose();

	const auto sphereInverseMass = 1.0f / pSphere->getMass();

	const auto sphereImpulseMag = pPoint.mContactNormal.dot((sphereWorldTensor * sphereCenterToCollision.cross(pPoint.mContactNormal)).cross(pPoint.mContactNormal));
	
	const auto j = -(1.0f + 0.8f) * relVel / (sphereInverseMass + sphereImpulseMag);
	
	tempVel = tempVel + (j * sphereInverseMass * pPoint.mContactNormal);
	tempAngVel = tempAngVel + sphereCenterToCollision.cross(j * pPoint.mContactNormal) * sphereWorldTensor;

	pSphere->setNewPos(tempPos);
	pSphere->setNewVel(tempVel);
	pSphere->setNewOrientation(tempOrr);
	pSphere->setNewAngularVel(tempAngVel);

	moved1 = true;
	moved2 = false;
}

void CollisionResponse::respondCollisionSphereCuboid(ManifoldPoint& pPoint, RigidBody * pSphere, RigidBody * pCuboid, bool & moved1, bool & moved2)
{
	const auto changeTime1 = pPoint.mTime - pSphere->getCurrentUpdateTime();

	auto tempPos1 = pSphere->getPos().interpolate(pSphere->getNewPos(), changeTime1);
	auto tempVel1 = pSphere->getVel().interpolate(pSphere->getNewVel(), changeTime1);
	auto tempAngVel1 = pSphere->getAngularVelocity().interpolate(pSphere->getNewAngularVelocity(), changeTime1);
	const auto tempOrr1 = glm::slerp(pSphere->getOrientation(), pSphere->getNewOrientation(), changeTime1);
	auto rot = glm::toMat3(tempOrr1);
	const auto tempOrrMat1 = Matrix3F(rot[0][0], rot[0][1], rot[0][2],
		rot[1][0], rot[1][1], rot[1][2],
		rot[2][0], rot[2][1], rot[2][2]);

	const auto changeTime2 = pPoint.mTime - pCuboid->getCurrentUpdateTime();

	auto tempPos2 = pCuboid->getPos().interpolate(pCuboid->getNewPos(), changeTime2);
	auto tempVel2 = pCuboid->getVel().interpolate(pCuboid->getNewVel(), changeTime2);
	auto tempAngVel2 = pCuboid->getAngularVelocity().interpolate(pCuboid->getNewAngularVelocity(), changeTime2);
	const auto tempOrr2 = glm::slerp(pCuboid->getOrientation(), pCuboid->getNewOrientation(), changeTime2);
	rot = glm::toMat3(tempOrr1);
	const auto tempOrrMat2 = Matrix3F(rot[0][0], rot[0][1], rot[0][2],
		rot[1][0], rot[1][1], rot[1][2],
		rot[2][0], rot[2][1], rot[2][2]);
	
	if (pPoint.mCollisionType == CollisionType::PENETRATION)
	{
		tempPos1 = tempPos1 + pPoint.mContactNormal * 0.5f * pPoint.mCollisionDepth;
		tempPos2 = tempPos2 - pPoint.mContactNormal * 0.5f * pPoint.mCollisionDepth;
	}

	const auto sphereCenterToCollision = pPoint.mContactPoint1 - tempPos1;
	const auto cuboidCenterToCollision = pPoint.mContactPoint2 - tempPos2;

	const auto tempSphereVel = tempVel1 + tempAngVel1.cross(sphereCenterToCollision);
	const auto tempCuboidVel = tempVel2 + tempAngVel2.cross(cuboidCenterToCollision);

	const auto relVel = pPoint.mContactNormal.dot(tempSphereVel - tempCuboidVel);

	const auto sphereWorldTensor = tempOrrMat1 * pSphere->getInverseImpulseTenser() * tempOrrMat1.transpose();
	const auto cuboidWorldTensor = tempOrrMat2 * pSphere->getInverseImpulseTenser() * tempOrrMat2.transpose();

	const auto sphereInverseMass = 1.0f / pSphere->getMass();
	const auto cuboidInverseMass = 1.0f / pCuboid->getMass();

	const auto sphereImpulseMag = pPoint.mContactNormal.dot((sphereWorldTensor * sphereCenterToCollision.cross(pPoint.mContactNormal)).cross(pPoint.mContactNormal));
	const auto cuboidImpulseMag = pPoint.mContactNormal.dot((cuboidWorldTensor * cuboidCenterToCollision.cross(pPoint.mContactNormal)).cross(pPoint.mContactNormal));

	const auto j = -(1.0f + 0.8f) * relVel / (sphereInverseMass + cuboidInverseMass + sphereImpulseMag + cuboidImpulseMag);

	tempVel1 = tempVel1 + (j * sphereInverseMass * pPoint.mContactNormal);
	tempVel2 = tempVel2 - (j * cuboidInverseMass * pPoint.mContactNormal);
	tempAngVel1 = tempAngVel1 + sphereCenterToCollision.cross(j * pPoint.mContactNormal) * sphereWorldTensor;
	tempAngVel2 = tempAngVel2 - cuboidCenterToCollision.cross(j * pPoint.mContactNormal) * cuboidWorldTensor;

	pSphere->setNewPos(tempPos1);
	pSphere->setNewVel(tempVel1);
	pSphere->setNewAngularVel(tempAngVel1);
	pSphere->setNewOrientation(tempOrr1);

	pCuboid->setNewPos(tempPos2);
	pCuboid->setNewVel(tempVel2);
	pCuboid->setNewAngularVel(tempAngVel2);
	pCuboid->setNewOrientation(tempOrr2);

	moved1 = true;
	moved2 = true;
}

void CollisionResponse::respondCollisionCuboidCuboid(ManifoldPoint & pPoint, RigidBody * pCuboid1, RigidBody * pCuboid2, bool & moved1, bool & moved2)
{
	//TODO: Implement
}

void CollisionResponse::respondCollisionCuboidBowl(ManifoldPoint & pPoint, RigidBody * pCuboid, RigidBody * pBowl, bool & moved1, bool & moved2)
{
	//TODO: Implement
}

void CollisionResponse::respondCollisionCuboidPlane(ManifoldPoint & pPoint, RigidBody * pCuboid, RigidBody * pPlane, bool & moved1, bool & moved2)
{
	//TODO: Implement
}

void CollisionResponse::respondCollisionCuboidPlaneHoles(ManifoldPoint & pPoint, RigidBody * pCuboid, RigidBody * pPlaneHoles, bool & moved1, bool & moved2)
{
	//TODO: Implement
}
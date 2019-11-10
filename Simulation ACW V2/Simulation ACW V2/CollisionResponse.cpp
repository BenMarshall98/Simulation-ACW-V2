#include "CollisionResponse.h"
#include "Game.h"

//https://www.scss.tcd.ie/~manzkem/CS7057/cs7057-1516-09-CollisionResponse-mm.pdf
//http://citeseerx.ist.psu.edu/viewdoc/download?doi=10.1.1.130.6905&rep=rep1&type=pdf

void CollisionResponse::dynamicCollisionResponse(ManifoldPoint& pPoint, bool & pMoved1, bool & pMoved2)
{
	auto * rigidBody1 = pPoint.mContactId1;
	auto * rigidBody2 = pPoint.mContactId2;

	if (rigidBody1->getObjectType() == ObjectType::SPHERE && rigidBody2->getObjectType() == ObjectType::SPHERE)
	{
		respondCollisionSphereSphere(pPoint, rigidBody1, rigidBody2, pMoved1, pMoved1);
	}
	else if (rigidBody1->getObjectType() == ObjectType::SPHERE && rigidBody2->getObjectType() == ObjectType::PLANE)
	{
		respondCollisionSpherePlane(pPoint, rigidBody1, rigidBody2, pMoved1, pMoved2);
	}
	else if (rigidBody1->getObjectType() == ObjectType::PLANE && rigidBody2->getObjectType() == ObjectType::SPHERE)
	{
		const auto tempRig = pPoint.mContactId1;
		pPoint.mContactId2 = pPoint.mContactId1;
		pPoint.mContactId1 = tempRig;

		const auto tempVal = pPoint.mContactPoint1;
		pPoint.mContactPoint2 = pPoint.mContactPoint1;
		pPoint.mContactPoint1 = tempVal;
		
		respondCollisionSpherePlane(pPoint, rigidBody2, rigidBody1, pMoved2, pMoved1);
	}
	else if (rigidBody1->getObjectType() == ObjectType::SPHERE && rigidBody2->getObjectType() == ObjectType::PLANEHOLES)
	{
		respondCollisionSpherePlaneHoles(pPoint, rigidBody1, rigidBody2, pMoved1, pMoved2);
	}
	else if (rigidBody1->getObjectType() == ObjectType::PLANEHOLES && rigidBody2->getObjectType() == ObjectType::SPHERE)
	{
		const auto tempRig = pPoint.mContactId1;
		pPoint.mContactId2 = pPoint.mContactId1;
		pPoint.mContactId1 = tempRig;

		const auto tempVal = pPoint.mContactPoint1;
		pPoint.mContactPoint2 = pPoint.mContactPoint1;
		pPoint.mContactPoint1 = tempVal;
		
		respondCollisionSpherePlaneHoles(pPoint, rigidBody2, rigidBody1, pMoved2, pMoved1);
	}
	else if (rigidBody1->getObjectType() == ObjectType::SPHERE && rigidBody2->getObjectType() == ObjectType::BOWL)
	{
		respondCollisionSphereBowl(pPoint, rigidBody1, rigidBody2, pMoved1, pMoved2);
	}
	else if (rigidBody1->getObjectType() == ObjectType::BOWL && rigidBody2->getObjectType() == ObjectType::SPHERE)
	{
		const auto tempRig = pPoint.mContactId1;
		pPoint.mContactId2 = pPoint.mContactId1;
		pPoint.mContactId1 = tempRig;

		const auto tempVal = pPoint.mContactPoint1;
		pPoint.mContactPoint2 = pPoint.mContactPoint1;
		pPoint.mContactPoint1 = tempVal;
		respondCollisionSphereBowl(pPoint, rigidBody2, rigidBody1, pMoved2, pMoved1);
	}
	else if (rigidBody1->getObjectType() == ObjectType::SPHERE && rigidBody2->getObjectType() == ObjectType::CUBOID)
	{
		respondCollisionSphereCuboid(pPoint, rigidBody1, rigidBody2, pMoved1, pMoved2);
	}
	else if (rigidBody1->getObjectType() == ObjectType::CUBOID && rigidBody2->getObjectType() == ObjectType::SPHERE)
	{
		const auto tempRig = pPoint.mContactId1;
		pPoint.mContactId2 = pPoint.mContactId1;
		pPoint.mContactId1 = tempRig;

		const auto tempVal = pPoint.mContactPoint1;
		pPoint.mContactPoint2 = pPoint.mContactPoint1;
		pPoint.mContactPoint1 = tempVal;
		respondCollisionSphereCuboid(pPoint, rigidBody2, rigidBody1, pMoved2, pMoved1);
	}
	else if (rigidBody1->getObjectType() == ObjectType::CUBOID && rigidBody2->getObjectType() == ObjectType::CUBOID)
	{
		respondCollisionCuboidCuboid(pPoint, rigidBody1, rigidBody2, pMoved1, pMoved2);
	}
	else if (rigidBody1->getObjectType() == ObjectType::CUBOID && rigidBody2->getObjectType() == ObjectType::BOWL)
	{
		respondCollisionCuboidBowl(pPoint, rigidBody1, rigidBody2, pMoved1, pMoved2);
	}
	else if (rigidBody1->getObjectType() == ObjectType::BOWL && rigidBody2->getObjectType() == ObjectType::CUBOID)
	{
		const auto tempRig = pPoint.mContactId1;
		pPoint.mContactId2 = pPoint.mContactId1;
		pPoint.mContactId1 = tempRig;

		const auto tempVal = pPoint.mContactPoint1;
		pPoint.mContactPoint2 = pPoint.mContactPoint1;
		pPoint.mContactPoint1 = tempVal;
		respondCollisionCuboidBowl(pPoint, rigidBody2, rigidBody1, pMoved2, pMoved1);
	}
	else if (rigidBody1->getObjectType() == ObjectType::CUBOID && rigidBody2->getObjectType() == ObjectType::PLANE)
	{
		respondCollisionCuboidPlane(pPoint, rigidBody1, rigidBody2, pMoved1, pMoved2);
	}
	else if (rigidBody1->getObjectType() == ObjectType::PLANE && rigidBody2->getObjectType() == ObjectType::CUBOID)
	{
		const auto tempRig = pPoint.mContactId1;
		pPoint.mContactId2 = pPoint.mContactId1;
		pPoint.mContactId1 = tempRig;

		const auto tempVal = pPoint.mContactPoint1;
		pPoint.mContactPoint2 = pPoint.mContactPoint1;
		pPoint.mContactPoint1 = tempVal;
		respondCollisionCuboidPlane(pPoint, rigidBody2, rigidBody1, pMoved2, pMoved1);
	}
	else if (rigidBody1->getObjectType() == ObjectType::CUBOID && rigidBody2->getObjectType() == ObjectType::PLANEHOLES)
	{
		respondCollisionCuboidPlaneHoles(pPoint, rigidBody1, rigidBody2, pMoved1, pMoved2);
	}
	else if (rigidBody1->getObjectType() == ObjectType::PLANEHOLES && rigidBody2->getObjectType() == ObjectType::CUBOID)
	{
		const auto tempRig = pPoint.mContactId1;
		pPoint.mContactId2 = pPoint.mContactId1;
		pPoint.mContactId1 = tempRig;

		const auto tempVal = pPoint.mContactPoint1;
		pPoint.mContactPoint2 = pPoint.mContactPoint1;
		pPoint.mContactPoint1 = tempVal;
		respondCollisionCuboidPlane(pPoint, rigidBody2, rigidBody1, pMoved2, pMoved1);
	}
}

void CollisionResponse::respondCollisionSphereSphere(ManifoldPoint& pPoint, RigidBody * pSphere1, RigidBody * pSphere2, bool & pMoved1, bool & pMoved2)
{
	const auto changeTime1 = pPoint.mTime - pSphere1->getCurrentUpdateTime();

	auto tempPos1 = mix(pSphere1->getPos(), pSphere1->getNewPos(), changeTime1);
	auto tempVel1 = mix(pSphere1->getVel(), pSphere1->getNewVel(), changeTime1);
	auto tempAngVel1 = mix(pSphere1->getAngularVelocity(), pSphere1->getNewAngularVelocity(), changeTime1);
	const auto tempOrr1 = slerp(pSphere1->getOrientation(), pSphere1->getNewOrientation(), changeTime1);
	const auto tempOrrMat1 = toMat3(tempOrr1);

	const auto changeTime2 = pPoint.mTime - pSphere2->getCurrentUpdateTime();

	auto tempPos2 = mix(pSphere2->getPos(), pSphere2->getNewPos(), changeTime2);
	auto tempVel2 = mix(pSphere2->getVel(), pSphere2->getNewVel(), changeTime2);
	auto tempAngVel2 = mix(pSphere2->getAngularVelocity(), pSphere2->getNewAngularVelocity(), changeTime2);
	const auto tempOrr2 = slerp(pSphere2->getOrientation(), pSphere2->getNewOrientation(), changeTime2);
	const auto tempOrrMat2 = toMat3(tempOrr1);

	if (pPoint.mCollisionType == CollisionType::PENETRATION)
	{
		tempPos1 = tempPos1 + pPoint.mContactNormal * 0.5f * pPoint.mCollisionDepth;
		tempPos2 = tempPos2 - pPoint.mContactNormal * 0.5f * pPoint.mCollisionDepth;
	}

	const auto sphereCenterToCollision1 = pPoint.mContactPoint1 - tempPos1;
	const auto sphereCenterToCollision2 = pPoint.mContactPoint2 - tempPos2;

	const auto tempSphereVel1 = tempVel1 + cross(tempAngVel1, sphereCenterToCollision1);
	const auto tempSphereVel2 = tempVel2 + cross(tempAngVel2, sphereCenterToCollision2);

	const auto relVel = dot(pPoint.mContactNormal, tempSphereVel1 - tempSphereVel2);

	const auto sphereWorldTensor1 = tempOrrMat1 * pSphere1->getInverseImpulseTenser() * transpose(tempOrrMat1);
	const auto sphereWorldTensor2 = tempOrrMat2 * pSphere2->getInverseImpulseTenser() * transpose(tempOrrMat2);

	const auto sphereInverseMass1 = 1.0f / pSphere1->getMass();
	const auto sphereInverseMass2 = 1.0f / pSphere2->getMass();

	const auto sphereImpulseMag1 = dot(pPoint.mContactNormal, cross(sphereWorldTensor1 * cross(sphereCenterToCollision1, pPoint.mContactNormal), pPoint.mContactNormal));
	const auto sphereImpulseMag2 = dot(pPoint.mContactNormal, cross(sphereWorldTensor2 * cross(sphereCenterToCollision2, pPoint.mContactNormal), pPoint.mContactNormal));

	const auto j = -(1.0f + 0.8f) * relVel / (sphereInverseMass1 + sphereInverseMass2 + sphereImpulseMag1 + sphereImpulseMag2);

	tempVel1 = tempVel1 + j * sphereInverseMass1 * pPoint.mContactNormal;
	tempVel2 = tempVel2 - j * sphereInverseMass2 * pPoint.mContactNormal;
	tempAngVel1 = tempAngVel1 + cross(sphereCenterToCollision1, j * pPoint.mContactNormal) * sphereWorldTensor1;
	tempAngVel2 = tempAngVel2 - cross(sphereCenterToCollision2, j * pPoint.mContactNormal) * sphereWorldTensor2;

	pSphere1->setNewPos(tempPos1);
	pSphere1->setNewVel(tempVel1);
	pSphere1->setNewAngularVel(tempAngVel1);
	pSphere1->setNewOrientation(tempOrr1);
						
	pSphere2->setNewPos(tempPos2);
	pSphere2->setNewVel(tempVel2);
	pSphere2->setNewAngularVel(tempAngVel2);
	pSphere2->setNewOrientation(tempOrr2);

	pMoved1 = true;
	pMoved2 = true;
}

void CollisionResponse::respondCollisionSphereBowl(ManifoldPoint& pPoint, RigidBody* pSphere, RigidBody *, bool& pMoved1, bool& pMoved2)
{
	const auto changeTime = pPoint.mTime - pSphere->getCurrentUpdateTime();

	auto tempPos = mix(pSphere->getPos(), pSphere->getNewPos(), changeTime);
	auto tempVel = mix(pSphere->getVel(), pSphere->getNewVel(), changeTime);
	auto tempAngVel = mix(pSphere->getAngularVelocity(), pSphere->getNewAngularVelocity(), changeTime);
	const auto tempOrr = slerp(pSphere->getOrientation(), pSphere->getNewOrientation(), changeTime);
	const auto tempOrrMat = toMat3(tempOrr);
	
	if (pPoint.mCollisionType == CollisionType::PENETRATION)
	{
		tempPos = tempPos - pPoint.mContactNormal * pPoint.mCollisionDepth;
	}

	const auto tempBowlVel = glm::vec3(0.0f, 0.0f, 0.0f);

	const auto sphereCenterToCollision = pPoint.mContactPoint1 - tempPos;
	const auto tempSphereVel = tempVel + cross(tempAngVel, sphereCenterToCollision);
	const auto relVel = dot(pPoint.mContactNormal, tempSphereVel - tempBowlVel);

	const auto sphereWorldTensor = tempOrrMat * pSphere->getInverseImpulseTenser() * transpose(tempOrrMat);

	const auto sphereInverseMass = 1.0f / pSphere->getMass();

	const auto sphereImpulseMag = dot(pPoint.mContactNormal, cross(sphereWorldTensor * cross(sphereCenterToCollision, pPoint.mContactNormal), pPoint.mContactNormal));

	const auto j = -(1.0f + 0.8f) * relVel / (sphereInverseMass + sphereImpulseMag);

	tempVel = tempVel + j * sphereInverseMass * pPoint.mContactNormal;
	tempAngVel = tempAngVel + cross(sphereCenterToCollision, j * pPoint.mContactNormal) * sphereWorldTensor;

	pSphere->setNewPos(tempPos);
	pSphere->setNewVel(tempVel);
	pSphere->setNewOrientation(tempOrr);
	pSphere->setNewAngularVel(tempAngVel);

	pMoved1 = true;
	pMoved2 = false;
}

void CollisionResponse::respondCollisionSpherePlane(ManifoldPoint& pPoint, RigidBody * pSphere, RigidBody * pPlane, bool & pMoved1, bool & pMoved2)
{
	const auto changeTime = pPoint.mTime - pSphere->getCurrentUpdateTime();

	auto tempPos = mix(pSphere->getPos(), pSphere->getNewPos(), changeTime);
	auto tempVel = mix(pSphere->getVel(), pSphere->getNewVel(), changeTime);
	auto tempAngVel = mix(pSphere->getAngularVelocity(), pSphere->getNewAngularVelocity(), changeTime);
	const auto tempOrr = slerp(pSphere->getOrientation(), pSphere->getNewOrientation(), changeTime);
	const auto tempOrrMat = toMat3(tempOrr);
	
	if (pPoint.mCollisionType == CollisionType::PENETRATION)
	{
		tempPos = tempPos + pPoint.mContactNormal * pPoint.mCollisionDepth;
	}

	const auto planeMat = pPlane->getMatrix();
	const auto center = pPlane->getPos() * glm::mat3(planeMat);

	const auto newPlaneMat = pPlane->getNewMatrix();
	const auto newCenter = pPlane->getPos() * glm::mat3(newPlaneMat);

	glm::vec3 tempPlaneVel;
	
	if (changeTime > 0.0f)
	{
		tempPlaneVel = (newCenter - center) / changeTime;
	}
	else
	{
		tempPlaneVel = glm::vec3(0.0f, 0.0f, 0.0f);
	}

	const auto sphereCenterToCollision = pPoint.mContactPoint1 - tempPos;
	const auto tempSphereVel = tempVel + cross(tempAngVel, sphereCenterToCollision);
	const auto relVel = dot(pPoint.mContactNormal, tempSphereVel - tempPlaneVel);

	const auto sphereWorldTensor = tempOrrMat * pSphere->getInverseImpulseTenser() * transpose(tempOrrMat);

	const auto sphereInverseMass = 1.0f / pSphere->getMass();

	const auto sphereImpulseMag = dot(pPoint.mContactNormal, cross(sphereWorldTensor * cross(sphereCenterToCollision, pPoint.mContactNormal), pPoint.mContactNormal));

	const auto j = -(1.0f + 0.8f) * relVel / (sphereInverseMass + sphereImpulseMag);
	
	tempVel = tempVel + j * sphereInverseMass * pPoint.mContactNormal;
	tempAngVel = tempAngVel + cross(sphereCenterToCollision, j * pPoint.mContactNormal) * sphereWorldTensor;

	pSphere->setNewPos(tempPos);
	pSphere->setNewVel(tempVel);
	pSphere->setNewOrientation(tempOrr);
	pSphere->setNewAngularVel(tempAngVel);

	pMoved1 = true;
	pMoved2 = false;
}

auto CollisionResponse::respondCollisionSpherePlaneHoles(ManifoldPoint& pPoint, RigidBody* pSphere,
                                                         RigidBody* pPlaneHoles, bool& pMoved1, bool& pMoved2) -> void
{
	const auto changeTime = pPoint.mTime - pSphere->getCurrentUpdateTime();

	auto tempPos = mix(pSphere->getPos(), pSphere->getNewPos(), changeTime);
	auto tempVel = mix(pSphere->getVel(), pSphere->getNewVel(), changeTime);
	auto tempAngVel = mix(pSphere->getAngularVelocity(), pSphere->getNewAngularVelocity(), changeTime);
	const auto tempOrr = slerp(pSphere->getOrientation(), pSphere->getNewOrientation(), changeTime);
	const auto tempOrrMat = toMat3(tempOrr);
	
	if (pPoint.mCollisionType == CollisionType::PENETRATION)
	{
		tempPos = tempPos - pPoint.mContactNormal * pPoint.mCollisionDepth;
	}

	const auto planeMat = pPlaneHoles->getMatrix();
	const auto center = pPlaneHoles->getPos() * glm::mat3(planeMat);

	const auto newPlaneMat = pPlaneHoles->getNewMatrix();
	const auto newCenter = pPlaneHoles->getPos() * glm::mat3(newPlaneMat);

	const auto tempPlaneVel = (newCenter - center) / changeTime;

	const auto sphereCenterToCollision = pPoint.mContactPoint1 - tempPos;
	const auto tempSphereVel = tempVel + cross(tempAngVel, sphereCenterToCollision);
	const auto relVel = dot(pPoint.mContactNormal, tempSphereVel - tempPlaneVel);

	const auto sphereWorldTensor = tempOrrMat * pSphere->getInverseImpulseTenser() * transpose(tempOrrMat);

	const auto sphereInverseMass = 1.0f / pSphere->getMass();

	const auto sphereImpulseMag = dot(pPoint.mContactNormal, cross(sphereWorldTensor * cross(sphereCenterToCollision, pPoint.mContactNormal), pPoint.mContactNormal));
	
	const auto j = -(1.0f + 0.8f) * relVel / (sphereInverseMass + sphereImpulseMag);
	
	tempVel = tempVel + j * sphereInverseMass * pPoint.mContactNormal;
	tempAngVel = tempAngVel + cross(sphereCenterToCollision, j * pPoint.mContactNormal) * sphereWorldTensor;

	pSphere->setNewPos(tempPos);
	pSphere->setNewVel(tempVel);
	pSphere->setNewOrientation(tempOrr);
	pSphere->setNewAngularVel(tempAngVel);

	pMoved1 = true;
	pMoved2 = false;
}

void CollisionResponse::respondCollisionSphereCuboid(ManifoldPoint& pPoint, RigidBody * pSphere, RigidBody * pCuboid, bool & pMoved1, bool & pMoved2)
{
	const auto changeTime1 = pPoint.mTime - pSphere->getCurrentUpdateTime();

	auto tempPos1 = mix(pSphere->getPos(), pSphere->getNewPos(), changeTime1);
	auto tempVel1 = mix(pSphere->getVel(), pSphere->getNewVel(), changeTime1);
	auto tempAngVel1 = mix(pSphere->getAngularVelocity(), pSphere->getNewAngularVelocity(), changeTime1);
	const auto tempOrr1 = slerp(pSphere->getOrientation(), pSphere->getNewOrientation(), changeTime1);
	const auto tempOrrMat1 = toMat3(tempOrr1);

	const auto changeTime2 = pPoint.mTime - pCuboid->getCurrentUpdateTime();

	auto tempPos2 = mix(pCuboid->getPos(), pCuboid->getNewPos(), changeTime2);
	auto tempVel2 = mix(pCuboid->getVel(), pCuboid->getNewVel(), changeTime2);
	auto tempAngVel2 = mix(pCuboid->getAngularVelocity(), pCuboid->getNewAngularVelocity(), changeTime2);
	const auto tempOrr2 = slerp(pCuboid->getOrientation(), pCuboid->getNewOrientation(), changeTime2);
	const auto tempOrrMat2 = toMat3(tempOrr1);
	
	if (pPoint.mCollisionType == CollisionType::PENETRATION)
	{
		tempPos1 = tempPos1 + pPoint.mContactNormal * 0.5f * pPoint.mCollisionDepth;
		tempPos2 = tempPos2 - pPoint.mContactNormal * 0.5f * pPoint.mCollisionDepth;
	}

	const auto sphereCenterToCollision = pPoint.mContactPoint1 - tempPos1;
	const auto cuboidCenterToCollision = pPoint.mContactPoint2 - tempPos2;

	const auto tempSphereVel = tempVel1 + cross(tempAngVel1, sphereCenterToCollision);
	const auto tempCuboidVel = tempVel2 + cross(tempAngVel2, cuboidCenterToCollision);

	const auto relVel = dot(pPoint.mContactNormal, tempSphereVel - tempCuboidVel);

	const auto sphereWorldTensor = tempOrrMat1 * pSphere->getInverseImpulseTenser() * transpose(tempOrrMat1);
	const auto cuboidWorldTensor = tempOrrMat2 * pSphere->getInverseImpulseTenser() * transpose(tempOrrMat2);

	const auto sphereInverseMass = 1.0f / pSphere->getMass();
	const auto cuboidInverseMass = 1.0f / pCuboid->getMass();

	const auto sphereImpulseMag = dot(pPoint.mContactNormal, cross(sphereWorldTensor * cross(sphereCenterToCollision, pPoint.mContactNormal), pPoint.mContactNormal));
	const auto cuboidImpulseMag = dot(pPoint.mContactNormal, cross(cuboidWorldTensor * cross(cuboidCenterToCollision, pPoint.mContactNormal), pPoint.mContactNormal));

	const auto j = -(1.0f + 0.8f) * relVel / (sphereInverseMass + cuboidInverseMass + sphereImpulseMag + cuboidImpulseMag);

	tempVel1 = tempVel1 + j * sphereInverseMass * pPoint.mContactNormal;
	tempVel2 = tempVel2 - j * cuboidInverseMass * pPoint.mContactNormal;
	tempAngVel1 = tempAngVel1 + cross(sphereCenterToCollision, j * pPoint.mContactNormal) * sphereWorldTensor;
	tempAngVel2 = tempAngVel2 - cross(cuboidCenterToCollision, j * pPoint.mContactNormal) * cuboidWorldTensor;

	pSphere->setNewPos(tempPos1);
	pSphere->setNewVel(tempVel1);
	pSphere->setNewAngularVel(tempAngVel1);
	pSphere->setNewOrientation(tempOrr1);

	pCuboid->setNewPos(tempPos2);
	pCuboid->setNewVel(tempVel2);
	pCuboid->setNewAngularVel(tempAngVel2);
	pCuboid->setNewOrientation(tempOrr2);

	pMoved1 = true;
	pMoved2 = true;
}

void CollisionResponse::respondCollisionCuboidCuboid(ManifoldPoint &, RigidBody *, RigidBody *, bool &, bool &)
{
	//TODO: Implement
}

void CollisionResponse::respondCollisionCuboidBowl(ManifoldPoint &, RigidBody *, RigidBody *, bool &, bool &)
{
	//TODO: Implement
}

void CollisionResponse::respondCollisionCuboidPlane(ManifoldPoint &, RigidBody *, RigidBody *, bool &, bool &)
{
	//TODO: Implement
}

void CollisionResponse::respondCollisionCuboidPlaneHoles(ManifoldPoint &, RigidBody *, RigidBody *, bool &, bool &)
{
	//TODO: Implement
}
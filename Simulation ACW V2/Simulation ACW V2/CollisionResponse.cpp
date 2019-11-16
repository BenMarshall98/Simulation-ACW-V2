#include "CollisionResponse.h"
#include "Game.h"
#include <corecrt_math_defines.h>

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
	else if (rigidBody1->getObjectType() == ObjectType::SPHERE && rigidBody2->getObjectType() == ObjectType::CYLINDER)
	{
		respondCollisionSphereCylinder(pPoint, rigidBody1, rigidBody2, pMoved1, pMoved2);
	}
	else if (rigidBody1->getObjectType() == ObjectType::CYLINDER && rigidBody2->getObjectType() == ObjectType::SPHERE)
	{
		const auto tempRig = pPoint.mContactId1;
		pPoint.mContactId2 = pPoint.mContactId1;
		pPoint.mContactId1 = tempRig;

		const auto tempVal = pPoint.mContactPoint1;
		pPoint.mContactPoint2 = pPoint.mContactPoint1;
		pPoint.mContactPoint1 = tempVal;
		respondCollisionSphereCylinder(pPoint, rigidBody2, rigidBody1, pMoved2, pMoved1);
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
	else if (rigidBody1->getObjectType() == ObjectType::CUBOID && rigidBody2->getObjectType() == ObjectType::CYLINDER)
	{
	respondCollisionCuboidCylinder(pPoint, rigidBody1, rigidBody2, pMoved1, pMoved2);
	}
	else if (rigidBody1->getObjectType() == ObjectType::CYLINDER && rigidBody2->getObjectType() == ObjectType::CUBOID)
	{
		const auto tempRig = pPoint.mContactId1;
		pPoint.mContactId2 = pPoint.mContactId1;
		pPoint.mContactId1 = tempRig;

		const auto tempVal = pPoint.mContactPoint1;
		pPoint.mContactPoint2 = pPoint.mContactPoint1;
		pPoint.mContactPoint1 = tempVal;
		respondCollisionCuboidCylinder(pPoint, rigidBody2, rigidBody1, pMoved2, pMoved1);
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

	dynamicCollisionResponse(pPoint, tempPos1, tempVel1, tempAngVel1, tempOrrMat1,
		tempPos2, tempVel2, tempAngVel2, tempOrrMat2,
		pSphere1->getInverseImpulseTenser(), pSphere2->getInverseImpulseTenser(),
		1.0f / pSphere1->getMass(), 1.0f / pSphere2->getMass());
	
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
	
	staticCollisionResponse(pPoint, tempVel, tempAngVel, tempOrrMat, 
		tempBowlVel, sphereCenterToCollision, tempSphereVel,
		pSphere->getInverseImpulseTenser(), 1.0f / pSphere->getMass());

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
	const auto center = glm::vec3(planeMat * glm::vec4(pPlane->getPos(), 1.0f));

	const auto newPlaneMat = pPlane->getNewMatrix();
	const auto newCenter = glm::vec3(newPlaneMat * glm::vec4(pPlane->getPos(), 1.0f));

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

	staticCollisionResponse(pPoint, tempVel, tempAngVel, tempOrrMat,
		tempPlaneVel, sphereCenterToCollision, tempSphereVel,
		pSphere->getInverseImpulseTenser(), 1.0f / pSphere->getMass());
	
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
	const auto center = glm::vec3(planeMat * glm::vec4(pPlaneHoles->getPos(), 1.0f));

	const auto newPlaneMat = pPlaneHoles->getNewMatrix();
	const auto newCenter = glm::vec3(newPlaneMat * glm::vec4(pPlaneHoles->getPos(), 1.0f));

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

	staticCollisionResponse(pPoint, tempVel, tempAngVel, tempOrrMat,
		tempPlaneVel, sphereCenterToCollision, tempSphereVel,
		pSphere->getInverseImpulseTenser(), 1.0f / pSphere->getMass());
	
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

	dynamicCollisionResponse(pPoint, tempPos1, tempVel1, tempAngVel1, tempOrrMat1, 
		tempPos2, tempVel2, tempAngVel2, tempOrrMat2, 
		pSphere->getInverseImpulseTenser(), pCuboid->getInverseImpulseTenser(), 
		1.0f / pSphere->getMass(), 1.0f / pSphere->getMass());

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

void CollisionResponse::respondCollisionCuboidCuboid(ManifoldPoint & pPoint, RigidBody * pCuboid1, RigidBody * pCuboid2, bool & pMoved1, bool & pMoved2)
{
	const auto changeTime1 = pPoint.mTime - pCuboid1->getCurrentUpdateTime();

	auto tempPos1 = mix(pCuboid1->getPos(), pCuboid1->getNewPos(), changeTime1);
	auto tempVel1 = mix(pCuboid1->getVel(), pCuboid1->getNewVel(), changeTime1);
	auto tempAngVel1 = mix(pCuboid1->getAngularVelocity(), pCuboid1->getNewAngularVelocity(), changeTime1);
	const auto tempOrr1 = slerp(pCuboid1->getOrientation(), pCuboid1->getNewOrientation(), changeTime1);
	const auto tempOrrMat1 = toMat3(tempOrr1);

	const auto changeTime2 = pPoint.mTime - pCuboid2->getCurrentUpdateTime();

	auto tempPos2 = mix(pCuboid2->getPos(), pCuboid2->getNewPos(), changeTime2);
	auto tempVel2 = mix(pCuboid2->getVel(), pCuboid2->getNewVel(), changeTime2);
	auto tempAngVel2 = mix(pCuboid2->getAngularVelocity(), pCuboid2->getNewAngularVelocity(), changeTime2);
	const auto tempOrr2 = glm::slerp(pCuboid2->getOrientation(), pCuboid2->getNewOrientation(), changeTime2);
	const auto tempOrrMat2 = toMat3(tempOrr1);

	if (pPoint.mCollisionType == CollisionType::PENETRATION)
	{
		tempPos1 = tempPos1 + pPoint.mContactNormal * 0.5f * pPoint.mCollisionDepth;
		tempPos2 = tempPos2 - pPoint.mContactNormal * 0.5f * pPoint.mCollisionDepth;
	}

	dynamicCollisionResponse(pPoint, tempPos1, tempVel1, tempAngVel1, tempOrrMat1,
		tempPos2, tempVel2, tempAngVel2, tempOrrMat2,
		pCuboid1->getInverseImpulseTenser(), pCuboid2->getInverseImpulseTenser(),
		1.0f / pCuboid1->getMass(), 1.0f / pCuboid2->getMass());

	pCuboid1->setNewPos(tempPos1);
	pCuboid1->setNewVel(tempVel1);
	pCuboid1->setNewAngularVel(tempAngVel1);
	pCuboid1->setNewOrientation(tempOrr1);

	pCuboid2->setNewPos(tempPos2);
	pCuboid2->setNewVel(tempVel2);
	pCuboid2->setNewAngularVel(tempAngVel2);
	pCuboid2->setNewOrientation(tempOrr2);

	pMoved1 = true;
	pMoved2 = true;
}

void CollisionResponse::respondCollisionCuboidBowl(ManifoldPoint & pPoint, RigidBody * pCuboid, RigidBody * pBowl, bool & pMoved1, bool & pMoved2)
{
	const auto changeTime = pPoint.mTime - pCuboid->getCurrentUpdateTime();

	auto tempPos = mix(pCuboid->getPos(), pCuboid->getNewPos(), changeTime);
	auto tempVel = mix(pCuboid->getVel(), pCuboid->getNewVel(), changeTime);
	auto tempAngVel = mix(pCuboid->getAngularVelocity(), pCuboid->getNewAngularVelocity(), changeTime);
	const auto tempOrr = slerp(pCuboid->getOrientation(), pCuboid->getNewOrientation(), changeTime);
	const auto tempOrrMat = toMat3(tempOrr);
	
	if (pPoint.mCollisionType == CollisionType::PENETRATION)
	{
		tempPos = tempPos - pPoint.mContactNormal * pPoint.mCollisionDepth;
	}

	const auto tempBowlVel = glm::vec3(0.0f, 0.0f, 0.0f);

	const auto cuboidCenterToCollision = pPoint.mContactPoint1 - tempPos;
	const auto tempSphereVel = tempVel + cross(tempAngVel, cuboidCenterToCollision);

	staticCollisionResponse(pPoint, tempVel, tempAngVel, tempOrrMat,
		tempBowlVel, cuboidCenterToCollision, tempSphereVel,
		pCuboid->getInverseImpulseTenser(), 1.0f / pCuboid->getMass());

	pCuboid->setNewPos(tempPos);
	pCuboid->setNewVel(tempVel);
	pCuboid->setNewOrientation(tempOrr);
	pCuboid->setNewAngularVel(tempAngVel);

	pMoved1 = true;
	pMoved2 = false;
}

void CollisionResponse::respondCollisionCuboidPlane(ManifoldPoint & pPoint, RigidBody * pCuboid, RigidBody * pPlane, bool & pMoved1, bool & pMoved2)
{
	const auto changeTime = pPoint.mTime - pCuboid->getCurrentUpdateTime();

	auto tempPos = mix(pCuboid->getPos(), pCuboid->getNewPos(), changeTime);
	auto tempVel = mix(pCuboid->getVel(), pCuboid->getNewVel(), changeTime);
	auto tempAngVel = mix(pCuboid->getAngularVelocity(), pCuboid->getNewAngularVelocity(), changeTime);
	const auto tempOrr = slerp(pCuboid->getOrientation(), pCuboid->getNewOrientation(), changeTime);
	const auto tempOrrMat = toMat3(tempOrr);

	if (pPoint.mCollisionType == CollisionType::PENETRATION)
	{
		tempPos = tempPos + pPoint.mContactNormal * pPoint.mCollisionDepth;
	}

	const auto planeMat = pPlane->getMatrix();
	const auto center = glm::vec3(planeMat * glm::vec4(pPlane->getPos(), 1.0f));

	const auto newPlaneMat = pPlane->getNewMatrix();
	const auto newCenter = glm::vec3(newPlaneMat * glm::vec4(pPlane->getPos(), 1.0f));

	glm::vec3 tempPlaneVel;

	if (changeTime > 0.0f)
	{
		tempPlaneVel = (newCenter - center) / changeTime;
	}
	else
	{
		tempPlaneVel = glm::vec3(0.0f, 0.0f, 0.0f);
	}

	const auto cuboidCenterToCollision = pPoint.mContactPoint1 - tempPos;
	const auto tempSphereVel = tempVel + cross(tempAngVel, cuboidCenterToCollision);

	staticCollisionResponse(pPoint, tempVel, tempAngVel, tempOrrMat,
		tempPlaneVel, cuboidCenterToCollision, tempSphereVel,
		pCuboid->getInverseImpulseTenser(), 1.0f / pCuboid->getMass());

	pCuboid->setNewPos(tempPos);
	pCuboid->setNewVel(tempVel);
	pCuboid->setNewOrientation(tempOrr);
	pCuboid->setNewAngularVel(tempAngVel);

	pMoved1 = true;
	pMoved2 = false;
}

void CollisionResponse::respondCollisionCuboidPlaneHoles(ManifoldPoint & pPoint, RigidBody * pCuboid, RigidBody * pPlaneHoles, bool & pMoved1, bool & pMoved2)
{
	const auto changeTime = pPoint.mTime - pCuboid->getCurrentUpdateTime();

	auto tempPos = mix(pCuboid->getPos(), pCuboid->getNewPos(), changeTime);
	auto tempVel = mix(pCuboid->getVel(), pCuboid->getNewVel(), changeTime);
	auto tempAngVel = mix(pCuboid->getAngularVelocity(), pCuboid->getNewAngularVelocity(), changeTime);
	const auto tempOrr = slerp(pCuboid->getOrientation(), pCuboid->getNewOrientation(), changeTime);
	const auto tempOrrMat = toMat3(tempOrr);

	if (pPoint.mCollisionType == CollisionType::PENETRATION)
	{
		tempPos = tempPos - pPoint.mContactNormal * pPoint.mCollisionDepth;
	}

	const auto planeMat = pPlaneHoles->getMatrix();
	const auto center = glm::vec3(planeMat * glm::vec4(pPlaneHoles->getPos(), 1.0f));

	const auto newPlaneMat = pPlaneHoles->getNewMatrix();
	const auto newCenter = glm::vec3(newPlaneMat * glm::vec4(pPlaneHoles->getPos(), 1.0f));

	glm::vec3 tempPlaneVel;

	if (changeTime > 0.0f)
	{
		tempPlaneVel = (newCenter - center) / changeTime;
	}
	else
	{
		tempPlaneVel = glm::vec3(0.0f, 0.0f, 0.0f);
	}

	const auto cuboidCenterToCollision = pPoint.mContactPoint1 - tempPos;
	const auto tempSphereVel = tempVel + cross(tempAngVel, cuboidCenterToCollision);

	staticCollisionResponse(pPoint, tempVel, tempAngVel, tempOrrMat,
		tempPlaneVel, cuboidCenterToCollision, tempSphereVel,
		pCuboid->getInverseImpulseTenser(), 1.0f / pCuboid->getMass());

	pCuboid->setNewPos(tempPos);
	pCuboid->setNewVel(tempVel);
	pCuboid->setNewOrientation(tempOrr);
	pCuboid->setNewAngularVel(tempAngVel);

	pMoved1 = true;
	pMoved2 = false;
}

void CollisionResponse::respondCollisionCuboidCylinder(ManifoldPoint& pPoint, RigidBody* pCuboid, RigidBody* pCylinder, bool& pMoved1, bool& pMoved2)
{
	//TODO: Implement
}

void CollisionResponse::respondCollisionSphereCylinder(ManifoldPoint& pPoint, RigidBody* pSphere, RigidBody* pCylinder, bool& pMoved1, bool& pMoved2)
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

	glm::vec3 cylinderVel;
	const auto cylinderHeight = pCylinder->getSize().y * 0.5f;
	const auto cylinderRadius = pCylinder->getSize().x * 0.5f;

	{
		const auto cylinderMat1 = pCylinder->getMatrix();
		const auto cylinderMat2 = pCylinder->getNewMatrix();

		auto cylinderCenter1 = glm::vec3(cylinderMat1 * glm::vec4(0, 0, 0, 1.0f));
		auto cylinderNormal1 = glm::vec3(cylinderMat1 * glm::vec4(0, 1, 0, 1.0f));

		cylinderNormal1 = normalize(cylinderNormal1 - cylinderCenter1);

		auto cylinder1Point1 = cylinderCenter1 + cylinderNormal1 * cylinderHeight;
		auto cylinder1Point2 = cylinderCenter1 - cylinderNormal1 * cylinderHeight;

		auto cylinderCenter2 = glm::vec3(cylinderMat2 * glm::vec4(0, 0, 0, 1.0f));
		auto cylinderNormal2 = glm::vec3(cylinderMat2 * glm::vec4(0, 1, 0, 1.0f));

		cylinderNormal2 = normalize(cylinderNormal2 - cylinderCenter2);

		auto cylinder2Point1 = cylinderCenter2 + cylinderNormal2 * cylinderHeight;
		auto cylinder2Point2 = cylinderCenter2 - cylinderNormal2 * cylinderHeight;

		auto cylinderEndPoint1 = (cylinder1Point2 + cylinder2Point2) / 2.0f;

		if (cylinder1Point1 == cylinder2Point1 && cylinder1Point2 == cylinder2Point2)
		{
			cylinderVel = glm::vec3(0, 0, 0);
		}
		else
		{
			auto angle1 = glm::atan(cylinder1Point1.x, cylinder1Point1.z);
			auto angle2 = glm::atan(cylinder2Point1.x, cylinder2Point1.z);

			if(angle2 < angle1)
			{
				angle2 += 2.0f * M_PI;
			}

			auto speed = (angle2 - angle1) / Game::getUpdateDt();

			cylinderVel = cross(pPoint.mContactPoint2 - cylinderEndPoint1, glm::vec3(0, speed, 0));
		}
	}

	const auto sphereCenterToCollision = pPoint.mContactPoint1 - tempPos;
	const auto tempSphereVel = tempVel + cross(tempAngVel, sphereCenterToCollision);

	staticCollisionResponse(pPoint, tempVel, tempAngVel, tempOrrMat,
		cylinderVel, sphereCenterToCollision, tempSphereVel,
		pSphere->getInverseImpulseTenser(), 1.0f / pSphere->getMass());

	pSphere->setNewPos(tempPos);
	pSphere->setNewVel(tempVel);
	pSphere->setNewOrientation(tempOrr);
	pSphere->setNewAngularVel(tempAngVel);

	pMoved1 = true;
	pMoved2 = false;
}

void CollisionResponse::dynamicCollisionResponse(ManifoldPoint& pPoint, glm::vec3 tempPos1, glm::vec3& tempVel1, glm::vec3& tempAngVel1, const glm::mat3 tempOrrMat1,
	glm::vec3 tempPos2, glm::vec3 & tempVel2, glm::vec3& tempAngVel2, const glm::mat3 tempOrrMat2,
	glm::mat3 inverseImpulseTensor1, glm::mat3 inverseImpulseTensor2, float inverseMass1, float inverseMass2)
{
	const auto sphereCenterToCollision = pPoint.mContactPoint1 - tempPos1;
	const auto cuboidCenterToCollision = pPoint.mContactPoint2 - tempPos2;

	const auto tempSphereVel = tempVel1 + cross(tempAngVel1, sphereCenterToCollision);
	const auto tempCuboidVel = tempVel2 + cross(tempAngVel2, cuboidCenterToCollision);

	const auto relVel = dot(pPoint.mContactNormal, tempSphereVel - tempCuboidVel);

	const auto sphereWorldTensor = tempOrrMat1 * inverseImpulseTensor1 * transpose(tempOrrMat1);
	const auto cuboidWorldTensor = tempOrrMat2 * inverseImpulseTensor2 * transpose(tempOrrMat2);

	const auto sphereInverseMass = inverseMass1;
	const auto cuboidInverseMass = inverseMass2;

	const auto sphereImpulseMag = dot(pPoint.mContactNormal, cross(sphereWorldTensor * cross(sphereCenterToCollision, pPoint.mContactNormal), pPoint.mContactNormal));
	const auto cuboidImpulseMag = dot(pPoint.mContactNormal, cross(cuboidWorldTensor * cross(cuboidCenterToCollision, pPoint.mContactNormal), pPoint.mContactNormal));

	const auto j = -(1.0f + 0.8f) * relVel / (sphereInverseMass + cuboidInverseMass + sphereImpulseMag + cuboidImpulseMag);

	tempVel1 = tempVel1 + j * sphereInverseMass * pPoint.mContactNormal;
	tempVel2 = tempVel2 - j * cuboidInverseMass * pPoint.mContactNormal;
	tempAngVel1 = tempAngVel1 + cross(sphereCenterToCollision, j * pPoint.mContactNormal) * sphereWorldTensor;
	tempAngVel2 = tempAngVel2 - cross(cuboidCenterToCollision, j * pPoint.mContactNormal) * cuboidWorldTensor;
}

void CollisionResponse::staticCollisionResponse(ManifoldPoint& pPoint, glm::vec3 & tempVel, glm::vec3& tempAngVel, const glm::mat3 tempOrrMat, const glm::vec3 tempStaticVel,
	const glm::vec3 rigidBodyCenterToCollision, const glm::vec3 tempRigidBodyVel, const glm::mat3 inverseImpulseTensor, const float inverseMass)
{
	const auto relVel = dot(pPoint.mContactNormal, tempRigidBodyVel - tempStaticVel);

	const auto worldTensor = tempOrrMat * inverseImpulseTensor * transpose(tempOrrMat);

	const auto impulseMag = dot(pPoint.mContactNormal, cross(worldTensor * cross(rigidBodyCenterToCollision, pPoint.mContactNormal), pPoint.mContactNormal));

	const auto j = -(1.0f + 0.8f) * relVel / (inverseMass + impulseMag);

	tempVel = tempVel + j * inverseMass * pPoint.mContactNormal;
	tempAngVel = tempAngVel + worldTensor * cross(rigidBodyCenterToCollision, j * pPoint.mContactNormal);
}
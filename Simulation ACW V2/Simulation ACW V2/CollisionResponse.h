#pragma once

#include "ContactManifold.h"

class CollisionResponse
{
public:
	static void dynamicCollisionResponse(ManifoldPoint & pPoint, bool & pMoved1, bool & pMoved2);

private:
	static void respondCollisionSphereSphere(ManifoldPoint & pPoint, RigidBody * pSphere1, RigidBody * pSphere2, bool &pMoved1, bool & pMoved2);
	static void respondCollisionSphereBowl(ManifoldPoint & pPoint, RigidBody * pSphere, RigidBody * pBowl, bool &pMoved1, bool & pMoved2);
	static void respondCollisionSpherePlane(ManifoldPoint & pPoint, RigidBody * pSphere, RigidBody * pPlane, bool &pMoved1, bool &pMoved2);
	static void respondCollisionSpherePlaneHoles(ManifoldPoint & pPoint, RigidBody * pSphere, RigidBody * pPlaneHoles, bool &pMoved1, bool &pMoved2);
	static void respondCollisionSphereCylinder(ManifoldPoint & pPoint, RigidBody * pSphere, RigidBody * pCylinder, bool & moved1, bool & moved2);
	static void respondCollisionSphereCuboid(ManifoldPoint & pPoint, RigidBody * pSphere, RigidBody * pCuboid, bool &pMoved1, bool & pMoved2);
	static void respondCollisionCuboidCuboid(ManifoldPoint & pPoint, RigidBody * pCuboid1, RigidBody * pCuboid2, bool &pMoved1, bool & pMoved2);
	static void respondCollisionCuboidBowl(ManifoldPoint & pPoint, RigidBody * pCuboid, RigidBody * pBowl, bool & pMoved1, bool & pMoved2);
	static void respondCollisionCuboidPlane(ManifoldPoint & pPoint, RigidBody * pCuboid, RigidBody * pPlane, bool & pMoved1, bool & pMoved2);
	static void respondCollisionCuboidPlaneHoles(ManifoldPoint & pPoint, RigidBody * pCuboid, RigidBody * pPlaneHoles, bool & pMoved1, bool & pMoved2);
	static void respondCollisionCuboidCylinder(ManifoldPoint & pPoint, RigidBody * pCuboid, RigidBody * pCylinder, bool & pMoved1, bool & pMoved2);

	static void dynamicCollisionResponse(ManifoldPoint& pPoint, glm::vec3 tempPos1, glm::vec3& tempVel1, glm::vec3& tempAngVel1, const glm::mat3 tempOrrMat1,
		glm::vec3 tempPos2, glm::vec3 & tempVel2, glm::vec3& tempAngVel2, const glm::mat3 tempOrrMat2,
		glm::mat3 inverseImpulseTensor1, glm::mat3 inverseImpulseTensor2, float inverseMass1, float inverseMass2);
};


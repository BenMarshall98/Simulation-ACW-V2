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
	static void respondCollisionSphereCuboid(ManifoldPoint & pPoint, RigidBody * pSphere, RigidBody * pCuboid, bool &pMoved1, bool & pMoved2);
	static void respondCollisionCuboidCuboid(ManifoldPoint & pPoint, RigidBody * pCuboid1, RigidBody * pCuboid2, bool &pMoved1, bool & pMoved2);
	static void respondCollisionCuboidBowl(ManifoldPoint & pPoint, RigidBody * pCuboid, RigidBody * pBowl, bool & pMoved1, bool & pMoved2);
	static void respondCollisionCuboidPlane(ManifoldPoint & pPoint, RigidBody * pCuboid, RigidBody * pPlane, bool & pMoved1, bool & pMoved2);
	static void respondCollisionCuboidPlaneHoles(ManifoldPoint & pPoint, RigidBody * pCuboid, RigidBody * pPlaneHoles, bool & pMoved1, bool & pMoved2);
};


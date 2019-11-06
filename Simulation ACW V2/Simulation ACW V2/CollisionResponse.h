#pragma once

#include "ContactManifold.h"
#include "Sphere.h"
#include "Plane.h"
#include "PlaneHoles.h"

class CollisionResponse
{
public:
	static void dynamicCollisionResponse(ManifoldPoint & pPoint, bool & moved1, bool & moved2);

private:
	static void respondCollisionSphereSphere(ManifoldPoint & pPoint, RigidBody * pSphere1, RigidBody * pSphere2, bool &moved1, bool & moved2);
	static void respondCollisionSphereBowl(ManifoldPoint & pPoint, RigidBody * pSphere, RigidBody * pBowl, bool &moved1, bool & moved2);
	static void respondCollisionSpherePlane(ManifoldPoint & pPoint, RigidBody * pSphere, RigidBody * pPlane, bool &moved1, bool &moved2);
	static void respondCollisionSpherePlaneHoles(ManifoldPoint & pPoint, RigidBody * pSphere, RigidBody * pPlaneHoles, bool &moved1, bool &moved2);
	static void respondCollisionSphereCuboid(ManifoldPoint & pPoint, RigidBody * pSphere, RigidBody * pCuboid, bool &moved1, bool & moved2);
	static void respondCollisionCuboidCuboid(ManifoldPoint & pPoint, RigidBody * pCuboid1, RigidBody * pCuboid2, bool &moved1, bool & moved2);
	static void respondCollisionCuboidBowl(ManifoldPoint & pPoint, RigidBody * pCuboid, RigidBody * pBowl, bool & moved1, bool & moved2);
	static void respondCollisionCuboidPlane(ManifoldPoint & pPoint, RigidBody * pCuboid, RigidBody * pPlane, bool & moved1, bool & moved2);
	static void respondCollisionCuboidPlaneHoles(ManifoldPoint & pPoint, RigidBody * pCuboid, RigidBody * pPlaneHoles, bool & moved1, bool & moved2);
};


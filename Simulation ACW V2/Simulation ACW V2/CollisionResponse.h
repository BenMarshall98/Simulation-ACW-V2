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
	static void respondCollisionSphereCubiod(ManifoldPoint & pPoint, RigidBody * pSphere, RigidBody * pCuboid, bool &moved1, bool & moved2);
	static void respondCollisionCubiodCubiod(ManifoldPoint & pPoint, RigidBody * pCubiod1, RigidBody * pCubiod2, bool &moved1, bool & moved2);
	static void respondCollisionCubiodBowl(ManifoldPoint & pPoint, RigidBody * pCubiod, RigidBody * pBowl, bool & moved1, bool & moved2);
	static void respondCollisionCubiodPlane(ManifoldPoint & pPoint, RigidBody * pCubiod, RigidBody * pPlane, bool & moved1, bool & moved2);
	static void respondCollisionCubiodPlaneHoles(ManifoldPoint & pPoint, RigidBody * pCubiod, RigidBody * pPlaneHoles, bool & moved1, bool & moved2);
};


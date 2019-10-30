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
	static void respondCollisionSpherePlane(ManifoldPoint & pPoint, RigidBody * pSphere, RigidBody * pPlane, bool &moved1, bool &moved2);
	static void respondCollisionSpherePlaneHoles(ManifoldPoint & pPoint, RigidBody * pSphere, RigidBody * pPlaneHoles, bool &moved1, bool &moved2);
};


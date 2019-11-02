#pragma once

#include "RigidBody.h"
#include "ContactManifold.h"

#include "Sphere.h"
#include "Plane.h"
#include "PlaneHoles.h"

class CollisionDetection
{
public:
	static void dynamicCollisionDetection(RigidBody * pRigidBody1, RigidBody * pRigidBody2, ContactManifold * pManifold, float pLastCollisionTime);

private:
	static void detectCollisionSphereSphere(Sphere * pSphere1, Sphere * pSphere2, ContactManifold * pManifold, float pLastCollisionTime);
	static void detectCollisionSpherePlane(Sphere * pSphere, Plane * pPlane, ContactManifold * pManifold, float pLastCollisionTime);
	static void detectCollisionSpherePlaneHoles(Sphere * pSphere, PlaneHoles * pPlaneHoles, ContactManifold * pManifold, float pLastCollisionTime);

	static bool detectCollisionSphereLine(Sphere * pSphere1, Vector3F pLineEnd1, Vector3F pLineEnd2, float & pTime, float pLastCollisionTime);
	static bool detectCollisionSphereVertex(Sphere * pSphere, Vector3F pVertex, float & pTime, float pLastCollisionTime);
};


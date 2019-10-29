#pragma once

#include "RigidBody.h"
#include "ContactManifold.h"

#include "Sphere.h"
#include "Plane.h"
#include "PlaneHoles.h"

class CollisionDetection
{
public:
	static void dynamicCollisionDetection(RigidBody * pRigidBody1, RigidBody * pRigidBody2, ContactManifold * pManifold);

private:
	static void detectCollisionSphereSphere(Sphere * pSphere1, Sphere * pSphere2, ContactManifold * pManifold);
	static void detectCollisionSpherePlane(Sphere * pSphere, Plane * pPlane, ContactManifold * pManifold);
	static void detectCollisionSpherePlaneHoles(Sphere * pSphere, PlaneHoles * pPlaneHoles, ContactManifold * pManifold);

	static bool detectCollisionSphereLine(Sphere * pSphere1, Vector3F pLineEnd1, Vector3F pLineEnd2, float & pTime);
	static bool detectCollisionSphereVertex(Sphere * pSphere, Vector3F pVertex, float & pTime);
};


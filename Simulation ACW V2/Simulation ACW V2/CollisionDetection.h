#pragma once

#include "RigidBody.h"
#include "ContactManifold.h"

#include "Sphere.h"
#include "Plane.h"
#include "PlaneHoles.h"
#include "Bowl.h"
#include "Cuboid.h"

class CollisionDetection
{
public:
	static void dynamicCollisionDetection(RigidBody * pRigidBody1, RigidBody * pRigidBody2, ContactManifold * pManifold, float pLastCollisionTime);

private:
	static void detectCollisionSphereSphere(Sphere * pSphere1, Sphere * pSphere2, ContactManifold * pManifold, float pLastCollisionTime);
	static void detectCollisionSpherePlane(Sphere * pSphere, Plane * pPlane, ContactManifold * pManifold, float pLastCollisionTime);
	static void detectCollisionSpherePlaneHoles(Sphere * pSphere, PlaneHoles * pPlaneHoles, ContactManifold * pManifold, float pLastCollisionTime);
	static void detectCollisionSphereBowl(Sphere * pSphere, Bowl * pBowl, ContactManifold * pManifold, float pLastCollisionTime);
	static void detectCollisionSphereCuboid(Sphere * pSphere, Cuboid * pCuboid, ContactManifold * pManifold, float pLastCollisionTime);

	static bool detectCollisionSphereLine(Sphere * pSphere, Vector3F pLineEnd1, Vector3F pLineEnd2, Vector3F pLineVelocity, float & pTime, float pLastCollisionTime);
	static bool detectCollisionSphereVertex(Sphere * pSphere, Vector3F pVertex, Vector3F pVertexVelocity, float & pTime, float pLastCollisionTime);
	static bool detectCollisionSphereTriangle(Sphere * pSphere, Vector3F pVertex1, Vector3F pVertex2, Vector3F pVertex3, Vector3F pTriangleVelocity, float pLastCollisionTime);
	static bool detectCollisionSphereCuboidStep(Vector3F pSphereCenter, float pSphereRadius, Vector3F pCuboidCenter, Vector3F pCuboidXAxis, Vector3F pCuboidYAxis, Vector3F pCuboidZAxis, Vector3F pCuboidSize, Vector3F & pPoint);
	static Vector3F calculateCuboidCollisionNormal(Vector3F pCuboidCenter, Vector3F pCuboidXAxis, Vector3F pCuboidYAxis, Vector3F pCuboidZAxis, Vector3F pCuboidSize, Vector3F pPoint);
	static bool detectCollisionCuboidCuboidStep(Vector3F pCuboid1Center, Vector3F pCuboidXAxis1, Vector3F pCuboidYAxis1, Vector3F pCuboidZAxis1, Vector3F pCuboidSize1, Vector3F pCuboid2Center, Vector3F pCuboidXAxis2, Vector3F pCuboidYAxis2, Vector3F pCuboidZAxis2, Vector3F pCuboidSize2);
	static float calculateCuboidCuboidCollisionDepth(Vector3F pPoint, Vector3F pCuboidCenter, Vector3F pCuboidXAxis, Vector3F pCuboidYAxis, Vector3F pCuboidZAxis, Vector3F pCuboidSize, bool & inside, Vector3F & pCollisionPoint);
};


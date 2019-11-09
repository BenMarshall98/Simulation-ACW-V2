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

	static bool detectCollisionSphereLine(Sphere * pSphere, glm::vec3 pLineEnd1, glm::vec3 pLineEnd2, glm::vec3 pLineVelocity, float & pTime, float pLastCollisionTime);
	static bool detectCollisionSphereVertex(Sphere * pSphere, glm::vec3 pVertex, glm::vec3 pVertexVelocity, float & pTime, float pLastCollisionTime);
	static bool detectCollisionSphereTriangle(Sphere * pSphere, glm::vec3 pVertex1, glm::vec3 pVertex2, glm::vec3 pVertex3, glm::vec3 pTriangleVelocity, float pLastCollisionTime);
	static bool detectCollisionSphereCuboidStep(glm::vec3 pSphereCenter, float pSphereRadius, glm::vec3 pCuboidCenter, glm::vec3 pCuboidXAxis, glm::vec3 pCuboidYAxis, glm::vec3 pCuboidZAxis, glm::vec3 pCuboidSize, glm::vec3 & pPoint);
	static glm::vec3 calculateCuboidCollisionNormal(glm::vec3 pCuboidCenter, glm::vec3 pCuboidXAxis, glm::vec3 pCuboidYAxis, glm::vec3 pCuboidZAxis, glm::vec3 pCuboidSize, glm::vec3 pPoint);
	static bool detectCollisionCuboidCuboidStep(glm::vec3 pCuboid1Center, glm::vec3 pCuboidXAxis1, glm::vec3 pCuboidYAxis1, glm::vec3 pCuboidZAxis1, glm::vec3 pCuboidSize1, glm::vec3 pCuboid2Center, glm::vec3 pCuboidXAxis2, glm::vec3 pCuboidYAxis2, glm::vec3 pCuboidZAxis2, glm::vec3 pCuboidSize2);
	static float calculateCuboidCuboidCollisionDepth(glm::vec3 pPoint, glm::vec3 pCuboidCenter, glm::vec3 pCuboidXAxis, glm::vec3 pCuboidYAxis, glm::vec3 pCuboidZAxis, glm::vec3 pCuboidSize, bool & inside, glm::vec3 & pCollisionPoint);
};


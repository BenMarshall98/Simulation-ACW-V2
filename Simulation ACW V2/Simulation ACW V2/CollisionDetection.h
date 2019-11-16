#pragma once

#include "RigidBody.h"
#include "ContactManifold.h"

class Cylinder;

class CollisionDetection
{
public:
	static void dynamicCollisionDetection(RigidBody * pRigidBody1, RigidBody * pRigidBody2, ContactManifold * pManifold, float pLastCollisionTime);

private:
	static void detectCollisionSphereSphere(RigidBody * pSphere1, RigidBody * pSphere2, ContactManifold * pManifold, float pLastCollisionTime);
	static void detectCollisionSpherePlane(RigidBody * pSphere, RigidBody * pPlane, ContactManifold * pManifold, float pLastCollisionTime);
	static void detectCollisionSpherePlaneHoles(RigidBody * pSphere, RigidBody * pPlaneHoles, ContactManifold * pManifold, float pLastCollisionTime);
	static void detectCollisionSphereBowl(RigidBody * pSphere, RigidBody * pBowl, ContactManifold * pManifold, float pLastCollisionTime);
	static void detectCollisionSphereCylinder(RigidBody * pSphere, RigidBody * pCylinder, ContactManifold * pManifold, float pLastCollisionTime);
	static void detectCollisionSphereCuboid(RigidBody * pSphere, RigidBody * pCuboid, ContactManifold * pManifold, float pLastCollisionTime);
	static void detectCollisionCuboidCuboid(RigidBody * pCuboid1, RigidBody * pCuboid2, ContactManifold * pManifold, float pLastCollisionTime);
	static void detectCollisionCuboidPlane(RigidBody * pCuboid, RigidBody * pPlane, ContactManifold * pManifold, float pLastCollisionTime);
	static void detectCollisionCuboidPlaneHoles(RigidBody * pCuboid, RigidBody * pPlaneHoles, ContactManifold * pManifold, float pLastCollisionTime);
	static void detectCollisionCuboidBowl(RigidBody * pCuboid, RigidBody * pBowl, ContactManifold * pManifold, float pLastCollisionTime);
	static void detectCollisionCuboidCylinder(RigidBody * pSphere,RigidBody * pCylinder, ContactManifold * pManifold, float pLastCollisionTime);

	static bool detectCollisionSphereLine(RigidBody * pSphere, glm::vec3 pLineEnd1, glm::vec3 pLineEnd2, glm::vec3 pLineVelocity, float & pTime, float pLastCollisionTime);
	static bool detectCollisionSphereVertex(RigidBody * pSphere, glm::vec3 pVertex, glm::vec3 pVertexVelocity, float & pTime, float pLastCollisionTime);
	static bool detectCollisionSphereTriangle(RigidBody * pSphere, glm::vec3 pVertex1, glm::vec3 pVertex2, glm::vec3 pVertex3, glm::vec3 pTriangleVelocity, float pLastCollisionTime);
	static bool detectCollisionSphereCuboidStep(glm::vec3 pSphereCenter, float pSphereRadius, glm::vec3 pCuboidCenter, glm::vec3 pCuboidXAxis, glm::vec3 pCuboidYAxis, glm::vec3 pCuboidZAxis, glm::vec3 pCuboidSize, glm::vec3 & pPoint);
	static glm::vec3 calculateCuboidCollisionNormal(glm::vec3 pCuboidCenter, glm::vec3 pCuboidXAxis, glm::vec3 pCuboidYAxis, glm::vec3 pCuboidZAxis, glm::vec3 pCuboidSize, glm::vec3 pPoint);
	static bool detectCollisionCuboidCuboidStep(glm::vec3 pCuboid1Center, std::vector<glm::vec3> pCuboidAxis1, glm::vec3 pCuboidSize1, glm::vec3 pCuboid2Center, std::vector<glm::vec3> pCuboidAxis2, glm::vec3 pCuboidSize2);
	static float calculateCuboidCuboidCollisionDepth(glm::vec3 pPoint, glm::vec3 pCuboidCenter, std::vector<glm::vec3> pCuboidAxis, glm::vec3 pCuboidSize, bool & pInside, glm::vec3 & pCollisionPoint);
	static bool calculateCuboidPointPlaneCollision(glm::vec3 pCuboidCenter, glm::vec3 pCuboidPoint, glm::vec3 pPlaneCenter, std::vector<glm::vec3> pPlaneAxis, glm::vec3 pPlaneSize, glm::vec3 & pCollisionPoint);
	static float calculateClosestPointBetweenLines(glm::vec3 pLine1Start, glm::vec3 pLine1End, glm::vec3 pLine2Start, glm::vec3 pLine2End, glm::vec3 & pLine1Point, glm::vec3 & pLine2Point);
	static bool detectCollisionSphereCylinderStep(glm::vec3 pCylinderCenter, glm::vec3 pCylinderEndPoint1, glm::vec3 pCylinderEndPoint2, float pCylinderRadius, float pCylinderHeight, glm::vec3 pSpherePos, float pSphereRadius, float & distance, glm::vec3 & pCollisionPoint);
	static bool detectCollisionLineTriangle(glm::vec3 pLineStart, glm::vec3 pLineEnd, glm::vec3 pTrianglePoint1, glm::vec3 pTrianglePoint2, glm::vec3 pTrianglePoint3);
};


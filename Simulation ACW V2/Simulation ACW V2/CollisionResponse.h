#pragma once

#include "ContactManifold.h"
#include "Sphere.h"
#include "Plane.h"
#include "PlaneHoles.h"

class CollisionResponse
{
public:
	static void dynamicCollisionResponse(ManifoldPoint & pPoint);

private:
	static void respondCollisionSphereSphere(Sphere * pSphere1, Sphere * pSphere2, ManifoldPoint & pPoint);
	static void respondCollisionSpherePlane(Sphere * pSphere, Plane * pPlane, ManifoldPoint & pPoint);
	static void respondCollisionSpherePlaneHoles(Sphere * pSphere, PlaneHoles * pPlaneHoles, ManifoldPoint & pPoint);
};


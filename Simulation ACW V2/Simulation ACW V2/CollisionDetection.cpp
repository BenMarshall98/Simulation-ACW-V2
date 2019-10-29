#include "CollisionDetection.h"
#include "Game.h"

void CollisionDetection::dynamicCollisionDetection(RigidBody* pRigidBody1, RigidBody* pRigidBody2, ContactManifold* pManifold)
{
	if (pRigidBody1->getObjectType() == ObjectType::SPHERE && pRigidBody2->getObjectType() == ObjectType::SPHERE)
	{
		auto * sphere1 = dynamic_cast<Sphere *>(pRigidBody1);
		auto * sphere2 = dynamic_cast<Sphere *>(pRigidBody2);

		detectCollisionSphereSphere(sphere1, sphere2, pManifold);
	}
	else if (pRigidBody1->getObjectType() == ObjectType::SPHERE && pRigidBody2->getObjectType() == ObjectType::PLANE)
	{
		auto * sphere = dynamic_cast<Sphere*>(pRigidBody1);
		auto * plane = dynamic_cast<Plane *>(pRigidBody2);

		detectCollisionSpherePlane(sphere, plane, pManifold);
	}
	else if (pRigidBody1->getObjectType() == ObjectType::PLANE && pRigidBody2->getObjectType() == ObjectType::SPHERE)
	{
		auto * sphere = dynamic_cast<Sphere*>(pRigidBody2);
		auto * plane = dynamic_cast<Plane*>(pRigidBody1);

		detectCollisionSpherePlane(sphere, plane, pManifold);
	}
	else if (pRigidBody1->getObjectType() == ObjectType::SPHERE && pRigidBody2->getObjectType() == ObjectType::PLANEHOLES)
	{
		auto * sphere = dynamic_cast<Sphere *>(pRigidBody1);
		auto * planeHoles = dynamic_cast<PlaneHoles *>(pRigidBody2);

		detectCollisionSpherePlaneHoles(sphere, planeHoles, pManifold);
	}
	else if (pRigidBody1->getObjectType() == ObjectType::PLANEHOLES && pRigidBody2->getObjectType() == ObjectType::SPHERE)
	{
		auto * sphere = dynamic_cast<Sphere *>(pRigidBody2);
		auto * planeHoles = dynamic_cast<PlaneHoles *>(pRigidBody1);

		detectCollisionSpherePlaneHoles(sphere, planeHoles, pManifold);
	}
}

void CollisionDetection::detectCollisionSphereSphere(Sphere* pSphere1, Sphere* pSphere2, ContactManifold* pManifold)
{
	Vector3F relCenter = pSphere2->getPos() - pSphere1->getPos();

	Vector3F velSphere1 = pSphere1->getNewPos() - pSphere1->getPos();
	Vector3F velSphere2 = pSphere2->getNewPos() - pSphere2->getPos();

	Vector3F relVelocity = velSphere2 - velSphere1;

	float radius = pSphere1->getSize().getX() + pSphere2->getSize().getX();

	float c = relCenter.dot(relCenter) - radius * radius;

	float a = relVelocity.dot(relVelocity);

	if (a == 0)
	{
		return; //Not moving relative to each other
	}

	float b = relVelocity.dot(relCenter);

	float discriminant = b * b - a * c;

	if (discriminant < 0)
	{
		return; //Spheres never interset
	}

	float time = (-b - sqrt(discriminant)) / a;

	if (time >= 0 && time <= 1)
	{
		Vector3F sphere1Point = pSphere1->getPos() + (time * velSphere1);
		Vector3F sphere2Point = pSphere2->getPos() + (time * velSphere2);

		ManifoldPoint manPoint;
		manPoint.mContactId1 = pSphere1;
		manPoint.mContactId2 = pSphere2;
		manPoint.mContactNormal = (sphere1Point - sphere2Point).normalise();
		manPoint.mTime = time;

		pManifold->add(manPoint);
	}
}

void CollisionDetection::detectCollisionSpherePlane(Sphere* pSphere, Plane* pPlane, ContactManifold* pManifold)
{
	Matrix4F planeMat = pPlane->getMatrix();

	Vector3F center = Vector3F(0, 0, 0) * planeMat;
	Vector3F normal = Vector3F(0, 1, 0) * planeMat;
	Vector3F tangent = Vector3F(1, 0, 0) * planeMat;
	Vector3F bitangent = Vector3F(0, 0, 1) * planeMat;

	normal = (normal - center).normalise();
	tangent = (tangent - center).normalise();
	bitangent = (bitangent - center).normalise();

	center = pPlane->getPos() * planeMat;

	Vector3F velocity = (pSphere->getNewPos() - pSphere->getPos());

	float planeDot = normal.dot(center + tangent);

	float dist = normal.dot(pSphere->getPos()) - planeDot;

	float velDot = normal.dot(velocity);

	if (velDot == 0)
	{
		return; //Moving parallel no collision;
	}

	float radius = dist > 0.0f ? pSphere->getSize().getX() : -pSphere->getSize().getX();
	float time = (radius - dist) / velDot;

	if (time >= 0 && time <= 1)
	{
		Vector3F spherePoint = pSphere->getPos() + (time * velocity);
		Vector3F collisionPoint = spherePoint - (radius * normal);

		float sizeX = pPlane->getSize().getX();
		float sizeY = pPlane->getSize().getZ();

		float dotX = (collisionPoint - center).dot(tangent);
		float dotY = (collisionPoint - center).dot(bitangent);

		radius = pSphere->getSize().getX();

		if (dotX <= sizeX && dotX >= -sizeX &&
			dotY <= sizeY && dotY >= -sizeY)
		{
			ManifoldPoint manPoint;
			manPoint.mContactId1 = pSphere;
			manPoint.mContactId2 = pPlane;
			manPoint.mContactNormal = (collisionPoint - spherePoint).normalise();
			manPoint.mTime = time;

			pManifold->add(manPoint);
		}
		else if (dotX <= sizeX + radius && dotX >= -sizeX - radius &&
			dotY <= sizeY + radius && dotY >= -sizeY - radius)
		{
			std::vector<Vector3F> pStartPoints =
			{
				center + sizeX * tangent + sizeY * bitangent,
				center + sizeX * tangent - sizeY * bitangent,
				center - sizeX * tangent + sizeY * bitangent,
				center - sizeX * tangent - sizeY * bitangent
			};

			std::vector<Vector3F> pEndPoints =
			{
				center + sizeX * tangent - sizeY * bitangent,
				center - sizeX * tangent - sizeY * bitangent,
				center + sizeX * tangent + sizeY * bitangent,
				center - sizeX * tangent + sizeY * bitangent
			};

			for (int i = 0; i < pStartPoints.size(); i++)
			{
				if (detectCollisionSphereLine(pSphere, pStartPoints[i], pEndPoints[i], time))
				{
					Vector3F spherePoint = pSphere->getPos() + (time * velocity);

					Vector3F lineVector = pEndPoints[i] - pStartPoints[i];

					float t = (spherePoint - pStartPoints[i]).dot(lineVector) / lineVector.dot(lineVector);

					Vector3F closestPoint = pStartPoints[i] + t * lineVector;

					ManifoldPoint manPoint;
					manPoint.mContactId1 = pSphere;
					manPoint.mContactId2 = pPlane;
					manPoint.mContactNormal = (closestPoint - spherePoint).normalise();
					manPoint.mTime = time;

					pManifold->add(manPoint);
				}
			}

			for (int i = 0; i < pStartPoints.size(); i++)
			{
				if (detectCollisionSphereVertex(pSphere, pStartPoints[i], time))
				{
					Vector3F spherePoint = pSphere->getPos() + (time * velocity);

					Vector3F closestPoint = pStartPoints[i];

					ManifoldPoint manPoint;
					manPoint.mContactId1 = pSphere;
					manPoint.mContactId2 = pPlane;
					manPoint.mContactNormal = (closestPoint - spherePoint).normalise();
					manPoint.mTime = time;

					pManifold->add(manPoint);
				}
			}
		}
	}
}

void CollisionDetection::detectCollisionSpherePlaneHoles(Sphere* pSphere, PlaneHoles* pPlaneHoles, ContactManifold* pManifold)
{
	Matrix4F planeMat = pPlaneHoles->getMatrix();

	Vector3F center = Vector3F(0, 0, 0) * planeMat;
	Vector3F normal = Vector3F(0, 1, 0) * planeMat;
	Vector3F tangent = Vector3F(1, 0, 0) * planeMat;
	Vector3F bitangent = Vector3F(0, 0, 1) * planeMat;

	normal = (normal - center).normalise();
	tangent = (tangent - center).normalise();
	bitangent = (bitangent - center).normalise();

	center = pPlaneHoles->getPos() * planeMat;

	Vector3F velocity = (pSphere->getNewPos() - pSphere->getPos());

	float planeDot = normal.dot(center + tangent);

	float dist = normal.dot(pSphere->getPos()) - planeDot;

	float velDot = normal.dot(velocity);

	if (velDot == 0)
	{
		return; //Moving parallel no collision;
	}

	float radius = dist > 0.0f ? pSphere->getSize().getX() : -pSphere->getSize().getX();
	float time = (radius - dist) / velDot;

	if (time >= 0 && time <= 1)
	{
		Vector3F spherePoint = pSphere->getPos() + (time * velocity);
		Vector3F collisionPoint = spherePoint - (radius * normal);

		float sizeX = pPlaneHoles->getSize().getX() * 5;
		float sizeY = pPlaneHoles->getSize().getZ() * 5;

		float dotX = (collisionPoint - center).dot(tangent);
		float dotY = (collisionPoint - center).dot(bitangent);

		radius = pSphere->getSize().getX();

		if (dotX <= sizeX && dotX >= -sizeX &&
			dotY <= sizeY && dotY >= -sizeY)
		{
			ManifoldPoint manPoint;
			manPoint.mContactId1 = pSphere;
			manPoint.mContactId2 = pPlaneHoles;
			manPoint.mContactNormal = (collisionPoint - spherePoint).normalise();
			manPoint.mTime = time;

			pManifold->add(manPoint);
		}
		else if (dotX <= sizeX + radius && dotX >= -sizeX - radius &&
			dotY <= sizeY + radius && dotY >= -sizeY - radius)
		{
			std::vector<Vector3F> pStartPoints =
			{
				center + sizeX * tangent + sizeY * bitangent,
				center + sizeX * tangent - sizeY * bitangent,
				center - sizeX * tangent + sizeY * bitangent,
				center - sizeX * tangent - sizeY * bitangent
			};

			std::vector<Vector3F> pEndPoints =
			{
				center + sizeX * tangent - sizeY * bitangent,
				center - sizeX * tangent - sizeY * bitangent,
				center + sizeX * tangent + sizeY * bitangent,
				center - sizeX * tangent + sizeY * bitangent
			};

			for (int i = 0; i < pStartPoints.size(); i++)
			{
				if (detectCollisionSphereLine(pSphere, pStartPoints[i], pEndPoints[i], time))
				{
					Vector3F spherePoint = pSphere->getPos() + (time * velocity);

					Vector3F lineVector = pEndPoints[i] - pStartPoints[i];

					float t = (spherePoint - pStartPoints[i]).dot(lineVector) / lineVector.dot(lineVector);

					Vector3F closestPoint = pStartPoints[i] + t * lineVector;

					ManifoldPoint manPoint;
					manPoint.mContactId1 = pSphere;
					manPoint.mContactId2 = pPlaneHoles;
					manPoint.mContactNormal = (closestPoint - spherePoint).normalise();
					manPoint.mTime = time;

					pManifold->add(manPoint);
				}
			}

			for (int i = 0; i < pStartPoints.size(); i++)
			{
				if (detectCollisionSphereVertex(pSphere, pStartPoints[i], time))
				{
					Vector3F spherePoint = pSphere->getPos() + (time * velocity);

					Vector3F closestPoint = pStartPoints[i];

					ManifoldPoint manPoint;
					manPoint.mContactId1 = pSphere;
					manPoint.mContactId2 = pPlaneHoles;
					manPoint.mContactNormal = (closestPoint - spherePoint).normalise();
					manPoint.mTime = time;

					pManifold->add(manPoint);
				}
			}
		}
	}
}

//http://www.peroxide.dk/papers/collision/collision.pdf

bool CollisionDetection::detectCollisionSphereLine(Sphere* pSphere1, Vector3F pLineEnd1, Vector3F pLineEnd2, float& pTime)
{
	Vector3F d = pLineEnd2 - pLineEnd1;
	Vector3F n = pSphere1->getNewPos() - pSphere1->getPos();
	Vector3F m = pSphere1->getPos() - pLineEnd1;
	float radius = pSphere1->getSize().getX();

	float md = m.dot(d);
	float nd = n.dot(d);
	float dd = d.dot(d);

	if (md < 0.0 && md + nd < 0.0)
	{
		return false; //Outside cylinder
	}

	if (md > dd && md + nd > dd)
	{
		return false; //Outside cylinder
	}

	float mn = m.dot(n);
	float nn = n.dot(n);

	float a = dd * nn - nd * nd;
	float k = m.dot(m) - radius * radius;
	float c = dd * k - md * md;

	if (a == 0)
	{
		return false;
	}

	float b = dd * mn - nd * md;
	float discr = b * b - a * c;

	if (discr < 0.0f)
	{
		return false;
	}

	pTime = (-b - sqrt(discr)) / a;

	if (pTime < 0.0f || pTime > 1.0f)
	{
		return false;
	}

	return true;
}

bool CollisionDetection::detectCollisionSphereVertex(Sphere* pSphere, Vector3F pVertex, float& pTime)
{
	Vector3F sphereStart = pSphere->getPos();
	Vector3F sphereEnd = pSphere->getNewPos();
	float radius = pSphere->getSize().getX();

	Vector3F d = sphereEnd - sphereStart;

	Vector3F m = sphereStart - pVertex;

	float a = d.dot(d);
	float b = m.dot(d);
	float c = m.dot(m) - radius * radius;

	if (c > 0.0f && b > 0.0f)
	{
		return false;
	}

	float discr = b * b - a * c;

	if (discr < 0.0f)
	{
		return false;
	}

	pTime = -b - sqrt(discr);

	if (pTime < 0.0f || pTime > 1.0f)
	{
		return false;
	}

	return true;
}


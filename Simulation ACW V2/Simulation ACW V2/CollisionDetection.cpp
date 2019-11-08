#include "CollisionDetection.h"
#include "Game.h"
#include <corecrt_math_defines.h>

void CollisionDetection::dynamicCollisionDetection(RigidBody* pRigidBody1, RigidBody* pRigidBody2, ContactManifold* pManifold, float pLastCollisionTime)
{
	if (pRigidBody1->getObjectType() == ObjectType::SPHERE && pRigidBody2->getObjectType() == ObjectType::SPHERE)
	{
		auto * sphere1 = dynamic_cast<Sphere *>(pRigidBody1);
		auto * sphere2 = dynamic_cast<Sphere *>(pRigidBody2);

		detectCollisionSphereSphere(sphere1, sphere2, pManifold, pLastCollisionTime);
	}
	else if (pRigidBody1->getObjectType() == ObjectType::SPHERE && pRigidBody2->getObjectType() == ObjectType::PLANE)
	{
		auto * sphere = dynamic_cast<Sphere*>(pRigidBody1);
		auto * plane = dynamic_cast<Plane *>(pRigidBody2);

		detectCollisionSpherePlane(sphere, plane, pManifold, pLastCollisionTime);
	}
	else if (pRigidBody1->getObjectType() == ObjectType::PLANE && pRigidBody2->getObjectType() == ObjectType::SPHERE)
	{
		auto * sphere = dynamic_cast<Sphere*>(pRigidBody2);
		auto * plane = dynamic_cast<Plane*>(pRigidBody1);

		detectCollisionSpherePlane(sphere, plane, pManifold, pLastCollisionTime);
	}
	else if (pRigidBody1->getObjectType() == ObjectType::SPHERE && pRigidBody2->getObjectType() == ObjectType::PLANEHOLES)
	{
		auto * sphere = dynamic_cast<Sphere *>(pRigidBody1);
		auto * planeHoles = dynamic_cast<PlaneHoles *>(pRigidBody2);

		detectCollisionSpherePlaneHoles(sphere, planeHoles, pManifold, pLastCollisionTime);
	}
	else if (pRigidBody1->getObjectType() == ObjectType::PLANEHOLES && pRigidBody2->getObjectType() == ObjectType::SPHERE)
	{
		auto * sphere = dynamic_cast<Sphere *>(pRigidBody2);
		auto * planeHoles = dynamic_cast<PlaneHoles *>(pRigidBody1);

		detectCollisionSpherePlaneHoles(sphere, planeHoles, pManifold, pLastCollisionTime);
	}
	else if (pRigidBody1->getObjectType() == ObjectType::SPHERE && pRigidBody2->getObjectType() == ObjectType::BOWL)
	{
		auto * sphere = dynamic_cast<Sphere *>(pRigidBody1);
		auto * bowl = dynamic_cast<Bowl *>(pRigidBody2);

		detectCollisionSphereBowl(sphere, bowl, pManifold, pLastCollisionTime);
	}
	else if (pRigidBody1->getObjectType() == ObjectType::BOWL && pRigidBody2->getObjectType() == ObjectType::SPHERE)
	{
		auto * sphere = dynamic_cast<Sphere *>(pRigidBody2);
		auto * bowl = dynamic_cast<Bowl *>(pRigidBody1);

		detectCollisionSphereBowl(sphere, bowl, pManifold, pLastCollisionTime);
	}
}

void CollisionDetection::detectCollisionSphereSphere(Sphere* pSphere1, Sphere* pSphere2, ContactManifold* pManifold, float pLastCollisionTime)
{
	Vector3F sphere1pos;
	Vector3F sphere2pos;
	
	if (pSphere1->getCurrentUpdateTime() < pLastCollisionTime)
	{
		float interValue = (pLastCollisionTime - pSphere1->getCurrentUpdateTime()) / (1.0f - pSphere1->getCurrentUpdateTime());
		
		sphere1pos = pSphere1->getPos().interpolate(pSphere1->getNewPos(), interValue);
	}
	else
	{
		sphere1pos = pSphere1->getPos();
	}

	if (pSphere2->getCurrentUpdateTime() < pLastCollisionTime)
	{
		float interValue = (pLastCollisionTime - pSphere2->getCurrentUpdateTime()) / (1.0f - pSphere2->getCurrentUpdateTime());

		sphere2pos = pSphere2->getPos().interpolate(pSphere2->getNewPos(), interValue);
	}
	else
	{
		sphere2pos = pSphere2->getPos();
	}
	
	Vector3F relCenter = sphere2pos - sphere1pos;

	Vector3F velSphere1 = pSphere1->getNewPos() - sphere1pos;
	Vector3F velSphere2 = pSphere2->getNewPos() - sphere2pos;

	Vector3F relVelocity = velSphere2 - velSphere1;

	float radius = pSphere1->getSize().getX() + pSphere2->getSize().getX();

	float c = relCenter.dot(relCenter) - radius * radius;

	if (c < 0.0f)
	{
		ManifoldPoint manPoint;
		manPoint.mContactId1 = pSphere1;
		manPoint.mContactId2 = pSphere2;
		manPoint.mContactNormal = (sphere1pos - sphere2pos).normalize();
		manPoint.mTime = pLastCollisionTime;
		manPoint.mCollisionDepth = radius - (sphere1pos - sphere2pos).length();
		manPoint.mCollisionType = CollisionType::PENETRATION;

		pManifold->add(manPoint);
		return;
	}

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
		Vector3F sphere1Point = sphere1pos + (time * velSphere1);
		Vector3F sphere2Point = sphere2pos + (time * velSphere2);

		ManifoldPoint manPoint;
		manPoint.mContactId1 = pSphere1;
		manPoint.mContactId2 = pSphere2;
		manPoint.mContactNormal = (sphere1Point - sphere2Point).normalize();
		manPoint.mTime = pLastCollisionTime + time * (1.0f - pLastCollisionTime);
		manPoint.mCollisionType = CollisionType::COLLISION;

		pManifold->add(manPoint);
	}
}

void CollisionDetection::detectCollisionSphereBowl(Sphere* pSphere, Bowl* pBowl, ContactManifold* pManifold, float pLastCollisionTime)
{
	Vector3F spherePos;

	if (pSphere->getCurrentUpdateTime() < pLastCollisionTime)
	{
		float interValue = (pLastCollisionTime - pSphere->getCurrentUpdateTime()) / (1.0f - pSphere->getCurrentUpdateTime());

		spherePos = pSphere->getPos() + interValue * (pSphere->getNewPos() - pSphere->getPos());
	}
	else
	{
		spherePos = pSphere->getPos();
	}

	Matrix4F bowlMat = pBowl->getMatrix();

	Vector3F center = pBowl->getPos() * bowlMat;

	Vector3F relCenter = spherePos - center;

	Vector3F velSphere = pSphere->getNewPos() - spherePos;

	float bowlRadius = pBowl->getSize().getX();
	float sphereRadius = pSphere->getSize().getX();
	float radius = bowlRadius - sphereRadius;

	float c = relCenter.dot(relCenter) - radius * radius;

	if (c >= 0.0f)
	{
		ManifoldPoint manPoint;
		manPoint.mContactId1 = pSphere;
		manPoint.mContactId2 = pBowl;
		manPoint.mContactNormal = (spherePos - center).normalize();
		manPoint.mTime = pLastCollisionTime;
		manPoint.mCollisionDepth = (spherePos - center).length() - radius;
		manPoint.mCollisionType = CollisionType::PENETRATION;

		pManifold->add(manPoint);
		return;
	}

	if (velSphere.length() == 0.0f)
	{
		return;
	}

	Vector3F d = (velSphere * -1) / velSphere.length();
	float b = relCenter.dot(d);
	c = relCenter.dot(relCenter) - radius * radius;

	float discr = b * b - c;

	if (discr < 0.0f)
	{
		return;
	}

	float time = -b - sqrt(discr);

	Vector3F intersection = spherePos + time * (d);

	if (abs(time) >= velSphere.length())
	{
		return;
	}
	
	Vector3F spherePoint = spherePos + (abs(time) * velSphere);

	ManifoldPoint manPoint;
	manPoint.mContactId1 = pSphere;
	manPoint.mContactId2 = pBowl;
	manPoint.mContactNormal = (spherePoint - intersection).normalize();
	manPoint.mTime = pLastCollisionTime + abs(time) * (1.0f - pLastCollisionTime);
	manPoint.mCollisionType = CollisionType::COLLISION;

	pManifold->add(manPoint);
}

void CollisionDetection::detectCollisionSpherePlane(Sphere* pSphere, Plane* pPlane, ContactManifold* pManifold, float pLastCollisionTime)
{
	Vector3F spherePos;

	if (pSphere->getCurrentUpdateTime() < pLastCollisionTime)
	{
		float interValue = (pLastCollisionTime - pSphere->getCurrentUpdateTime()) / (1.0f - pSphere->getCurrentUpdateTime());

		spherePos = pSphere->getPos() + interValue * (pSphere->getNewPos() - pSphere->getPos());
	}
	else
	{
		spherePos = pSphere->getPos();
	}

	Matrix4F planeMat = pPlane->getMatrix();

	Vector3F center = Vector3F(0, 0, 0) * planeMat;
	Vector3F normal = Vector3F(0, 1, 0) * planeMat;
	Vector3F tangent = Vector3F(1, 0, 0) * planeMat;
	Vector3F bitangent = Vector3F(0, 0, 1) * planeMat;

	normal = (normal - center).normalize();
	tangent = (tangent - center).normalize();
	bitangent = (bitangent - center).normalize();

	center = pPlane->getPos() * planeMat;

	Matrix4F newPlaneMat = pPlane->getNewMatrix();

	auto newCenter = pPlane->getPos() * newPlaneMat;

	Vector3F planeVelocity = newCenter - center;

	Vector3F velocity = (pSphere->getNewPos() - spherePos) - planeVelocity;

	float planeDot = normal.dot(center + tangent);

	float dist = normal.dot(spherePos) - planeDot;

	float radius = dist > 0.0f ? pSphere->getSize().getX() : -pSphere->getSize().getX();

	if (abs(dist) <= abs(radius))
	{
		Vector3F spherePoint = spherePos;
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
			manPoint.mContactNormal = normal.normalize();
			manPoint.mTime = pLastCollisionTime;
			manPoint.mCollisionType = CollisionType::PENETRATION;
			manPoint.mCollisionDepth = abs(radius) - abs(dist);
			manPoint.mContactPoint1 = collisionPoint;
			manPoint.mContactPoint2 = collisionPoint;

			pManifold->add(manPoint);
			return;
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

			float time = 0.0f;
			
			for (auto i = 0u; i < pStartPoints.size(); i++)
			{
				if (detectCollisionSphereLine(pSphere, pStartPoints[i], pEndPoints[i], planeVelocity, time, pLastCollisionTime))
				{
					spherePoint = spherePos + (time * velocity);

					Vector3F lineVector = pEndPoints[i] - pStartPoints[i];

					float t = (spherePoint - pStartPoints[i]).dot(lineVector) / lineVector.dot(lineVector);

					Vector3F closestPoint = pStartPoints[i] + t * lineVector;

					ManifoldPoint manPoint;
					manPoint.mContactId1 = pSphere;
					manPoint.mContactId2 = pPlane;
					manPoint.mContactNormal = normal.normalize();
					manPoint.mTime = pLastCollisionTime + time * (1.0f - pLastCollisionTime);
					manPoint.mCollisionType = CollisionType::COLLISION;
					manPoint.mContactPoint1 = closestPoint;
					manPoint.mContactPoint2 = closestPoint;

					pManifold->add(manPoint);
					return;
				}
			}

			for (auto i = 0u; i < pStartPoints.size(); i++)
			{
				if (detectCollisionSphereVertex(pSphere, pStartPoints[i], planeVelocity, time, pLastCollisionTime))
				{
					spherePoint = spherePos + (time * velocity);

					Vector3F closestPoint = pStartPoints[i];

					ManifoldPoint manPoint;
					manPoint.mContactId1 = pSphere;
					manPoint.mContactId2 = pPlane;
					manPoint.mContactNormal = normal.normalize();
					manPoint.mTime = pLastCollisionTime + time * (1.0f - pLastCollisionTime);
					manPoint.mCollisionType = CollisionType::PENETRATION;
					manPoint.mContactPoint1 = closestPoint;
					manPoint.mContactPoint2 = closestPoint;

					pManifold->add(manPoint);
					return;
				}
			}
		}
	}

	float velDot = normal.dot(velocity);

	if (velDot * dist >= 0)
	{
		return; //Moving parallel no collision;
	}

	float time = (radius - dist) / velDot;

	if (time >= 0 && time <= 1)
	{
		Vector3F spherePoint = spherePos + (time * velocity);
		Vector3F collisionPoint = spherePoint - (radius * normal);

		float sizeX = pPlane->getSize().getX();
		float sizeY = pPlane->getSize().getZ();

		float dotX = (collisionPoint - center + planeVelocity * time).dot(tangent);
		float dotY = (collisionPoint - center + planeVelocity * time).dot(bitangent);

		radius = pSphere->getSize().getX();

		if (dotX <= sizeX && dotX >= -sizeX &&
			dotY <= sizeY && dotY >= -sizeY)
		{
			ManifoldPoint manPoint;
			manPoint.mContactId1 = pSphere;
			manPoint.mContactId2 = pPlane;
			manPoint.mContactNormal = normal.normalize();
			manPoint.mTime = pLastCollisionTime + time * (1.0f - pLastCollisionTime);
			manPoint.mCollisionType = CollisionType::COLLISION;
			manPoint.mContactPoint1 = collisionPoint;
			manPoint.mContactPoint2 = collisionPoint;

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
				if (detectCollisionSphereLine(pSphere, pStartPoints[i], pEndPoints[i], planeVelocity, time, pLastCollisionTime))
				{
					Vector3F spherePoint = spherePos + (time * velocity);

					Vector3F lineVector = pEndPoints[i] - pStartPoints[i];

					float t = (spherePoint - pStartPoints[i]).dot(lineVector) / lineVector.dot(lineVector);

					Vector3F closestPoint = pStartPoints[i] + t * lineVector;

					ManifoldPoint manPoint;
					manPoint.mContactId1 = pSphere;
					manPoint.mContactId2 = pPlane;
					manPoint.mContactNormal = normal.normalize();
					manPoint.mTime = pLastCollisionTime + time * (1.0f - pLastCollisionTime);
					manPoint.mCollisionType = CollisionType::COLLISION;
					manPoint.mContactPoint1 = closestPoint;
					manPoint.mContactPoint2 = closestPoint;

					pManifold->add(manPoint);
					return;
				}
			}

			for (auto i = 0u; i < pStartPoints.size(); i++)
			{
				if (detectCollisionSphereVertex(pSphere, pStartPoints[i], planeVelocity, time, pLastCollisionTime))
				{
					Vector3F spherePoint = spherePos + (time * velocity);

					Vector3F closestPoint = pStartPoints[i];

					ManifoldPoint manPoint;
					manPoint.mContactId1 = pSphere;
					manPoint.mContactId2 = pPlane;
					manPoint.mContactNormal = normal.normalize();
					manPoint.mTime = pLastCollisionTime + time * (1.0f - pLastCollisionTime);
					manPoint.mCollisionType = CollisionType::PENETRATION;
					manPoint.mContactPoint1 = closestPoint;
					manPoint.mContactPoint2 = closestPoint;

					pManifold->add(manPoint);
					return;
				}
			}
		}
	}
}

void CollisionDetection::detectCollisionSpherePlaneHoles(Sphere* pSphere, PlaneHoles* pPlaneHoles, ContactManifold* pManifold, float pLastCollisionTime)
{
	Vector3F spherePos;

	if (pSphere->getCurrentUpdateTime() < pLastCollisionTime)
	{
		float interValue = (pLastCollisionTime - pSphere->getCurrentUpdateTime()) / (1.0f - pSphere->getCurrentUpdateTime());

		spherePos = pSphere->getPos() + interValue * (pSphere->getNewPos() - pSphere->getPos());
	}
	else
	{
		spherePos = pSphere->getPos();
	}
	
	Matrix4F planeMat = pPlaneHoles->getMatrix();

	Vector3F center = Vector3F(0, 0, 0) * planeMat;
	Vector3F normal = Vector3F(0, 1, 0) * planeMat;
	Vector3F tangent = Vector3F(1, 0, 0) * planeMat;
	Vector3F bitangent = Vector3F(0, 0, 1) * planeMat;

	normal = (normal - center).normalize();
	tangent = (tangent - center).normalize();
	bitangent = (bitangent - center).normalize();

	center = pPlaneHoles->getPos() * planeMat;

	Matrix4F newPlaneMat = pPlaneHoles->getNewMatrix();

	auto newCenter = pPlaneHoles->getPos() * newPlaneMat;

	Vector3F planeVelocity = newCenter - center;

	Vector3F velocity = (pSphere->getNewPos() - spherePos) - planeVelocity;

	float planeDot = normal.dot(center + tangent);

	float dist = normal.dot(spherePos) - planeDot;

	float radius = dist > 0.0f ? pSphere->getSize().getX() : -pSphere->getSize().getX();

	if (abs(dist) <= abs(radius))
	{
		Vector3F spherePoint = spherePos;
		Vector3F collisionPoint = spherePoint - (radius * normal);

		float sizeX = pPlaneHoles->getSize().getX() * 5;
		float sizeY = pPlaneHoles->getSize().getZ() * 5;

		float dotX = (collisionPoint - center).dot(tangent);
		float dotY = (collisionPoint - center).dot(bitangent);

		radius = pSphere->getSize().getX();
		float time = 0.0f;
		if (dotX <= sizeX && dotX >= -sizeX &&
			dotY <= sizeY && dotY >= -sizeY)
		{
			std::vector<Vector3F> centers =
			{
				-3 * tangent,
				-3 * tangent + 3 * bitangent,
				3 * bitangent,
				3 * tangent + 3 * bitangent,
				3 * tangent,
				3 * tangent + -3 * bitangent,
				-3 * bitangent,
				-3 * tangent + -3 * bitangent
			};

			for (int i = 0; i < centers.size(); i++)
			{
				if (dotX <= centers[i].getX() + 1 && dotX >= centers[i].getX() - 1 &&
					dotY <= centers[i].getZ() + 1 && dotY >= centers[i].getZ() - 1)
				{
					bool insideCircle = false;
					int segments = 20;
					const auto angle = static_cast<float>(M_PI * 2) / segments;

					for (int j = 0; j < segments; j++)
					{
						Vector3F tempCenter = center + centers[i];
						Vector3F tempVertex1 = tempCenter + cos(j * angle) * tangent + sin(j * angle) * bitangent;
						Vector3F tempVertex2 = tempCenter + cos((j + 1) * angle) * tangent + sin((j + 1) * angle) * bitangent;

						if (detectCollisionSphereTriangle(pSphere, tempCenter, tempVertex1, tempVertex2, planeVelocity, pLastCollisionTime))
						{
							insideCircle = true;

							if (detectCollisionSphereLine(pSphere, tempVertex1, tempVertex2, planeVelocity, time, pLastCollisionTime))
							{
								Vector3F spherePoint = spherePos + (time * velocity);

								Vector3F lineVector = tempVertex2 - tempVertex1;

								float t = (spherePoint - tempVertex1).dot(lineVector) / lineVector.dot(lineVector);

								Vector3F closestPoint = tempVertex1 + t * lineVector;

								ManifoldPoint manPoint;
								manPoint.mContactId1 = pSphere;
								manPoint.mContactId2 = pPlaneHoles;
								manPoint.mContactNormal = (closestPoint - spherePoint).normalize();
								manPoint.mTime = pLastCollisionTime + time * (1.0f - pLastCollisionTime);
								manPoint.mCollisionType = CollisionType::COLLISION;

								pManifold->add(manPoint);
								return;
							}
						}
					}

					if (insideCircle)
					{
						return;
					}

					break;
				}
			}
			
			ManifoldPoint manPoint;
			manPoint.mContactId1 = pSphere;
			manPoint.mContactId2 = pPlaneHoles;
			manPoint.mContactNormal = (collisionPoint - spherePoint).normalize();
			manPoint.mTime = pLastCollisionTime;
			manPoint.mCollisionType = CollisionType::PENETRATION;
			manPoint.mCollisionDepth = abs(radius) - abs(dist);

			pManifold->add(manPoint);
			return;
		}
		else if (dotX <= sizeX + 2 * radius && dotX >= -sizeX - 2 * radius &&
			dotY <= sizeY + 2 * radius && dotY >= -sizeY - 2 * radius)
		{
			std::vector<Vector3F> startPoints =
			{
				center + sizeX * tangent + sizeY * bitangent,
				center + sizeX * tangent - sizeY * bitangent,
				center - sizeX * tangent + sizeY * bitangent,
				center - sizeX * tangent - sizeY * bitangent
			};

			std::vector<Vector3F> endPoints =
			{
				center + sizeX * tangent - sizeY * bitangent,
				center - sizeX * tangent - sizeY * bitangent,
				center + sizeX * tangent + sizeY * bitangent,
				center - sizeX * tangent + sizeY * bitangent
			};

			for (int i = 0; i < startPoints.size(); i++)
			{
				if (detectCollisionSphereLine(pSphere, startPoints[i], endPoints[i], planeVelocity, time, pLastCollisionTime))
				{
					Vector3F spherePoint = spherePos + (time * velocity);

					Vector3F lineVector = endPoints[i] - startPoints[i];

					float t = (spherePoint - startPoints[i]).dot(lineVector) / lineVector.dot(lineVector);

					Vector3F closestPoint = startPoints[i] + t * lineVector;

					ManifoldPoint manPoint;
					manPoint.mContactId1 = pSphere;
					manPoint.mContactId2 = pPlaneHoles;
					manPoint.mContactNormal = (closestPoint - spherePoint).normalize();
					manPoint.mTime = pLastCollisionTime + time * (1.0f - pLastCollisionTime);
					manPoint.mCollisionType = CollisionType::COLLISION;

					pManifold->add(manPoint);
					return;
				}
			}

			for (int i = 0; i < startPoints.size(); i++)
			{
				if (detectCollisionSphereVertex(pSphere, startPoints[i], planeVelocity, time, pLastCollisionTime))
				{
					Vector3F spherePoint = spherePos + (time * velocity);

					Vector3F closestPoint = startPoints[i];

					ManifoldPoint manPoint;
					manPoint.mContactId1 = pSphere;
					manPoint.mContactId2 = pPlaneHoles;
					manPoint.mContactNormal = (closestPoint - spherePoint).normalize();
					manPoint.mTime = pLastCollisionTime + time * (1.0f - pLastCollisionTime);
					manPoint.mCollisionType = CollisionType::PENETRATION;

					pManifold->add(manPoint);
					return;
				}
			}
		}
	}

	float velDot = normal.dot(velocity);

	if (velDot * dist >= 0)
	{
		return; //Moving parallel no collision;
	}
	
	float time = (radius - dist) / velDot;

	if (time >= 0 && time <= 1)
	{
		Vector3F spherePoint = spherePos + (time * velocity);
		Vector3F collisionPoint = spherePoint - (radius * normal);

		float sizeX = pPlaneHoles->getSize().getX() * 5;
		float sizeY = pPlaneHoles->getSize().getZ() * 5;

		float dotX = (collisionPoint - center + planeVelocity * time).dot(tangent);
		float dotY = (collisionPoint - center + planeVelocity * time).dot(bitangent);

		radius = pSphere->getSize().getX();

		if (dotX <= sizeX && dotX >= -sizeX &&
			dotY <= sizeY && dotY >= -sizeY)
		{
			std::vector<Vector3F> centers =
			{
				-3 * tangent,
				-3 * tangent + 3 * bitangent,
				3 * bitangent,
				3 * tangent + 3 * bitangent,
				3 * tangent,
				3 * tangent + -3 * bitangent,
				-3 * bitangent,
				-3 * tangent + -3 * bitangent
			};

			for (int i = 0; i < centers.size(); i++)
			{
				if (dotX <= centers[i].getX() + 1 && dotX >= centers[i].getX() - 1 &&
					dotY <= centers[i].getZ() + 1 && dotY >= centers[i].getZ() - 1)
				{
					bool insideCircle = false;
					int segments = 20;
					const auto angle = static_cast<float>(M_PI * 2) / segments;
					
					for (int j = 0; j < segments; j++)
					{
						Vector3F tempCenter = center + centers[i];
						Vector3F tempVertex1 = tempCenter + cos(j * angle) * tangent + sin(j * angle) * bitangent;
						Vector3F tempVertex2 = tempCenter + cos((j + 1) * angle) * tangent + sin((j + 1) * angle) * bitangent;

						if (detectCollisionSphereTriangle(pSphere, tempCenter, tempVertex1, tempVertex2, planeVelocity, pLastCollisionTime))
						{
							insideCircle = true;

							if(detectCollisionSphereLine(pSphere, tempVertex1, tempVertex2, planeVelocity, time, pLastCollisionTime))
							{
								Vector3F spherePoint = spherePos + (time * velocity);

								Vector3F lineVector = tempVertex2 - tempVertex1;

								float t = (spherePoint - tempVertex1).dot(lineVector) / lineVector.dot(lineVector);

								Vector3F closestPoint = tempVertex1 + t * lineVector;

								ManifoldPoint manPoint;
								manPoint.mContactId1 = pSphere;
								manPoint.mContactId2 = pPlaneHoles;
								manPoint.mContactNormal = (closestPoint - spherePoint).normalize();
								manPoint.mTime = pLastCollisionTime + time * (1.0f - pLastCollisionTime);
								manPoint.mCollisionType = CollisionType::COLLISION;

								pManifold->add(manPoint);
								return;
							}
						}
					}

					if (insideCircle)
					{
						return;
					}
					
					break;
				}
			}
			
			ManifoldPoint manPoint;
			manPoint.mContactId1 = pSphere;
			manPoint.mContactId2 = pPlaneHoles;
			manPoint.mContactNormal = (collisionPoint - spherePoint).normalize();
			manPoint.mTime = pLastCollisionTime + time * (1.0f - pLastCollisionTime);
			manPoint.mCollisionType = CollisionType::COLLISION;

			pManifold->add(manPoint);
		}
		else if (dotX <= sizeX + 2 * radius && dotX >= -sizeX - 2 * radius &&
			dotY <= sizeY + 2 * radius && dotY >= -sizeY - 2 * radius)
		{
			std::vector<Vector3F> startPoints =
			{
				center + sizeX * tangent + sizeY * bitangent,
				center + sizeX * tangent - sizeY * bitangent,
				center - sizeX * tangent + sizeY * bitangent,
				center - sizeX * tangent - sizeY * bitangent
			};

			std::vector<Vector3F> endPoints =
			{
				center + sizeX * tangent - sizeY * bitangent,
				center - sizeX * tangent - sizeY * bitangent,
				center + sizeX * tangent + sizeY * bitangent,
				center - sizeX * tangent + sizeY * bitangent
			};

			for (int i = 0; i < startPoints.size(); i++)
			{
				if (detectCollisionSphereLine(pSphere, startPoints[i], endPoints[i], planeVelocity, time, pLastCollisionTime))
				{
					Vector3F spherePoint = spherePos + (time * velocity);

					Vector3F lineVector = endPoints[i] - startPoints[i];

					float t = (spherePoint - startPoints[i]).dot(lineVector) / lineVector.dot(lineVector);

					Vector3F closestPoint = startPoints[i] + t * lineVector;

					ManifoldPoint manPoint;
					manPoint.mContactId1 = pSphere;
					manPoint.mContactId2 = pPlaneHoles;
					manPoint.mContactNormal = (closestPoint - spherePoint).normalize();
					manPoint.mTime = pLastCollisionTime + time * (1.0f - pLastCollisionTime);
					manPoint.mCollisionType = CollisionType::COLLISION;

					pManifold->add(manPoint);
					return;
				}
			}

			for (int i = 0; i < startPoints.size(); i++)
			{
				if (detectCollisionSphereVertex(pSphere, startPoints[i], planeVelocity, time, pLastCollisionTime))
				{
					Vector3F spherePoint = spherePos + (time * velocity);

					Vector3F closestPoint = startPoints[i];

					ManifoldPoint manPoint;
					manPoint.mContactId1 = pSphere;
					manPoint.mContactId2 = pPlaneHoles;
					manPoint.mContactNormal = (closestPoint - spherePoint).normalize();
					manPoint.mTime = pLastCollisionTime + time * (1.0f - pLastCollisionTime);
					manPoint.mCollisionType = CollisionType::PENETRATION;

					pManifold->add(manPoint);
					return;
				}
			}
		}
	}
}

//http://www.peroxide.dk/papers/collision/collision.pdf

bool CollisionDetection::detectCollisionSphereLine(Sphere* pSphere, Vector3F pLineEnd1, Vector3F pLineEnd2, Vector3F pLineVelocity, float& pTime, float pLastCollisionTime)
{
	Vector3F spherePos;

	if (pSphere->getCurrentUpdateTime() < pLastCollisionTime)
	{
		float interValue = (pLastCollisionTime - pSphere->getCurrentUpdateTime()) / (1.0f - pSphere->getCurrentUpdateTime());

		spherePos = pSphere->getPos() + interValue * (pSphere->getNewPos() - pSphere->getPos());
	}
	else
	{
		spherePos = pSphere->getPos();
	}
	
	Vector3F d = pLineEnd2 - pLineEnd1;
	Vector3F n = (pSphere->getNewPos()+ pLineVelocity) - spherePos;
	Vector3F m = spherePos - pLineEnd1;
	float radius = pSphere->getSize().getX();

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
		if (c > 0.0f)
		{
			return false;
		}
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

bool CollisionDetection::detectCollisionSphereVertex(Sphere* pSphere, Vector3F pVertex, Vector3F pVertexVelocity, float& pTime, float pLastCollisionTime)
{
	Vector3F spherePos;

	if (pSphere->getCurrentUpdateTime() < pLastCollisionTime)
	{
		float interValue = (pLastCollisionTime - pSphere->getCurrentUpdateTime()) / (1.0f - pSphere->getCurrentUpdateTime());

		spherePos = pSphere->getPos() + interValue * (pSphere->getNewPos() - pSphere->getPos());
	}
	else
	{
		spherePos = pSphere->getPos();
	}
	
	Vector3F sphereStart = spherePos;
	Vector3F sphereEnd = pSphere->getNewPos() - pVertexVelocity;
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

bool CollisionDetection::detectCollisionSphereTriangle(Sphere* pSphere, Vector3F pVertex1, Vector3F pVertex2, Vector3F pVertex3, Vector3F pTriangleVelocity, float pLastCollisionTime)
{
	Vector3F spherePos;

	if (pSphere->getCurrentUpdateTime() < pLastCollisionTime)
	{
		float interValue = (pLastCollisionTime - pSphere->getCurrentUpdateTime()) / (1.0f - pSphere->getCurrentUpdateTime());

		spherePos = pSphere->getPos() + interValue * (pSphere->getNewPos() - pSphere->getPos());
	}
	else
	{
		spherePos = pSphere->getPos();
	}

	Vector3F velocity = (pSphere->getNewPos() - spherePos) - pTriangleVelocity;
	float radius = pSphere->getSize().getX();

	Vector3F planeNormal = (pVertex2 - pVertex1).cross(pVertex3 - pVertex1).normalize();
	float planeDot = planeNormal.dot(pVertex2);

	float dist = planeNormal.dot(spherePos) - planeDot;

	if (abs(dist) <= radius)
	{
		Vector3F collisionPoint = spherePos - ((-radius + radius - abs(dist)) * planeNormal);

		Vector3F tempVertex1 = pVertex1 - collisionPoint;
		Vector3F tempVertex2 = pVertex2 - collisionPoint;
		Vector3F tempVertex3 = pVertex3 - collisionPoint;

		Vector3F u = tempVertex2.cross(tempVertex3);
		Vector3F v = tempVertex3.cross(tempVertex1);

		if (u.dot(v) < 0.0f)
		{
			return false;
		}

		Vector3F w = tempVertex1.cross(tempVertex2);

		if (u.dot(w) < 0.0f)
		{
			return false;
		}

		return true;
	}

	float denom = planeNormal.dot(velocity);

	if (denom * dist >= 0.0f)
	{
		return false;
	}

	radius = dist > 0.0f ? radius : -radius;

	float time = (radius - dist) / denom;

	Vector3F spherePoint = spherePos + (time * velocity);
	Vector3F collisionPoint = spherePoint - (radius * planeNormal);

	Vector3F tempVertex1 = pVertex1 - collisionPoint;
	Vector3F tempVertex2 = pVertex2 - collisionPoint;
	Vector3F tempVertex3 = pVertex3 - collisionPoint;

	Vector3F u = tempVertex2.cross(tempVertex3);
	Vector3F v = tempVertex3.cross(tempVertex1);

	if (u.dot(v) < 0.0f)
	{
		return false;
	}

	Vector3F w = tempVertex1.cross(tempVertex2);

	if (u.dot(w) < 0.0f)
	{
		return false;
	}

	return true;
}

void CollisionDetection::detectCollisionSphereCuboid(Sphere * pSphere, Cuboid * pCuboid, ContactManifold * pManifold, float pLastCollisionTime)
{
	Vector3F spherePos;

	if (pSphere->getCurrentUpdateTime() < pLastCollisionTime)
	{
		float interValue = (pLastCollisionTime - pSphere->getCurrentUpdateTime()) / (1.0f - pSphere->getCurrentUpdateTime());

		spherePos = pSphere->getPos().interpolate(pSphere->getNewPos(), interValue);
	}
	else
	{
		spherePos = pSphere->getPos();
	}

	Vector3F cuboidPos;
	Matrix3F cuboidOrientation;

	if (pCuboid->getCurrentUpdateTime() < pLastCollisionTime)
	{
		float interValue = (pLastCollisionTime - pCuboid->getCurrentUpdateTime()) / (1.0f - pCuboid->getCurrentUpdateTime());

		cuboidPos = pCuboid->getPos().interpolate(pCuboid->getNewPos(), interValue);
		cuboidOrientation = pCuboid->getOrientation().interpolate(pCuboid->getNewOrientation(), interValue);
	}
	else
	{
		cuboidPos = pCuboid->getPos();
		cuboidOrientation = pCuboid->getOrientation();
	}

	float sphereRadius = pSphere->getSize().getX();
	Vector3F cuboidSize = pCuboid->getSize();

	Vector3F cuboidXAxis = Vector3F(1, 0, 0) * cuboidOrientation;
	Vector3F cuboidYAxis = Vector3F(0, 1, 0) * cuboidOrientation;
	Vector3F cuboidZAxis = Vector3F(0, 0, 1) * cuboidOrientation;

	Vector3F closestPoint;
	if (detectCollisionSphereCuboidStep(spherePos, sphereRadius, cuboidPos, cuboidXAxis, cuboidYAxis, cuboidZAxis, cuboidSize, closestPoint))
	{
		ManifoldPoint manPoint;
		manPoint.mContactId1 = pSphere;
		manPoint.mContactId2 = pCuboid;
		manPoint.mContactNormal = calculateCuboidCollisionNormal(cuboidPos, cuboidXAxis, cuboidYAxis, cuboidZAxis, cuboidSize, closestPoint).normalize();
		manPoint.mCollisionDepth = sphereRadius - (spherePos - closestPoint).length();
		manPoint.mContactPoint1 = spherePos + sphereRadius * (closestPoint - spherePos);
		manPoint.mContactPoint2 = closestPoint;
	}


}

bool CollisionDetection::detectCollisionSphereCuboidStep(Vector3F pSphereCenter, float pSphereRadius, Vector3F pCuboidCenter,
	Vector3F pCuboidXAxis, Vector3F pCuboidYAxis, Vector3F pCuboidZAxis, Vector3F pCuboidSize, Vector3F & pPoint)
{
	//Get closest point on cuboid from sphere center
	Vector3F d = pSphereCenter - pCuboidCenter;

	pPoint = pCuboidCenter;

	{
		float dist = d.dot(pCuboidXAxis);

		if (dist > pCuboidSize.getX())
		{
			dist = pCuboidSize.getX();
		}
		else if (dist < -pCuboidSize.getX())
		{
			dist = -pCuboidSize.getX();
		}

		pPoint = pPoint + dist * pCuboidXAxis;
	}

	{
		float dist = d.dot(pCuboidYAxis);

		if (dist > pCuboidSize.getY())
		{
			dist = pCuboidSize.getY();
		}
		else if (dist < -pCuboidSize.getY())
		{
			dist = -pCuboidSize.getY();
		}

		pPoint = pPoint + dist * pCuboidYAxis;
	}

	{
		float dist = d.dot(pCuboidZAxis);

		if (dist > pCuboidSize.getZ())
		{
			dist = pCuboidSize.getZ();
		}
		else if (dist < -pCuboidSize.getZ())
		{
			dist = -pCuboidSize.getZ();
		}

		pPoint = pPoint + dist * pCuboidZAxis;
	}

	//Check if point on cuboid is touching sphere, therefore colliding

	Vector3F dist = pPoint - pSphereCenter;

	return dist.dot(dist) <= pSphereRadius * pSphereRadius;
}

Vector3F CollisionDetection::calculateCuboidCollisionNormal(Vector3F pCuboidCenter, Vector3F pCuboidXAxis, Vector3F pCuboidYAxis, Vector3F pCuboidZAxis, Vector3F pCuboidSize, Vector3F pPoint)
{
	Vector3F d = pPoint - pCuboidCenter;

	float Xdist = d.dot(pCuboidXAxis);
	float Ydist = d.dot(pCuboidYAxis);
	float Zdist = d.dot(pCuboidZAxis);

	if (Xdist == pCuboidSize.getX() &&
		Ydist == pCuboidSize.getY() &&
		Zdist == pCuboidSize.getZ())
	{
		return pCuboidXAxis + pCuboidYAxis + pCuboidZAxis;
	}
	else if (Xdist == pCuboidSize.getX() &&
		Ydist == pCuboidSize.getY() &&
		Zdist == -pCuboidSize.getZ())
	{
		return pCuboidXAxis + pCuboidYAxis - pCuboidZAxis;
	}
	else if (Xdist == pCuboidSize.getX() &&
		Ydist == -pCuboidSize.getY() &&
		Zdist == pCuboidSize.getZ())
	{
		return pCuboidXAxis - pCuboidYAxis + pCuboidZAxis;
	}
	else if (Xdist == pCuboidSize.getX() &&
		Ydist == -pCuboidSize.getY() &&
		Zdist == -pCuboidSize.getZ())
	{
		return pCuboidXAxis - pCuboidYAxis - pCuboidZAxis;
	}
	if (Xdist == -pCuboidSize.getX() &&
		Ydist == pCuboidSize.getY() &&
		Zdist == pCuboidSize.getZ())
	{
		return (-1 * pCuboidXAxis) + pCuboidYAxis + pCuboidZAxis;
	}
	else if (Xdist == -pCuboidSize.getX() &&
		Ydist == pCuboidSize.getY() &&
		Zdist == -pCuboidSize.getZ())
	{
		return (-1 * pCuboidXAxis) + pCuboidYAxis - pCuboidZAxis;
	}
	else if (Xdist == -pCuboidSize.getX() &&
		Ydist == -pCuboidSize.getY() &&
		Zdist == pCuboidSize.getZ())
	{
		return (-1 * pCuboidXAxis) - pCuboidYAxis + pCuboidZAxis;
	}
	else if (Xdist == -pCuboidSize.getX() &&
		Ydist == -pCuboidSize.getY() &&
		Zdist == -pCuboidSize.getZ())
	{
		return (-1 * pCuboidXAxis) - pCuboidYAxis - pCuboidZAxis;
	}
	else if (Xdist == pCuboidSize.getX() &&
		Ydist == pCuboidSize.getY())
	{
		return pCuboidXAxis + pCuboidYAxis;
	}
	else if (Xdist == pCuboidSize.getX() &&
		Ydist == -pCuboidSize.getY())
	{
		return pCuboidXAxis - pCuboidYAxis;
	}
	else if (Xdist == pCuboidSize.getX() &&
		Zdist == pCuboidSize.getZ())
	{
		return pCuboidXAxis + pCuboidZAxis;
	}
	else if (Xdist == pCuboidSize.getX() &&
		Zdist == -pCuboidSize.getZ())
	{
		return pCuboidXAxis - pCuboidZAxis;
	}
	else if (Xdist == -pCuboidSize.getX() &&
		Ydist == pCuboidSize.getY())
	{
		return (-1.0f * pCuboidXAxis) + pCuboidYAxis;
	}
	else if (Xdist == -pCuboidSize.getX() &&
		Ydist == -pCuboidSize.getY())
	{
		return (-1.0f * pCuboidXAxis) - pCuboidYAxis;
	}
	else if (Xdist == -pCuboidSize.getX() &&
		Zdist == pCuboidSize.getZ())
	{
		return (-1.0f * pCuboidXAxis) + pCuboidZAxis;
	}
	else if (Xdist == -pCuboidSize.getX() &&
		Zdist == -pCuboidSize.getZ())
	{
		return (-1.0f * pCuboidXAxis) - pCuboidZAxis;
	}
	else if (Ydist == pCuboidSize.getY() &&
		Zdist == pCuboidSize.getZ())
	{
		return pCuboidYAxis + pCuboidZAxis;
	}
	else if (Ydist == pCuboidSize.getY() &&
		Zdist == -pCuboidSize.getZ())
	{
		return pCuboidYAxis - pCuboidZAxis;
	}
	else if (Ydist == -pCuboidSize.getY() &&
		Zdist == pCuboidSize.getZ())
	{
		return (-1.0f * pCuboidYAxis) + pCuboidZAxis;
	}
	else if (Ydist == -pCuboidSize.getY() &&
		Zdist == -pCuboidSize.getZ())
	{
		return (-1.0f * pCuboidYAxis) - pCuboidZAxis;
	}
	else if (Xdist == pCuboidSize.getX())
	{
		return pCuboidXAxis;
	}
	else if (Xdist == -pCuboidSize.getX())
	{
		return -1.0f * pCuboidXAxis;
	}
	else if (Ydist == pCuboidSize.getY())
	{
		return pCuboidYAxis;
	}
	else if (Ydist == -pCuboidSize.getY())
	{
		return -1.0f * pCuboidYAxis;
	}
	else if (Zdist == pCuboidSize.getZ())
	{
		return pCuboidZAxis;
	}
	else if (Zdist == -pCuboidSize.getZ())
	{
		return -1.0f * pCuboidZAxis;
	}

	//TODO: Check that this is almost never reached
	return Vector3F(0, 1, 0);
}
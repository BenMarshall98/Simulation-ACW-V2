#include "CollisionDetection.h"
#include "Game.h"
#include <corecrt_math_defines.h>

void CollisionDetection::dynamicCollisionDetection(RigidBody* pRigidBody1, RigidBody* pRigidBody2, ContactManifold* pManifold, const float pLastCollisionTime)
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

void CollisionDetection::detectCollisionSphereSphere(Sphere* pSphere1, Sphere* pSphere2, ContactManifold* pManifold, const float pLastCollisionTime)
{
	glm::vec3 sphere1Pos;
	glm::vec3 sphere2Pos;
	
	if (pSphere1->getCurrentUpdateTime() < pLastCollisionTime)
	{
		const auto interValue = (pLastCollisionTime - pSphere1->getCurrentUpdateTime()) / (1.0f - pSphere1->getCurrentUpdateTime());
		
		sphere1Pos = mix(pSphere1->getPos(), pSphere1->getNewPos(), interValue);
	}
	else
	{
		sphere1Pos = pSphere1->getPos();
	}

	if (pSphere2->getCurrentUpdateTime() < pLastCollisionTime)
	{
		const auto interValue = (pLastCollisionTime - pSphere2->getCurrentUpdateTime()) / (1.0f - pSphere2->getCurrentUpdateTime());

		sphere2Pos = mix(pSphere2->getPos(), pSphere2->getNewPos(), interValue);
	}
	else
	{
		sphere2Pos = pSphere2->getPos();
	}

	const auto relCenter = sphere2Pos - sphere1Pos;

	const auto velSphere1 = pSphere1->getNewPos() - sphere1Pos;
	const auto velSphere2 = pSphere2->getNewPos() - sphere2Pos;

	const auto relVelocity = velSphere2 - velSphere1;

	const auto radius = pSphere1->getSize().x + pSphere2->getSize().x;

	const auto c = dot(relCenter, relCenter) - radius * radius;

	if (c < 0.0f)
	{
		ManifoldPoint manPoint;
		manPoint.mContactId1 = pSphere1;
		manPoint.mContactId2 = pSphere2;
		manPoint.mContactNormal = normalize(sphere1Pos - sphere2Pos);
		manPoint.mTime = pLastCollisionTime;
		manPoint.mCollisionDepth = radius - (sphere1Pos - sphere2Pos).length();
		manPoint.mCollisionType = CollisionType::PENETRATION;

		pManifold->add(manPoint);
		return;
	}

	const auto a = dot(relVelocity, relVelocity);

	if (a == 0)
	{
		return; //Not moving relative to each other
	}

	const auto b = dot(relVelocity, relCenter);

	const auto discriminant = b * b - a * c;

	if (discriminant < 0)
	{
		return; //Spheres never inter-set
	}

	const auto time = (-b - sqrt(discriminant)) / a;

	if (time >= 0 && time <= 1)
	{
		const auto sphere1Point = sphere1Pos + time * velSphere1;
		const auto sphere2Point = sphere2Pos + time * velSphere2;

		ManifoldPoint manPoint;
		manPoint.mContactId1 = pSphere1;
		manPoint.mContactId2 = pSphere2;
		manPoint.mContactNormal = normalize(sphere1Point - sphere2Point);
		manPoint.mTime = pLastCollisionTime + time * (1.0f - pLastCollisionTime);
		manPoint.mCollisionType = CollisionType::COLLISION;

		pManifold->add(manPoint);
	}
}

void CollisionDetection::detectCollisionSphereBowl(Sphere* pSphere, Bowl* pBowl, ContactManifold* pManifold, const float pLastCollisionTime)
{
	glm::vec3 spherePos;

	if (pSphere->getCurrentUpdateTime() < pLastCollisionTime)
	{
		const auto interValue = (pLastCollisionTime - pSphere->getCurrentUpdateTime()) / (1.0f - pSphere->getCurrentUpdateTime());

		spherePos = pSphere->getPos() + interValue * (pSphere->getNewPos() - pSphere->getPos());
	}
	else
	{
		spherePos = pSphere->getPos();
	}

	const auto bowlMat = pBowl->getMatrix();

	const auto center = glm::vec3(glm::vec4(pBowl->getPos(), 0.0f) * bowlMat);

	const auto relCenter = spherePos - center;

	const auto velSphere = pSphere->getNewPos() - spherePos;

	const auto bowlRadius = pBowl->getSize().x;
	const auto sphereRadius = pSphere->getSize().x;
	const auto radius = bowlRadius - sphereRadius;

	auto c = dot(relCenter, relCenter) - radius * radius;

	if (c >= 0.0f)
	{
		ManifoldPoint manPoint;
		manPoint.mContactId1 = pSphere;
		manPoint.mContactId2 = pBowl;
		manPoint.mContactNormal = normalize(spherePos - center);
		manPoint.mTime = pLastCollisionTime;
		manPoint.mCollisionDepth = (spherePos - center).length() - radius;
		manPoint.mCollisionType = CollisionType::PENETRATION;

		pManifold->add(manPoint);
		return;
	}

	if (length(velSphere) == 0.0f)
	{
		return;
	}

	const auto d = velSphere * -1.0f / length(velSphere);
	const auto b = dot(relCenter, d);
	c = dot(relCenter, relCenter) - radius * radius;

	const auto discr = b * b - c;

	if (discr < 0.0f)
	{
		return;
	}

	const auto time = -b - sqrt(discr);

	const auto intersection = spherePos + time * d;

	if (abs(time) >= length(velSphere))
	{
		return;
	}

	const auto spherePoint = spherePos + abs(time) * velSphere;

	ManifoldPoint manPoint;
	manPoint.mContactId1 = pSphere;
	manPoint.mContactId2 = pBowl;
	manPoint.mContactNormal = normalize(spherePoint - intersection);
	manPoint.mTime = pLastCollisionTime + abs(time) * (1.0f - pLastCollisionTime);
	manPoint.mCollisionType = CollisionType::COLLISION;

	pManifold->add(manPoint);
}

void CollisionDetection::detectCollisionSpherePlane(Sphere* pSphere, Plane* pPlane, ContactManifold* pManifold, const float pLastCollisionTime)
{
	glm::vec3 spherePos;

	if (pSphere->getCurrentUpdateTime() < pLastCollisionTime)
	{
		const auto interValue = (pLastCollisionTime - pSphere->getCurrentUpdateTime()) / (1.0f - pSphere->getCurrentUpdateTime());

		spherePos = pSphere->getPos() + interValue * (pSphere->getNewPos() - pSphere->getPos());
	}
	else
	{
		spherePos = pSphere->getPos();
	}

	auto planeMat = pPlane->getMatrix();

	auto center = glm::vec3(glm::vec4(0, 0, 0, 0.0f) * planeMat);
	auto normal = glm::vec3(glm::vec4(0, 1, 0, 0.0f) * planeMat);
	auto tangent = glm::vec3(glm::vec4(1, 0, 0, 0.0f) * planeMat);
	auto biTangent = glm::vec3(glm::vec4(0, 0, 1, 0.0f) * planeMat);

	normal = normalize(normal - center);
	tangent = normalize(tangent - center);
	biTangent = normalize(biTangent - center);

	center = glm::vec3(glm::vec4(pPlane->getPos(), 0.0f) * planeMat);

	auto newPlaneMat = pPlane->getNewMatrix();

	auto newCenter = pPlane->getPos() * newPlaneMat;

	auto planeVelocity = newCenter - center;

	auto velocity = pSphere->getNewPos() - spherePos - planeVelocity;

	auto planeDot = normal.dot(center + tangent);

	auto dist = normal.dot(spherePos) - planeDot;

	auto radius = dist > 0.0f ? pSphere->getSize().x() : -pSphere->getSize().x();

	if (abs(dist) <= abs(radius))
	{
		auto spherePoint = spherePos;
		auto collisionPoint = spherePoint - radius * normal;

		auto sizeX = pPlane->getSize().x();
		auto sizeY = pPlane->getSize().z();

		auto dotX = (collisionPoint - center).dot(tangent);
		auto dotY = (collisionPoint - center).dot(biTangent);

		radius = pSphere->getSize().x();

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
		if (dotX <= sizeX + radius && dotX >= -sizeX - radius &&
			dotY <= sizeY + radius && dotY >= -sizeY - radius)
		{
			std::vector<glm::vec3> startPoints =
			{
				center + sizeX * tangent + sizeY * biTangent,
				center + sizeX * tangent - sizeY * biTangent,
				center - sizeX * tangent + sizeY * biTangent,
				center - sizeX * tangent - sizeY * biTangent
			};

			std::vector<glm::vec3> endPoints =
			{
				center + sizeX * tangent - sizeY * biTangent,
				center - sizeX * tangent - sizeY * biTangent,
				center + sizeX * tangent + sizeY * biTangent,
				center - sizeX * tangent + sizeY * biTangent
			};

			auto time = 0.0f;
			
			for (auto i = 0u; i < startPoints.size(); i++)
			{
				if (detectCollisionSphereLine(pSphere, startPoints[i], endPoints[i], planeVelocity, time, pLastCollisionTime))
				{
					spherePoint = spherePos + time * velocity;

					auto lineVector = endPoints[i] - startPoints[i];

					auto t = (spherePoint - startPoints[i]).dot(lineVector) / lineVector.dot(lineVector);

					auto closestPoint = startPoints[i] + t * lineVector;

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

			for (auto startPoint : startPoints)
			{
				if (detectCollisionSphereVertex(pSphere, startPoint, planeVelocity, time, pLastCollisionTime))
				{
					spherePoint = spherePos + time * velocity;

					auto closestPoint = startPoint;

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

	auto velDot = normal.dot(velocity);

	if (velDot * dist >= 0)
	{
		return; //Moving parallel no collision;
	}

	auto time = (radius - dist) / velDot;

	if (time >= 0 && time <= 1)
	{
		auto spherePoint = spherePos + time * velocity;
		auto collisionPoint = spherePoint - radius * normal;

		auto sizeX = pPlane->getSize().x();
		auto sizeY = pPlane->getSize().z();

		auto dotX = (collisionPoint - center + planeVelocity * time).dot(tangent);
		auto dotY = (collisionPoint - center + planeVelocity * time).dot(biTangent);

		radius = pSphere->getSize().x();

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
			std::vector<glm::vec3> startPoints =
			{
				center + sizeX * tangent + sizeY * biTangent,
				center + sizeX * tangent - sizeY * biTangent,
				center - sizeX * tangent + sizeY * biTangent,
				center - sizeX * tangent - sizeY * biTangent
			};

			std::vector<glm::vec3> endPoints =
			{
				center + sizeX * tangent - sizeY * biTangent,
				center - sizeX * tangent - sizeY * biTangent,
				center + sizeX * tangent + sizeY * biTangent,
				center - sizeX * tangent + sizeY * biTangent
			};

			for (auto i = 0u; i < startPoints.size(); i++)
			{
				if (detectCollisionSphereLine(pSphere, startPoints[i], endPoints[i], planeVelocity, time, pLastCollisionTime))
				{
					spherePoint = spherePos + time * velocity;

					auto lineVector = endPoints[i] - startPoints[i];

					auto t = (spherePoint - startPoints[i]).dot(lineVector) / lineVector.dot(lineVector);

					auto closestPoint = startPoints[i] + t * lineVector;

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

			for (auto startPoint : startPoints)
			{
				if (detectCollisionSphereVertex(pSphere, startPoint, planeVelocity, time, pLastCollisionTime))
				{
					spherePoint = spherePos + time * velocity;

					auto closestPoint = startPoint;

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

void CollisionDetection::detectCollisionSpherePlaneHoles(Sphere* pSphere, PlaneHoles* pPlaneHoles, ContactManifold* pManifold, const float pLastCollisionTime)
{
	glm::vec3 spherePos;

	if (pSphere->getCurrentUpdateTime() < pLastCollisionTime)
	{
		auto interValue = (pLastCollisionTime - pSphere->getCurrentUpdateTime()) / (1.0f - pSphere->getCurrentUpdateTime());

		spherePos = pSphere->getPos() + interValue * (pSphere->getNewPos() - pSphere->getPos());
	}
	else
	{
		spherePos = pSphere->getPos();
	}

	auto planeMat = pPlaneHoles->getMatrix();

	auto center = glm::vec3(0, 0, 0) * planeMat;
	auto normal = glm::vec3(0, 1, 0) * planeMat;
	auto tangent = glm::vec3(1, 0, 0) * planeMat;
	auto bitangent = glm::vec3(0, 0, 1) * planeMat;

	normal = (normal - center).normalize();
	tangent = (tangent - center).normalize();
	bitangent = (bitangent - center).normalize();

	center = pPlaneHoles->getPos() * planeMat;

	auto newPlaneMat = pPlaneHoles->getNewMatrix();

	auto newCenter = pPlaneHoles->getPos() * newPlaneMat;

	auto planeVelocity = newCenter - center;

	auto velocity = pSphere->getNewPos() - spherePos - planeVelocity;

	auto planeDot = normal.dot(center + tangent);

	auto dist = normal.dot(spherePos) - planeDot;

	auto radius = dist > 0.0f ? pSphere->getSize().x() : -pSphere->getSize().x();

	if (abs(dist) <= abs(radius))
	{
		auto spherePoint = spherePos;
		auto collisionPoint = spherePoint - radius * normal;

		auto sizeX = pPlaneHoles->getSize().x() * 5;
		auto sizeY = pPlaneHoles->getSize().z() * 5;

		auto dotX = (collisionPoint - center).dot(tangent);
		auto dotY = (collisionPoint - center).dot(bitangent);

		radius = pSphere->getSize().x();
		auto time = 0.0f;
		if (dotX <= sizeX && dotX >= -sizeX &&
			dotY <= sizeY && dotY >= -sizeY)
		{
			std::vector<glm::vec3> centers =
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

			for (auto i = 0; i < centers.size(); i++)
			{
				if (dotX <= centers[i].x() + 1 && dotX >= centers[i].x() - 1 &&
					dotY <= centers[i].z() + 1 && dotY >= centers[i].z() - 1)
				{
					auto insideCircle = false;
					auto segments = 20;
					const auto angle = static_cast<float>(M_PI * 2) / segments;

					for (auto j = 0; j < segments; j++)
					{
						auto tempCenter = center + centers[i];
						auto tempVertex1 = tempCenter + cos(j * angle) * tangent + sin(j * angle) * bitangent;
						auto tempVertex2 = tempCenter + cos((j + 1) * angle) * tangent + sin((j + 1) * angle) * bitangent;

						if (detectCollisionSphereTriangle(pSphere, tempCenter, tempVertex1, tempVertex2, planeVelocity, pLastCollisionTime))
						{
							insideCircle = true;

							if (detectCollisionSphereLine(pSphere, tempVertex1, tempVertex2, planeVelocity, time, pLastCollisionTime))
							{
								auto spherePoint = spherePos + time * velocity;

								auto lineVector = tempVertex2 - tempVertex1;

								auto t = (spherePoint - tempVertex1).dot(lineVector) / lineVector.dot(lineVector);

								auto closestPoint = tempVertex1 + t * lineVector;

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
		if (dotX <= sizeX + 2 * radius && dotX >= -sizeX - 2 * radius &&
			dotY <= sizeY + 2 * radius && dotY >= -sizeY - 2 * radius)
		{
			std::vector<glm::vec3> startPoints =
			{
				center + sizeX * tangent + sizeY * bitangent,
				center + sizeX * tangent - sizeY * bitangent,
				center - sizeX * tangent + sizeY * bitangent,
				center - sizeX * tangent - sizeY * bitangent
			};

			std::vector<glm::vec3> endPoints =
			{
				center + sizeX * tangent - sizeY * bitangent,
				center - sizeX * tangent - sizeY * bitangent,
				center + sizeX * tangent + sizeY * bitangent,
				center - sizeX * tangent + sizeY * bitangent
			};

			for (auto i = 0; i < startPoints.size(); i++)
			{
				if (detectCollisionSphereLine(pSphere, startPoints[i], endPoints[i], planeVelocity, time, pLastCollisionTime))
				{
					auto spherePoint = spherePos + time * velocity;

					auto lineVector = endPoints[i] - startPoints[i];

					auto t = (spherePoint - startPoints[i]).dot(lineVector) / lineVector.dot(lineVector);

					auto closestPoint = startPoints[i] + t * lineVector;

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

			for (auto i = 0; i < startPoints.size(); i++)
			{
				if (detectCollisionSphereVertex(pSphere, startPoints[i], planeVelocity, time, pLastCollisionTime))
				{
					spherePoint = spherePos + time * velocity;

					auto closestPoint = startPoints[i];

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

	auto velDot = normal.dot(velocity);

	if (velDot * dist >= 0)
	{
		return; //Moving parallel no collision;
	}

	auto time = (radius - dist) / velDot;

	if (time >= 0 && time <= 1)
	{
		auto spherePoint = spherePos + time * velocity;
		auto collisionPoint = spherePoint - radius * normal;

		auto sizeX = pPlaneHoles->getSize().x() * 5;
		auto sizeY = pPlaneHoles->getSize().z() * 5;

		auto dotX = (collisionPoint - center + planeVelocity * time).dot(tangent);
		auto dotY = (collisionPoint - center + planeVelocity * time).dot(bitangent);

		radius = pSphere->getSize().x();

		if (dotX <= sizeX && dotX >= -sizeX &&
			dotY <= sizeY && dotY >= -sizeY)
		{
			std::vector<glm::vec3> centers =
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

			for (auto& i : centers)
			{
				if (dotX <= i.x() + 1 && dotX >= i.x() - 1 &&
					dotY <= i.z() + 1 && dotY >= i.z() - 1)
				{
					auto insideCircle = false;
					auto segments = 20.0f;
					const auto angle = static_cast<float>(M_PI * 2) / segments;
					
					for (auto j = 0.0f; j < segments; j++)
					{
						auto tempCenter = i + center;
						auto tempVertex1 = tempCenter + cos(j * angle) * tangent + sin(j * angle) * bitangent;
						auto tempVertex2 = tempCenter + cos((j + 1.0f) * angle) * tangent + sin((j + 1.0f) * angle) * bitangent;

						if (detectCollisionSphereTriangle(pSphere, tempCenter, tempVertex1, tempVertex2, planeVelocity, pLastCollisionTime))
						{
							insideCircle = true;

							if(detectCollisionSphereLine(pSphere, tempVertex1, tempVertex2, planeVelocity, time, pLastCollisionTime))
							{
								spherePoint = spherePos + time * velocity;

								auto lineVector = tempVertex2 - tempVertex1;

								auto t = (spherePoint - tempVertex1).dot(lineVector) / lineVector.dot(lineVector);

								auto closestPoint = tempVertex1 + t * lineVector;

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
			std::vector<glm::vec3> startPoints =
			{
				center + sizeX * tangent + sizeY * bitangent,
				center + sizeX * tangent - sizeY * bitangent,
				center - sizeX * tangent + sizeY * bitangent,
				center - sizeX * tangent - sizeY * bitangent
			};

			std::vector<glm::vec3> endPoints =
			{
				center + sizeX * tangent - sizeY * bitangent,
				center - sizeX * tangent - sizeY * bitangent,
				center + sizeX * tangent + sizeY * bitangent,
				center - sizeX * tangent + sizeY * bitangent
			};

			for (auto i = 0u; i < startPoints.size(); i++)
			{
				if (detectCollisionSphereLine(pSphere, startPoints[i], endPoints[i], planeVelocity, time, pLastCollisionTime))
				{
					spherePoint = spherePos + time * velocity;

					auto lineVector = endPoints[i] - startPoints[i];

					auto t = (spherePoint - startPoints[i]).dot(lineVector) / lineVector.dot(lineVector);

					auto closestPoint = startPoints[i] + t * lineVector;

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

			for (auto startPoint : startPoints)
			{
				if (detectCollisionSphereVertex(pSphere, startPoint, planeVelocity, time, pLastCollisionTime))
				{
					spherePoint = spherePos + time * velocity;

					auto closestPoint = startPoint;

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

bool CollisionDetection::detectCollisionSphereLine(Sphere* pSphere, const glm::vec3 pLineEnd1, const glm::vec3 pLineEnd2, const glm::vec3 pLineVelocity, float& pTime, const float pLastCollisionTime)
{
	glm::vec3 spherePos;

	if (pSphere->getCurrentUpdateTime() < pLastCollisionTime)
	{
		const auto interValue = (pLastCollisionTime - pSphere->getCurrentUpdateTime()) / (1.0f - pSphere->getCurrentUpdateTime());

		spherePos = pSphere->getPos() + interValue * (pSphere->getNewPos() - pSphere->getPos());
	}
	else
	{
		spherePos = pSphere->getPos();
	}

	const auto d = pLineEnd2 - pLineEnd1;
	const auto n = pSphere->getNewPos()+ pLineVelocity - spherePos;
	const auto m = spherePos - pLineEnd1;
	const auto radius = pSphere->getSize().x();

	const auto md = m.dot(d);
	const auto nd = n.dot(d);
	const auto dd = d.dot(d);

	if (md < 0.0 && md + nd < 0.0)
	{
		return false; //Outside cylinder
	}

	if (md > dd && md + nd > dd)
	{
		return false; //Outside cylinder
	}

	const auto mn = m.dot(n);
	const auto nn = n.dot(n);

	const auto a = dd * nn - nd * nd;
	const auto k = m.dot(m) - radius * radius;
	const auto c = dd * k - md * md;

	if (a == 0)
	{
		if (c > 0.0f)
		{
			return false;
		}
	}

	const auto b = dd * mn - nd * md;
	const auto discr = b * b - a * c;

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

bool CollisionDetection::detectCollisionSphereVertex(Sphere* pSphere, glm::vec3 pVertex, glm::vec3 pVertexVelocity, float& pTime, const float pLastCollisionTime)
{
	glm::vec3 spherePos;

	if (pSphere->getCurrentUpdateTime() < pLastCollisionTime)
	{
		const auto interValue = (pLastCollisionTime - pSphere->getCurrentUpdateTime()) / (1.0f - pSphere->getCurrentUpdateTime());

		spherePos = pSphere->getPos() + interValue * (pSphere->getNewPos() - pSphere->getPos());
	}
	else
	{
		spherePos = pSphere->getPos();
	}

	const auto sphereStart = spherePos;
	const auto sphereEnd = pSphere->getNewPos() - pVertexVelocity;
	const auto radius = pSphere->getSize().x();

	const auto d = sphereEnd - sphereStart;

	const auto m = sphereStart - pVertex;

	const auto a = d.dot(d);
	const auto b = m.dot(d);
	const auto c = m.dot(m) - radius * radius;

	if (c > 0.0f && b > 0.0f)
	{
		return false;
	}

	const auto discr = b * b - a * c;

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

bool CollisionDetection::detectCollisionSphereTriangle(Sphere* pSphere, glm::vec3 pVertex1, glm::vec3 pVertex2, glm::vec3 pVertex3, glm::vec3 pTriangleVelocity, const float pLastCollisionTime)
{
	glm::vec3 spherePos;

	if (pSphere->getCurrentUpdateTime() < pLastCollisionTime)
	{
		auto interValue = (pLastCollisionTime - pSphere->getCurrentUpdateTime()) / (1.0f - pSphere->getCurrentUpdateTime());

		spherePos = pSphere->getPos() + interValue * (pSphere->getNewPos() - pSphere->getPos());
	}
	else
	{
		spherePos = pSphere->getPos();
	}

	auto velocity = pSphere->getNewPos() - spherePos - pTriangleVelocity;
	auto radius = pSphere->getSize().x();

	auto planeNormal = (pVertex2 - pVertex1).cross(pVertex3 - pVertex1).normalize();
	auto planeDot = planeNormal.dot(pVertex2);

	auto dist = planeNormal.dot(spherePos) - planeDot;

	if (abs(dist) <= radius)
	{
		auto collisionPoint = spherePos - (-radius + radius - abs(dist)) * planeNormal;

		auto tempVertex1 = pVertex1 - collisionPoint;
		auto tempVertex2 = pVertex2 - collisionPoint;
		auto tempVertex3 = pVertex3 - collisionPoint;

		auto u = tempVertex2.cross(tempVertex3);
		auto v = tempVertex3.cross(tempVertex1);

		if (u.dot(v) < 0.0f)
		{
			return false;
		}

		auto w = tempVertex1.cross(tempVertex2);

		if (u.dot(w) < 0.0f)
		{
			return false;
		}

		return true;
	}

	auto denom = planeNormal.dot(velocity);

	if (denom * dist >= 0.0f)
	{
		return false;
	}

	radius = dist > 0.0f ? radius : -radius;

	const auto time = (radius - dist) / denom;

	const auto spherePoint = spherePos + time * velocity;
	const auto collisionPoint = spherePoint - radius * planeNormal;

	auto tempVertex1 = pVertex1 - collisionPoint;
	auto tempVertex2 = pVertex2 - collisionPoint;
	auto tempVertex3 = pVertex3 - collisionPoint;

	auto u = tempVertex2.cross(tempVertex3);
	auto v = tempVertex3.cross(tempVertex1);

	if (u.dot(v) < 0.0f)
	{
		return false;
	}

	auto w = tempVertex1.cross(tempVertex2);

	if (u.dot(w) < 0.0f)
	{
		return false;
	}

	return true;
}

void CollisionDetection::detectCollisionSphereCuboid(Sphere * pSphere, Cuboid * pCuboid, ContactManifold * pManifold, const float pLastCollisionTime)
{
	glm::vec3 spherePos;

	if (pSphere->getCurrentUpdateTime() < pLastCollisionTime)
	{
		const auto interValue = (pLastCollisionTime - pSphere->getCurrentUpdateTime()) / (1.0f - pSphere->getCurrentUpdateTime());

		spherePos = pSphere->getPos().interpolate(pSphere->getNewPos(), interValue);
	}
	else
	{
		spherePos = pSphere->getPos();
	}

	glm::vec3 cuboidPos;
	glm::quat cuboidOrientation;

	if (pCuboid->getCurrentUpdateTime() < pLastCollisionTime)
	{
		const auto interValue = (pLastCollisionTime - pCuboid->getCurrentUpdateTime()) / (1.0f - pCuboid->getCurrentUpdateTime());

		cuboidPos = pCuboid->getPos().interpolate(pCuboid->getNewPos(), interValue);
		cuboidOrientation = slerp(pCuboid->getOrientation(), pCuboid->getNewOrientation(), interValue);
	}
	else
	{
		cuboidPos = pCuboid->getPos();
		cuboidOrientation = pCuboid->getOrientation();
	}

	const auto sphereRadius = pSphere->getSize().x();
	const auto cuboidSize = pCuboid->getSize();

	const auto rot = toMat4(cuboidOrientation);
	const auto rotation = glm::mat4(rot[0][0], rot[0][1], rot[0][2], rot[0][3],
		rot[1][0], rot[1][1], rot[1][2], rot[1][3],
		rot[2][0], rot[2][1], rot[2][2], rot[2][3],
		rot[3][0], rot[3][1], rot[3][2], rot[3][3]);

	const auto cuboidXAxis = glm::vec3(1, 0, 0) * rotation;
	const auto cuboidYAxis = glm::vec3(0, 1, 0) * rotation;
	const auto cuboidZAxis = glm::vec3(0, 0, 1) * rotation;

	glm::vec3 closestPoint;
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

bool CollisionDetection::detectCollisionSphereCuboidStep(const glm::vec3 pSphereCenter, const float pSphereRadius, const glm::vec3 pCuboidCenter,
                                                         const glm::vec3 pCuboidXAxis, const glm::vec3 pCuboidYAxis, const glm::vec3 pCuboidZAxis, const glm::vec3 pCuboidSize, glm::vec3 & pPoint)
{
	//Get closest point on cuboid from sphere center
	const auto d = pSphereCenter - pCuboidCenter;

	pPoint = pCuboidCenter;

	{
		auto dist = d.dot(pCuboidXAxis);

		if (dist > pCuboidSize.x())
		{
			dist = pCuboidSize.x();
		}
		else if (dist < -pCuboidSize.x())
		{
			dist = -pCuboidSize.x();
		}

		pPoint = pPoint + dist * pCuboidXAxis;
	}

	{
		auto dist = d.dot(pCuboidYAxis);

		if (dist > pCuboidSize.y())
		{
			dist = pCuboidSize.y();
		}
		else if (dist < -pCuboidSize.y())
		{
			dist = -pCuboidSize.y();
		}

		pPoint = pPoint + dist * pCuboidYAxis;
	}

	{
		auto dist = d.dot(pCuboidZAxis);

		if (dist > pCuboidSize.z())
		{
			dist = pCuboidSize.z();
		}
		else if (dist < -pCuboidSize.z())
		{
			dist = -pCuboidSize.z();
		}

		pPoint = pPoint + dist * pCuboidZAxis;
	}

	//Check if point on cuboid is touching sphere, therefore colliding

	const auto dist = pPoint - pSphereCenter;

	return dist.dot(dist) <= pSphereRadius * pSphereRadius;
}

glm::vec3 CollisionDetection::calculateCuboidCollisionNormal(const glm::vec3 pCuboidCenter, const glm::vec3 pCuboidXAxis, const glm::vec3 pCuboidYAxis, const glm::vec3 pCuboidZAxis, const glm::vec3 pCuboidSize, const glm::vec3 pPoint)
{
	const auto d = pPoint - pCuboidCenter;

	const auto xDist = d.dot(pCuboidXAxis);
	const auto yDist = d.dot(pCuboidYAxis);
	const auto zDist = d.dot(pCuboidZAxis);

	if (xDist == pCuboidSize.x() &&
		yDist == pCuboidSize.y() &&
		zDist == pCuboidSize.z())
	{
		return pCuboidXAxis + pCuboidYAxis + pCuboidZAxis;
	}
	if (xDist == pCuboidSize.x() &&
		yDist == pCuboidSize.y() &&
		zDist == -pCuboidSize.z())
	{
		return pCuboidXAxis + pCuboidYAxis - pCuboidZAxis;
	}
	if (xDist == pCuboidSize.x() &&
		yDist == -pCuboidSize.y() &&
		zDist == pCuboidSize.z())
	{
		return pCuboidXAxis - pCuboidYAxis + pCuboidZAxis;
	}
	if (xDist == pCuboidSize.x() &&
		yDist == -pCuboidSize.y() &&
		zDist == -pCuboidSize.z())
	{
		return pCuboidXAxis - pCuboidYAxis - pCuboidZAxis;
	}
	if (xDist == -pCuboidSize.x() &&
		yDist == pCuboidSize.y() &&
		zDist == pCuboidSize.z())
	{
		return -1 * pCuboidXAxis + pCuboidYAxis + pCuboidZAxis;
	}
	if (xDist == -pCuboidSize.x() &&
		yDist == pCuboidSize.y() &&
		zDist == -pCuboidSize.z())
	{
		return -1 * pCuboidXAxis + pCuboidYAxis - pCuboidZAxis;
	}
	if (xDist == -pCuboidSize.x() &&
		yDist == -pCuboidSize.y() &&
		zDist == pCuboidSize.z())
	{
		return -1 * pCuboidXAxis - pCuboidYAxis + pCuboidZAxis;
	}
	if (xDist == -pCuboidSize.x() &&
		yDist == -pCuboidSize.y() &&
		zDist == -pCuboidSize.z())
	{
		return -1 * pCuboidXAxis - pCuboidYAxis - pCuboidZAxis;
	}
	if (xDist == pCuboidSize.x() &&
		yDist == pCuboidSize.y())
	{
		return pCuboidXAxis + pCuboidYAxis;
	}
	if (xDist == pCuboidSize.x() &&
		yDist == -pCuboidSize.y())
	{
		return pCuboidXAxis - pCuboidYAxis;
	}
	if (xDist == pCuboidSize.x() &&
		zDist == pCuboidSize.z())
	{
		return pCuboidXAxis + pCuboidZAxis;
	}
	if (xDist == pCuboidSize.x() &&
		zDist == -pCuboidSize.z())
	{
		return pCuboidXAxis - pCuboidZAxis;
	}
	if (xDist == -pCuboidSize.x() &&
		yDist == pCuboidSize.y())
	{
		return -1.0f * pCuboidXAxis + pCuboidYAxis;
	}
	if (xDist == -pCuboidSize.x() &&
		yDist == -pCuboidSize.y())
	{
		return -1.0f * pCuboidXAxis - pCuboidYAxis;
	}
	if (xDist == -pCuboidSize.x() &&
		zDist == pCuboidSize.z())
	{
		return -1.0f * pCuboidXAxis + pCuboidZAxis;
	}
	if (xDist == -pCuboidSize.x() &&
		zDist == -pCuboidSize.z())
	{
		return -1.0f * pCuboidXAxis - pCuboidZAxis;
	}
	if (yDist == pCuboidSize.y() &&
		zDist == pCuboidSize.z())
	{
		return pCuboidYAxis + pCuboidZAxis;
	}
	if (yDist == pCuboidSize.y() &&
		zDist == -pCuboidSize.z())
	{
		return pCuboidYAxis - pCuboidZAxis;
	}
	if (yDist == -pCuboidSize.y() &&
		zDist == pCuboidSize.z())
	{
		return -1.0f * pCuboidYAxis + pCuboidZAxis;
	}
	if (yDist == -pCuboidSize.y() &&
		zDist == -pCuboidSize.z())
	{
		return -1.0f * pCuboidYAxis - pCuboidZAxis;
	}
	if (xDist == pCuboidSize.x())
	{
		return pCuboidXAxis;
	}
	if (xDist == -pCuboidSize.x())
	{
		return -1.0f * pCuboidXAxis;
	}
	if (yDist == pCuboidSize.y())
	{
		return pCuboidYAxis;
	}
	if (yDist == -pCuboidSize.y())
	{
		return -1.0f * pCuboidYAxis;
	}
	if (zDist == pCuboidSize.z())
	{
		return pCuboidZAxis;
	}
	if (zDist == -pCuboidSize.z())
	{
		return -1.0f * pCuboidZAxis;
	}

	//TODO: Check that this is almost never reached
	return glm::vec3(0, 1, 0);
}

bool CollisionDetection::detectCollisionCuboidCuboidStep(glm::vec3 pCuboid1Center, glm::vec3 pCuboidXAxis1, glm::vec3 pCuboidYAxis1, glm::vec3 pCuboidZAxis1, glm::vec3 pCuboidSize1, glm::vec3 pCuboid2Center, glm::vec3 pCuboidXAxis2, glm::vec3 pCuboidYAxis2, glm::vec3 pCuboidZAxis2, glm::vec3 pCuboidSize2)
{
	float ra, rb;
	glm::mat3 rot, absRot;

	rot[0][0] = pCuboidXAxis1.dot(pCuboidXAxis2);
	rot[0][1] = pCuboidXAxis1.dot(pCuboidYAxis2);
	rot[0][2] = pCuboidXAxis1.dot(pCuboidZAxis2);
	rot[1][0] = pCuboidYAxis1.dot(pCuboidXAxis2);
	rot[1][1] = pCuboidYAxis1.dot(pCuboidYAxis2);
	rot[1][2] = pCuboidYAxis1.dot(pCuboidZAxis2);
	rot[2][0] = pCuboidZAxis1.dot(pCuboidXAxis2);
	rot[2][1] = pCuboidZAxis1.dot(pCuboidYAxis2);
	rot[2][2] = pCuboidZAxis1.dot(pCuboidZAxis2);

	auto t = pCuboid2Center - pCuboid1Center;

	t = glm::vec3(t.dot(pCuboidXAxis1), t.dot(pCuboidZAxis1), t.dot(pCuboidZAxis1));

	for (auto i = 0u; i < 3; i++)
	{
		for (auto j = 0u; j < 3; j++)
		{
			absRot[i][j] = abs(rot[i][j]) + 0.00001f;
		}
	}

	{
		ra = pCuboidSize1.x();
		rb = pCuboidSize2.x() * absRot[0][0] + pCuboidSize2.y() * absRot[0][1] + pCuboidSize2.z() * absRot[0][2];

		if (abs(t.x()) > ra + rb)
		{
			return false;
		}
	}

	{
		ra = pCuboidSize1.y();
		rb = pCuboidSize2.x() * absRot[1][0] + pCuboidSize2.y() * absRot[1][1] + pCuboidSize2.z() * absRot[1][2];

		if (abs(t.y()) > ra + rb)
		{
			return false;
		}
	}

	{
		ra = pCuboidSize1.z();
		rb = pCuboidSize2.x() * absRot[2][0] + pCuboidSize2.y() * absRot[2][1] + pCuboidSize2.z() * absRot[2][2];

		if (abs(t.z()) > ra + rb)
		{
			return false;
		}
	}

	{
		ra = pCuboidSize1.x() * absRot[0][0] + pCuboidSize1.y() * absRot[1][0] + pCuboidSize1.z() * absRot[2][0];
		rb = pCuboidSize2.x();
		const auto temp = t.x() * rot[0][0] + t.y() * rot[1][0] + t.z() * rot[2][0];
		if (abs(temp) > ra + rb)
		{
			return false;
		}
	}

	{
		ra = pCuboidSize1.x() * absRot[0][1] + pCuboidSize1.y() * absRot[1][1] + pCuboidSize1.z() * absRot[2][1];
		rb = pCuboidSize2.y();
		const auto temp = t.x() * rot[0][1] + t.y() * rot[1][1] + t.z() * rot[2][1];

		if (abs(temp) > ra + rb)
		{
			return false;
		}
	}

	{
		ra = pCuboidSize1.x() * absRot[0][2] + pCuboidSize1.y() * absRot[1][2] + pCuboidSize1.z() * absRot[2][2];
		rb = pCuboidSize2.z();
		const auto temp = t.x() * rot[0][2] + t.y() * rot[1][2] + t.z() * rot[2][2];

		if (abs(temp) > ra + rb)
		{
			return false;
		}
	}

	{
		ra = pCuboidSize1.y() * absRot[2][0] + pCuboidSize1.z() * absRot[1][0];
		rb = pCuboidSize2.y() * absRot[0][2] + pCuboidSize2.z() * absRot[0][1];
		const auto temp = t.z() * rot[1][0] - t.y() * rot[2][0];

		if (abs(temp) > ra + rb)
		{
			return false;
		}
	}

	{
		ra = pCuboidSize1.y() * absRot[2][1] + pCuboidSize1.z() * absRot[1][1];
		rb = pCuboidSize2.x() * absRot[0][2] + pCuboidSize2.z() * absRot[0][0];
		const auto temp = t.z() * rot[1][1] - t.y() * rot[2][1];

		if (abs(temp) > ra + rb)
		{
			return false;
		}
	}

	{
		ra = pCuboidSize1.y() * absRot[2][2] + pCuboidSize2.z() * absRot[1][2];
		rb = pCuboidSize2.x() * absRot[0][1] + pCuboidSize2.y() * absRot[0][0];
		const auto temp = t.z() * rot[1][2] - t.y() * rot[2][2];

		if (abs(temp) > ra + rb)
		{
			return false;
		}
	}

	{
		ra = pCuboidSize1.x() * absRot[2][0] + pCuboidSize1.z() * absRot[0][0];
		rb = pCuboidSize2.y() * absRot[1][2] + pCuboidSize2.z() * absRot[1][1];
		const auto temp = t.x() * rot[2][0] - t.z() * rot[0][0];

		if (abs(temp) > ra + rb)
		{
			return false;
		}
	}

	{
		ra = pCuboidSize1.x() * absRot[2][1] + pCuboidSize1.z() * absRot[0][1];
		rb = pCuboidSize2.x() * absRot[1][2] + pCuboidSize2.z() * absRot[1][1];
		const auto temp = t.x() * rot[2][1] - t.z() * rot[0][1];

		if (abs(temp) > ra + rb)
		{
			return false;
		}
	}

	{
		ra = pCuboidSize1.x() * absRot[2][2] + pCuboidSize1.z() * absRot[0][2];
		rb = pCuboidSize2.x() * absRot[1][1] + pCuboidSize2.y() * absRot[1][0];
		const auto temp = t.x() * rot[2][2] - t.z() * rot[0][2];

		if (abs(temp) > ra + rb)
		{
			return false;
		}
	}

	{
		ra = pCuboidSize1.x() * absRot[1][0] + pCuboidSize1.y() * absRot[0][0];
		rb = pCuboidSize2.y() * absRot[2][2] + pCuboidSize2.z() * absRot[2][0];
		const auto temp = t.y() * rot[0][0] - t.x() * rot[1][0];

		if (abs(temp) > ra + rb)
		{
			return false;
		}
	}

	{
		ra = pCuboidSize1.x() * absRot[1][1] + pCuboidSize1.y() * absRot[0][1];
		rb = pCuboidSize2.x() * absRot[2][2] + pCuboidSize2.z() * absRot[2][0];
		const auto temp = t.y() * rot[0][1] - t.x() * rot[1][1];

		if (abs(temp) > ra + rb)
		{
			return false;
		}
	}

	{
		ra = pCuboidSize1.x() * absRot[1][2] + pCuboidSize1.y() * absRot[0][2];
		rb = pCuboidSize2.x() * absRot[2][1] + pCuboidSize2.y() * absRot[2][0];
		const auto temp = t.y() * rot[0][2] - t.x() * rot[1][2];

		if (abs(temp) > ra + rb)
		{
			return false;
		}
	}

	return true;
}

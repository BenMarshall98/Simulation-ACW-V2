#include "CollisionDetection.h"
#include "Game.h"
#include <corecrt_math_defines.h>

void CollisionDetection::dynamicCollisionDetection(RigidBody* pRigidBody1, RigidBody* pRigidBody2, ContactManifold* pManifold, const float pLastCollisionTime)
{
	if (pRigidBody1->getObjectType() == ObjectType::SPHERE && pRigidBody2->getObjectType() == ObjectType::SPHERE)
	{
		detectCollisionSphereSphere(pRigidBody1, pRigidBody2, pManifold, pLastCollisionTime);
	}
	else if (pRigidBody1->getObjectType() == ObjectType::SPHERE && pRigidBody2->getObjectType() == ObjectType::PLANE)
	{
		detectCollisionSpherePlane(pRigidBody1, pRigidBody2, pManifold, pLastCollisionTime);
	}
	else if (pRigidBody1->getObjectType() == ObjectType::PLANE && pRigidBody2->getObjectType() == ObjectType::SPHERE)
	{
		detectCollisionSpherePlane(pRigidBody2, pRigidBody1, pManifold, pLastCollisionTime);
	}
	else if (pRigidBody1->getObjectType() == ObjectType::SPHERE && pRigidBody2->getObjectType() == ObjectType::PLANEHOLES)
	{
		detectCollisionSpherePlaneHoles(pRigidBody1, pRigidBody2, pManifold, pLastCollisionTime);
	}
	else if (pRigidBody1->getObjectType() == ObjectType::PLANEHOLES && pRigidBody2->getObjectType() == ObjectType::SPHERE)
	{
		detectCollisionSpherePlaneHoles(pRigidBody2, pRigidBody1, pManifold, pLastCollisionTime);
	}
	else if (pRigidBody1->getObjectType() == ObjectType::SPHERE && pRigidBody2->getObjectType() == ObjectType::BOWL)
	{
		detectCollisionSphereBowl(pRigidBody1, pRigidBody2, pManifold, pLastCollisionTime);
	}
	else if (pRigidBody1->getObjectType() == ObjectType::BOWL && pRigidBody2->getObjectType() == ObjectType::SPHERE)
	{
		detectCollisionSphereBowl(pRigidBody2, pRigidBody1, pManifold, pLastCollisionTime);
	}
	else if (pRigidBody1->getObjectType() == ObjectType::SPHERE && pRigidBody2->getObjectType() == ObjectType::CYLINDER)
	{
		detectCollisionSphereCylinder(pRigidBody1, pRigidBody2, pManifold, pLastCollisionTime);
	}
	else if (pRigidBody1->getObjectType() == ObjectType::CYLINDER && pRigidBody2->getObjectType() == ObjectType::SPHERE)
	{
		detectCollisionSphereCylinder(pRigidBody2, pRigidBody1, pManifold, pLastCollisionTime);
	}
	else if (pRigidBody1->getObjectType() == ObjectType::SPHERE && pRigidBody2->getObjectType() == ObjectType::CUBOID)
	{
		detectCollisionSphereCuboid(pRigidBody1, pRigidBody2, pManifold, pLastCollisionTime);
	}
	else if (pRigidBody1->getObjectType() == ObjectType::CUBOID && pRigidBody2->getObjectType() == ObjectType::SPHERE)
	{
		detectCollisionSphereCuboid(pRigidBody2, pRigidBody1, pManifold, pLastCollisionTime);
	}
	else if (pRigidBody1->getObjectType() == ObjectType::CUBOID && pRigidBody2->getObjectType() == ObjectType::CUBOID)
	{
		detectCollisionCuboidCuboid(pRigidBody1, pRigidBody2, pManifold, pLastCollisionTime);
	}
	else if (pRigidBody1->getObjectType() == ObjectType::CUBOID && pRigidBody2->getObjectType() == ObjectType::PLANE)
	{
		detectCollisionCuboidPlane(pRigidBody1, pRigidBody2, pManifold, pLastCollisionTime);
	}
	else if (pRigidBody1->getObjectType() == ObjectType::PLANE && pRigidBody2->getObjectType() == ObjectType::CUBOID)
	{
		detectCollisionCuboidPlane(pRigidBody2, pRigidBody1, pManifold, pLastCollisionTime);
	}
	else if (pRigidBody1->getObjectType() == ObjectType::CUBOID && pRigidBody2->getObjectType() == ObjectType::PLANEHOLES)
	{
		detectCollisionCuboidPlaneHoles(pRigidBody1, pRigidBody2, pManifold, pLastCollisionTime);
	}
	else if (pRigidBody1->getObjectType() == ObjectType::PLANEHOLES && pRigidBody2->getObjectType() == ObjectType::CUBOID)
	{
		detectCollisionCuboidPlaneHoles(pRigidBody2, pRigidBody1, pManifold, pLastCollisionTime);
	}
	else if (pRigidBody1->getObjectType() == ObjectType::CUBOID && pRigidBody2->getObjectType() == ObjectType::BOWL)
	{
		detectCollisionCuboidBowl(pRigidBody1, pRigidBody2, pManifold, pLastCollisionTime);
	}
	else if (pRigidBody1->getObjectType() == ObjectType::BOWL && pRigidBody2->getObjectType() == ObjectType::CUBOID)
	{
		detectCollisionCuboidBowl(pRigidBody2, pRigidBody1, pManifold, pLastCollisionTime);
	}
	else if (pRigidBody1->getObjectType() == ObjectType::CUBOID && pRigidBody2->getObjectType() == ObjectType::CYLINDER)
	{
		detectCollisionCuboidCylinder(pRigidBody1, pRigidBody2, pManifold, pLastCollisionTime);
	}
	else if (pRigidBody1->getObjectType() == ObjectType::CYLINDER && pRigidBody2->getObjectType() == ObjectType::CUBOID)
	{
		detectCollisionCuboidCylinder(pRigidBody2, pRigidBody1, pManifold, pLastCollisionTime);
	}
}

void CollisionDetection::detectCollisionSphereSphere(RigidBody * pSphere1, RigidBody * pSphere2, ContactManifold* pManifold, const float pLastCollisionTime)
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
		const auto contactNormal = normalize(sphere1Pos - sphere2Pos);
		const auto contactPoint1 = sphere1Pos - contactNormal * pSphere1->getSize().x;
		const auto contactPoint2 = sphere2Pos + contactNormal * pSphere2->getSize().x;
		
		ManifoldPoint manPoint = {
			pSphere1,
			pSphere2,
			contactPoint1,
			contactPoint2,
			contactNormal,
			pLastCollisionTime,
			radius - length(sphere1Pos - sphere2Pos),
			CollisionType::PENETRATION
		};

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

		const auto contactNormal = normalize(sphere1Point - sphere2Point);
		const auto contactPoint1 = sphere1Point - contactNormal * pSphere1->getSize().x;
		const auto contactPoint2 = sphere2Point + contactNormal * pSphere2->getSize().y;

		ManifoldPoint manPoint = {
			pSphere1,
			pSphere2,
			contactPoint1,
			contactPoint2,
			contactNormal,
			pLastCollisionTime + time * (1.0f - pLastCollisionTime),
			0.0f,
			CollisionType::COLLISION,
		};
		
		pManifold->add(manPoint);
	}
}

void CollisionDetection::detectCollisionSphereBowl(RigidBody * pSphere, RigidBody * pBowl, ContactManifold* pManifold, const float pLastCollisionTime)
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

	const auto center = glm::vec3(bowlMat * glm::vec4(pBowl->getPos(), 1.0f));
	const auto relCenter = spherePos - center;

	const auto velSphere = pSphere->getNewPos() - spherePos;

	const auto bowlRadius = pBowl->getSize().x;
	const auto sphereRadius = pSphere->getSize().x;
	const auto radius = bowlRadius - sphereRadius;

	auto c = dot(relCenter, relCenter) - radius * radius;

	if (c >= 0.0f)
	{
		const auto contactNormal = normalize(center - spherePos);
		const auto contactPoint = spherePos - contactNormal * pSphere->getSize().x;
		
		ManifoldPoint manPoint = {
			pSphere,
			pBowl,
			contactPoint,
			contactPoint,
			contactNormal,
			pLastCollisionTime,
			sphereRadius - length(spherePos - contactPoint),
			CollisionType::PENETRATION,
		};
		
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

	const auto contactNormal = normalize(spherePoint - intersection);

	ManifoldPoint manPoint = {
		pSphere,
		pBowl,
		intersection,
		intersection,
		contactNormal,
		pLastCollisionTime + abs(time) * (1.0f - pLastCollisionTime),
		0.0f,
		CollisionType::COLLISION
	};

	pManifold->add(manPoint);
}

void CollisionDetection::detectCollisionSpherePlane(RigidBody * pSphere, RigidBody * pPlane, ContactManifold* pManifold, const float pLastCollisionTime)
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

	auto center = glm::vec3(planeMat * glm::vec4(0, 0, 0, 1.0f));
	auto normal = glm::vec3(planeMat * glm::vec4(0, 1, 0, 1.0f));
	auto tangent = glm::vec3(planeMat * glm::vec4(1, 0, 0, 1.0f));
	auto biTangent = glm::vec3(planeMat * glm::vec4(0, 0, 1, 1.0f));

	normal = normalize(normal - center);
	tangent = normalize(tangent - center);
	biTangent = normalize(biTangent - center);

	center = glm::vec3(planeMat * glm::vec4(pPlane->getPos(), 1.0f));

	auto newPlaneMat = pPlane->getNewMatrix();

	auto newCenter = glm::vec3(newPlaneMat * glm::vec4(pPlane->getPos(), 1.0f));

	auto planeVelocity = newCenter - center;

	auto velocity = pSphere->getNewPos() - spherePos - planeVelocity;

	auto planeDot = dot(normal, center + tangent);

	auto dist = dot(normal, spherePos) - planeDot;

	auto radius = dist > 0.0f ? pSphere->getSize().x : -pSphere->getSize().x;

	if (abs(dist) <= abs(radius))
	{
		auto spherePoint = spherePos;
		auto collisionPoint = spherePoint - radius * normal;

		auto sizeX = pPlane->getSize().x;
		auto sizeY = pPlane->getSize().z;

		auto dotX = dot(collisionPoint - center, tangent);
		auto dotY = dot(collisionPoint - center, biTangent);

		radius = pSphere->getSize().x;

		if (dotX <= sizeX && dotX >= -sizeX &&
			dotY <= sizeY && dotY >= -sizeY)
		{
			ManifoldPoint manPoint = {
				pSphere,
				pPlane,
				collisionPoint,
				collisionPoint,
				normalize(normal),
				pLastCollisionTime,
				abs(radius) - abs(dist),
				CollisionType::PENETRATION
			};

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

					auto t = dot(spherePoint - startPoints[i], lineVector) / dot(lineVector, lineVector);

					auto closestPoint = startPoints[i] + t * lineVector;

					ManifoldPoint manPoint = {
						pSphere,
						pPlane,
						closestPoint,
						closestPoint,
						normalize(normal),
						pLastCollisionTime + time * (1.0f - pLastCollisionTime),
						0.0f,
						CollisionType::COLLISION
					};

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

					ManifoldPoint manPoint = {
						pSphere,
						pPlane,
						closestPoint,
						closestPoint,
						normalize(normal),
						pLastCollisionTime + time * (1.0f - pLastCollisionTime),
						0.0f,
						CollisionType::PENETRATION
					};
					
					pManifold->add(manPoint);
					return;
				}
			}
		}
	}

	auto velDot = dot(normal, velocity);

	if (velDot * dist >= 0)
	{
		return; //Moving parallel no collision;
	}

	auto time = (radius - dist) / velDot;

	if (time >= 0 && time <= 1)
	{
		auto spherePoint = spherePos + time * velocity;
		auto collisionPoint = spherePoint - radius * normal;

		auto sizeX = pPlane->getSize().x;
		auto sizeY = pPlane->getSize().z;

		auto dotX = dot(collisionPoint - center + planeVelocity * time, tangent);
		auto dotY = dot(collisionPoint - center + planeVelocity * time, biTangent);

		radius = pSphere->getSize().x;

		if (dotX <= sizeX && dotX >= -sizeX &&
			dotY <= sizeY && dotY >= -sizeY)
		{
			ManifoldPoint manPoint = {
				pSphere,
				pPlane,
				collisionPoint,
				collisionPoint,
				normalize(normal),
				pLastCollisionTime + time * (1.0f - pLastCollisionTime),
				0.0f,
				CollisionType::COLLISION
			};

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

					auto t = dot(spherePoint - startPoints[i], lineVector) / dot(lineVector, lineVector);

					auto closestPoint = startPoints[i] + t * lineVector;

					ManifoldPoint manPoint = {
						pSphere,
						pPlane,
						closestPoint,
						closestPoint,
						normalize(normal),
						pLastCollisionTime + time * (1.0f - pLastCollisionTime),
						0.0f,
						manPoint.mCollisionType = CollisionType::COLLISION
					};
					
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

					ManifoldPoint manPoint = {
						pSphere,
						pPlane,
						closestPoint,
						closestPoint,
						normalize(normal),
						pLastCollisionTime + time * (1.0f - pLastCollisionTime),
						0.0f,
						CollisionType::PENETRATION,
					};

					pManifold->add(manPoint);
					return;
				}
			}
		}
	}
}

void CollisionDetection::detectCollisionSpherePlaneHoles(RigidBody * pSphere, RigidBody * pPlaneHoles, ContactManifold* pManifold, const float pLastCollisionTime)
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

	auto center = glm::vec3(planeMat * glm::vec4(0, 0, 0, 1.0f));
	auto normal = glm::vec3(planeMat * glm::vec4(0, 1, 0, 1.0f));
	auto tangent = glm::vec3(planeMat * glm::vec4(1, 0, 0, 1.0f));
	auto bitangent = glm::vec3(planeMat * glm::vec4(0, 0, 1, 1.0f));

	normal = normalize(normal - center);
	tangent = normalize(tangent - center);
	bitangent = normalize(bitangent - center);

	center = glm::vec3(planeMat * glm::vec4(pPlaneHoles->getPos(), 1.0f));

	auto newPlaneMat = pPlaneHoles->getNewMatrix();

	auto newCenter = glm::vec3(newPlaneMat * glm::vec4(pPlaneHoles->getPos(), 1.0f));

	auto planeVelocity = newCenter - center;

	auto velocity = pSphere->getNewPos() - spherePos - planeVelocity;

	auto planeDot = dot(normal, center + tangent);

	auto dist = dot(normal, spherePos) - planeDot;

	auto radius = dist > 0.0f ? pSphere->getSize().x : -pSphere->getSize().x;

	if (abs(dist) <= abs(radius))
	{
		auto spherePoint = spherePos;
		auto collisionPoint = spherePoint - radius * normal;

		auto sizeX = pPlaneHoles->getSize().x * 5;
		auto sizeY = pPlaneHoles->getSize().z * 5;

		auto dotX = dot(collisionPoint - center, tangent);
		auto dotY = dot(collisionPoint - center, bitangent);

		radius = pSphere->getSize().x;
		auto time = 0.0f;
		if (dotX <= sizeX && dotX >= -sizeX &&
			dotY <= sizeY && dotY >= -sizeY)
		{
			std::vector<glm::vec3> centers =
			{
				-3.0f * tangent,
				-3.0f * tangent + 3.0f * bitangent,
				3.0f * bitangent,
				3.0f * tangent + 3.0f * bitangent,
				3.0f * tangent,
				3.0f * tangent + -3.0f * bitangent,
				-3.0f * bitangent,
				-3.0f * tangent + -3.0f * bitangent
			};

			for (auto i = 0u; i < centers.size(); i++)
			{
				if (dotX <= centers[i].x + 1 && dotX >= centers[i].x - 1 &&
					dotY <= centers[i].z + 1 && dotY >= centers[i].z - 1)
				{
					auto insideCircle = false;
					auto segments = 20;
					const auto angle = static_cast<float>(M_PI * 2) / segments;

					for (auto j = 0; j < segments; j++)
					{
						auto tempCenter = center + centers[i];
						auto tempVertex1 = tempCenter + cos(j * angle) * tangent + sin(j * angle) * bitangent;
						auto tempVertex2 = tempCenter + cos(static_cast<float>(j + 1) * angle) * tangent + sin(static_cast<float>(j + 1) * angle) * bitangent;

						if (detectCollisionSphereTriangle(pSphere, tempCenter, tempVertex1, tempVertex2, planeVelocity, pLastCollisionTime))
						{
							insideCircle = true;

							if (detectCollisionSphereLine(pSphere, tempVertex1, tempVertex2, planeVelocity, time, pLastCollisionTime))
							{
								spherePoint = spherePos + time * velocity;

								auto lineVector = tempVertex2 - tempVertex1;

								auto t = dot(spherePoint - tempVertex1, lineVector) / dot(lineVector, lineVector);

								auto closestPoint = tempVertex1 + t * lineVector;

								ManifoldPoint manPoint = {
									pSphere,
									pPlaneHoles,
									closestPoint,
									closestPoint,
									normalize(normal),
									pLastCollisionTime + time * (1.0f - pLastCollisionTime),
									0.0f,
									CollisionType::COLLISION
								};
							
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

			collisionPoint = center + dotX * tangent + dotY * bitangent;
			
			ManifoldPoint manPoint = {
				pSphere,
				pPlaneHoles,
				collisionPoint,
				collisionPoint,
				normalize(normal),
				pLastCollisionTime,
				abs(radius) - abs(dist),
				CollisionType::PENETRATION
			};
			
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

			for (auto i = 0u; i < startPoints.size(); i++)
			{
				if (detectCollisionSphereLine(pSphere, startPoints[i], endPoints[i], planeVelocity, time, pLastCollisionTime))
				{
					spherePoint = spherePos + time * velocity;

					auto lineVector = endPoints[i] - startPoints[i];

					auto t = dot(spherePoint - startPoints[i], lineVector) / dot(lineVector, lineVector);

					auto closestPoint = startPoints[i] + t * lineVector;

					ManifoldPoint manPoint = {
						pSphere,
						pPlaneHoles,
						closestPoint,
						closestPoint,
						normalize(normal),
						pLastCollisionTime + time * (1.0f - pLastCollisionTime),
						0.0f,
						CollisionType::COLLISION
					};
					
					pManifold->add(manPoint);
					return;
				}
			}

			for (auto i = 0u; i < startPoints.size(); i++)
			{
				if (detectCollisionSphereVertex(pSphere, startPoints[i], planeVelocity, time, pLastCollisionTime))
				{
					spherePoint = spherePos + time * velocity;

					auto closestPoint = startPoints[i];

					ManifoldPoint manPoint = {
						pSphere,
						pPlaneHoles,
						closestPoint,
						closestPoint,
						normalize(normal),
						pLastCollisionTime + time * (1.0f - pLastCollisionTime),
						0.0f,
						CollisionType::PENETRATION
					};

					pManifold->add(manPoint);
					return;
				}
			}
		}
	}

	auto velDot = dot(normal, velocity);

	if (velDot * dist >= 0)
	{
		return; //Moving parallel no collision;
	}

	auto time = (radius - dist) / velDot;

	if (time >= 0 && time <= 1)
	{
		auto spherePoint = spherePos + time * velocity;
		auto collisionPoint = spherePoint - radius * normal;

		auto sizeX = pPlaneHoles->getSize().x * 5;
		auto sizeY = pPlaneHoles->getSize().z * 5;

		auto dotX = dot(collisionPoint - center + planeVelocity * time, tangent);
		auto dotY = dot(collisionPoint - center + planeVelocity * time, bitangent);

		radius = pSphere->getSize().x;

		if (dotX <= sizeX && dotX >= -sizeX &&
			dotY <= sizeY && dotY >= -sizeY)
		{
			std::vector<glm::vec3> centers =
			{
				-3.0f * tangent,
				-3.0f * tangent + 3.0f * bitangent,
				3.0f * bitangent,
				3.0f * tangent + 3.0f * bitangent,
				3.0f * tangent,
				3.0f * tangent + -3.0f * bitangent,
				-3.0f * bitangent,
				-3.0f * tangent + -3.0f * bitangent
			};

			for (auto& i : centers)
			{
				if (dotX <= i.x + 1 && dotX >= i.x - 1 &&
					dotY <= i.z + 1 && dotY >= i.z - 1)
				{
					auto insideCircle = false;
					auto segments = 20.0f;
					const auto angle = static_cast<float>(M_PI * 2) / segments;
					
					for (auto j = 0; j < segments; j++)
					{
						auto tempCenter = i + center;
						auto tempVertex1 = tempCenter + cos(j * angle) * tangent + sin(j * angle) * bitangent;
						auto tempVertex2 = tempCenter + cos(static_cast<float>(j + 1) * angle) * tangent + sin(static_cast<float>(j + 1) * angle) * bitangent;

						if (detectCollisionSphereTriangle(pSphere, tempCenter, tempVertex1, tempVertex2, planeVelocity, pLastCollisionTime))
						{
							insideCircle = true;

							if(detectCollisionSphereLine(pSphere, tempVertex1, tempVertex2, planeVelocity, time, pLastCollisionTime))
							{
								spherePoint = spherePos + time * velocity;

								auto lineVector = tempVertex2 - tempVertex1;

								auto t = dot(spherePoint - tempVertex1, lineVector) / dot(lineVector, lineVector);

								auto closestPoint = tempVertex1 + t * lineVector;

								ManifoldPoint manPoint = {
									pSphere,
									pPlaneHoles,
									closestPoint,
									closestPoint,
									normalize(normal),
									pLastCollisionTime + time * (1.0f - pLastCollisionTime),
									0.0f,
									CollisionType::COLLISION
								};
								
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

			collisionPoint = center + dotX * tangent + dotY * bitangent;
			
			ManifoldPoint manPoint = {
				pSphere,
				pPlaneHoles,
				collisionPoint,
				collisionPoint,
				normalize(normal),
				pLastCollisionTime + time * (1.0f - pLastCollisionTime),
				0.0f,
				CollisionType::COLLISION
			};

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

					auto t = dot(spherePoint - startPoints[i], lineVector) / dot(lineVector, lineVector);

					auto closestPoint = startPoints[i] + t * lineVector;

					ManifoldPoint manPoint =
					{
						pSphere,
						pPlaneHoles,
						closestPoint,
						closestPoint,
						normalize(normal),
						pLastCollisionTime + time * (1.0f - pLastCollisionTime),
						0.0f,
						CollisionType::COLLISION
					};

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
					manPoint.mContactNormal = normalize(closestPoint - spherePoint);
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

bool CollisionDetection::detectCollisionSphereLine(RigidBody * pSphere, const glm::vec3 pLineEnd1, const glm::vec3 pLineEnd2, const glm::vec3 pLineVelocity, float& pTime, const float pLastCollisionTime)
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
	const auto radius = pSphere->getSize().x;

	const auto md = dot(m, d);
	const auto nd = dot(n, d);
	const auto dd = dot(d, d);

	if (md < 0.0 && md + nd < 0.0)
	{
		return false; //Outside cylinder
	}

	if (md > dd && md + nd > dd)
	{
		return false; //Outside cylinder
	}

	const auto mn = dot(m, n);
	const auto nn = dot(n, n);

	const auto a = dd * nn - nd * nd;
	const auto k = dot(m, m) - radius * radius;
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

bool CollisionDetection::detectCollisionSphereVertex(RigidBody * pSphere, glm::vec3 pVertex, glm::vec3 pVertexVelocity, float& pTime, const float pLastCollisionTime)
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
	const auto radius = pSphere->getSize().x;

	const auto d = sphereEnd - sphereStart;

	const auto m = sphereStart - pVertex;

	const auto a = dot(d, d);
	const auto b = dot(m, d);
	const auto c = dot(m, m) - radius * radius;

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

bool CollisionDetection::detectCollisionSphereTriangle(RigidBody * pSphere, glm::vec3 pVertex1, glm::vec3 pVertex2, glm::vec3 pVertex3, glm::vec3 pTriangleVelocity, const float pLastCollisionTime)
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
	auto radius = pSphere->getSize().x;

	auto planeNormal = normalize(cross(pVertex2 - pVertex1, pVertex3 - pVertex1));
	auto planeDot = dot(planeNormal, pVertex2);

	auto dist = dot(planeNormal, spherePos) - planeDot;

	if (abs(dist) <= radius)
	{
		auto collisionPoint = spherePos - (-radius + radius - abs(dist)) * planeNormal;

		auto tempVertex1 = pVertex1 - collisionPoint;
		auto tempVertex2 = pVertex2 - collisionPoint;
		auto tempVertex3 = pVertex3 - collisionPoint;

		auto u = cross(tempVertex2, tempVertex3);
		auto v = cross(tempVertex3, tempVertex1);

		if (dot(u, v) < 0.0f)
		{
			return false;
		}

		auto w = cross(tempVertex1, tempVertex2);

		if (dot(u, w) < 0.0f)
		{
			return false;
		}

		return true;
	}

	auto denom = dot(planeNormal, velocity);

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

	auto u = cross(tempVertex2, tempVertex3);
	auto v = cross(tempVertex3, tempVertex1);

	if (dot(u, v) < 0.0f)
	{
		return false;
	}

	auto w = cross(tempVertex1, tempVertex2);

	if (dot(u, w) < 0.0f)
	{
		return false;
	}

	return true;
}

void CollisionDetection::detectCollisionSphereCuboid(RigidBody * pSphere, RigidBody * pCuboid, ContactManifold * pManifold, const float pLastCollisionTime)
{
	glm::vec3 spherePos;

	if (pSphere->getCurrentUpdateTime() < pLastCollisionTime)
	{
		const auto interValue = (pLastCollisionTime - pSphere->getCurrentUpdateTime()) / (1.0f - pSphere->getCurrentUpdateTime());

		spherePos = mix(pSphere->getPos(), pSphere->getNewPos(), interValue);
	}
	else
	{
		spherePos = pSphere->getPos();
	}

	glm::vec3 cuboidPos;

	if (pCuboid->getCurrentUpdateTime() < pLastCollisionTime)
	{
		const auto interValue = (pLastCollisionTime - pCuboid->getCurrentUpdateTime()) / (1.0f - pCuboid->getCurrentUpdateTime());

		cuboidPos = mix(pCuboid->getPos(), pCuboid->getNewPos(), interValue);
	}
	else
	{
		cuboidPos = pCuboid->getPos();
	}

	const auto sphereRadius = pSphere->getSize().x;
	const auto cuboidSize = pCuboid->getSize();

	const auto cuboidVelocity = pCuboid->getNewPos() - cuboidPos;
	const auto sphereVelocity = pSphere->getNewPos() - spherePos;

	if (dot(cuboidVelocity, sphereVelocity) > 0.0f)
	{
		//Moving away so don't collide
		return;
	}

	auto timeStart = pLastCollisionTime;
	auto currentTime = timeStart;
	auto timeEnd = 1.0f;
	auto firstTime = true;

	while (true)
	{
		if (firstTime)
		{
			//Time = 0
			{
				const auto interValue = (currentTime - pCuboid->getCurrentUpdateTime()) / (1.0f - pCuboid->getCurrentUpdateTime());
				const auto cuboidOrr = slerp(pCuboid->getOrientation(), pCuboid->getNewOrientation(), interValue);
				const auto cuboidOrrMat = toMat4(cuboidOrr);

				const auto cuboidNormal = glm::vec3(cuboidOrrMat * glm::vec4(0, 1, 0, 1.0f));
				const auto cuboidTangent = glm::vec3(cuboidOrrMat * glm::vec4(1, 0, 0, 1.0f));
				const auto cuboidBiTangent = glm::vec3(cuboidOrrMat * glm::vec4(0, 0, 1, 1.0f));

				const auto tempCuboidPos = mix(cuboidPos, pCuboid->getNewPos(), interValue);
				const auto tempSpherePos = mix(spherePos, pSphere->getNewPos(), interValue);

				glm::vec3 collisionPoint;
				
				if (detectCollisionSphereCuboidStep(tempSpherePos, sphereRadius, tempCuboidPos, cuboidNormal, cuboidTangent, cuboidBiTangent, pCuboid->getSize(), collisionPoint))
				{
					const auto dist = sphereRadius - length(tempSpherePos - collisionPoint);

					ManifoldPoint manPoint = {
						pSphere,
						pCuboid,
						collisionPoint,
						collisionPoint,
						normalize(calculateCuboidCollisionNormal(tempCuboidPos, cuboidNormal, cuboidTangent, cuboidBiTangent, cuboidSize, collisionPoint)),
						0.0f,
						dist,
						CollisionType::PENETRATION
					};
				}
			}

			//Time = 1
			{
				const auto interValue = (timeEnd - pCuboid->getCurrentUpdateTime()) / (1.0f - pCuboid->getCurrentUpdateTime());
				const auto cuboidOrr = slerp(pCuboid->getOrientation(), pCuboid->getNewOrientation(), interValue);
				const auto cuboidOrrMat = toMat4(cuboidOrr);

				const auto cuboidNormal = glm::vec3(cuboidOrrMat * glm::vec4(0, 1, 0, 1.0f));
				const auto cuboidTangent = glm::vec3(cuboidOrrMat * glm::vec4(1, 0, 0, 1.0f));
				const auto cuboidBiTangent = glm::vec3(cuboidOrrMat * glm::vec4(0, 0, 1, 1.0f));

				const auto tempCuboidPos = mix(cuboidPos, pCuboid->getNewPos(), interValue);
				const auto tempSpherePos = mix(spherePos, pSphere->getNewPos(), interValue);

				glm::vec3 collisionPoint;

				if (!detectCollisionSphereCuboidStep(tempSpherePos, sphereRadius, tempCuboidPos, cuboidNormal, cuboidTangent, cuboidBiTangent, pCuboid->getSize(), collisionPoint))
				{
					return;
				}

				currentTime = glm::mix(timeStart, timeEnd, 0.5f);
				firstTime = false;
			}
		}
		else //Not First Time
		{
			const auto interValue = (currentTime - pCuboid->getCurrentUpdateTime()) / (1.0f - pCuboid->getCurrentUpdateTime());
			const auto cuboidOrr = slerp(pCuboid->getOrientation(), pCuboid->getNewOrientation(), interValue);
			const auto cuboidOrrMat = toMat4(cuboidOrr);

			const auto cuboidNormal = glm::vec3(cuboidOrrMat * glm::vec4(0, 1, 0, 1.0f));
			const auto cuboidTangent = glm::vec3(cuboidOrrMat * glm::vec4(1, 0, 0, 1.0f));
			const auto cuboidBiTangent = glm::vec3(cuboidOrrMat * glm::vec4(0, 0, 1, 1.0f));

			const auto tempCuboidPos = mix(cuboidPos, pCuboid->getNewPos(), interValue);
			const auto tempSpherePos = mix(spherePos, pSphere->getNewPos(), interValue);

			glm::vec3 collisionPoint;

			if (detectCollisionSphereCuboidStep(tempSpherePos, sphereRadius, tempCuboidPos, cuboidNormal, cuboidTangent, cuboidBiTangent, pCuboid->getSize(), collisionPoint))
			{
				timeEnd = currentTime;
				currentTime = glm::mix(timeStart, timeEnd, 0.5f);
			}
			else
			{
				const auto dist = length(tempSpherePos - collisionPoint) - sphereRadius;

				if (dist < 0.0005f)
				{
					ManifoldPoint manPoint = {
						pSphere,
						pCuboid,
						collisionPoint,
						collisionPoint,
						normalize(calculateCuboidCollisionNormal(tempCuboidPos, cuboidNormal, cuboidTangent, cuboidBiTangent, cuboidSize, collisionPoint)),
						currentTime,
						0.0f,
						CollisionType::COLLISION
					};

					pManifold->add(manPoint);
					return;
				}

				timeStart = currentTime;
				currentTime = glm::mix(timeStart, timeEnd, 0.5f);
			}
		}
	}
}

bool CollisionDetection::detectCollisionSphereCuboidStep(const glm::vec3 pSphereCenter, const float pSphereRadius, const glm::vec3 pCuboidCenter,
                                                         const glm::vec3 pCuboidXAxis, const glm::vec3 pCuboidYAxis, const glm::vec3 pCuboidZAxis, const glm::vec3 pCuboidSize, glm::vec3 & pPoint)
{
	//Get closest point on cuboid from sphere center
	const auto d = pSphereCenter - pCuboidCenter;

	pPoint = pCuboidCenter;

	{
		auto dist = dot(d, pCuboidXAxis);

		if (dist > pCuboidSize.x)
		{
			dist = pCuboidSize.x;
		}
		else if (dist < -pCuboidSize.x)
		{
			dist = -pCuboidSize.x;
		}

		pPoint = pPoint + dist * pCuboidXAxis;
	}

	{
		auto dist = dot(d, pCuboidYAxis);

		if (dist > pCuboidSize.y)
		{
			dist = pCuboidSize.y;
		}
		else if (dist < -pCuboidSize.y)
		{
			dist = -pCuboidSize.y;
		}

		pPoint = pPoint + dist * pCuboidYAxis;
	}

	{
		auto dist = dot(d, pCuboidZAxis);

		if (dist > pCuboidSize.z)
		{
			dist = pCuboidSize.z;
		}
		else if (dist < -pCuboidSize.z)
		{
			dist = -pCuboidSize.z;
		}

		pPoint = pPoint + dist * pCuboidZAxis;
	}

	//Check if point on cuboid is touching sphere, therefore colliding

	const auto dist = pPoint - pSphereCenter;

	const auto dis = length(dist);

	return dis <= pSphereRadius;
}

glm::vec3 CollisionDetection::calculateCuboidCollisionNormal(const glm::vec3 pCuboidCenter, const glm::vec3 pCuboidXAxis, const glm::vec3 pCuboidYAxis, const glm::vec3 pCuboidZAxis, const glm::vec3 pCuboidSize, const glm::vec3 pPoint)
{
	const auto d = pPoint - pCuboidCenter;

	const auto xDist = dot(d,pCuboidXAxis);
	const auto yDist = dot(d,pCuboidYAxis);
	const auto zDist = dot(d,pCuboidZAxis);

	if (xDist == pCuboidSize.x &&
		yDist == pCuboidSize.y &&
		zDist == pCuboidSize.z)
	{
		return pCuboidXAxis + pCuboidYAxis + pCuboidZAxis;
	}
	if (xDist == pCuboidSize.x &&
		yDist == pCuboidSize.y &&
		zDist == -pCuboidSize.z)
	{
		return pCuboidXAxis + pCuboidYAxis - pCuboidZAxis;
	}
	if (xDist == pCuboidSize.x &&
		yDist == -pCuboidSize.y &&
		zDist == pCuboidSize.z)
	{
		return pCuboidXAxis - pCuboidYAxis + pCuboidZAxis;
	}
	if (xDist == pCuboidSize.x &&
		yDist == -pCuboidSize.y &&
		zDist == -pCuboidSize.z)
	{
		return pCuboidXAxis - pCuboidYAxis - pCuboidZAxis;
	}
	if (xDist == -pCuboidSize.x &&
		yDist == pCuboidSize.y &&
		zDist == pCuboidSize.z)
	{
		return -1.0f * pCuboidXAxis + pCuboidYAxis + pCuboidZAxis;
	}
	if (xDist == -pCuboidSize.x &&
		yDist == pCuboidSize.y &&
		zDist == -pCuboidSize.z)
	{
		return -1.0f * pCuboidXAxis + pCuboidYAxis - pCuboidZAxis;
	}
	if (xDist == -pCuboidSize.x &&
		yDist == -pCuboidSize.y &&
		zDist == pCuboidSize.z)
	{
		return -1.0f * pCuboidXAxis - pCuboidYAxis + pCuboidZAxis;
	}
	if (xDist == -pCuboidSize.x &&
		yDist == -pCuboidSize.y &&
		zDist == -pCuboidSize.z)
	{
		return -1.0f * pCuboidXAxis - pCuboidYAxis - pCuboidZAxis;
	}
	if (xDist == pCuboidSize.x &&
		yDist == pCuboidSize.y)
	{
		return pCuboidXAxis + pCuboidYAxis;
	}
	if (xDist == pCuboidSize.x &&
		yDist == -pCuboidSize.y)
	{
		return pCuboidXAxis - pCuboidYAxis;
	}
	if (xDist == pCuboidSize.x &&
		zDist == pCuboidSize.z)
	{
		return pCuboidXAxis + pCuboidZAxis;
	}
	if (xDist == pCuboidSize.x &&
		zDist == -pCuboidSize.z)
	{
		return pCuboidXAxis - pCuboidZAxis;
	}
	if (xDist == -pCuboidSize.x &&
		yDist == pCuboidSize.y)
	{
		return -1.0f * pCuboidXAxis + pCuboidYAxis;
	}
	if (xDist == -pCuboidSize.x &&
		yDist == -pCuboidSize.y)
	{
		return -1.0f * pCuboidXAxis - pCuboidYAxis;
	}
	if (xDist == -pCuboidSize.x &&
		zDist == pCuboidSize.z)
	{
		return -1.0f * pCuboidXAxis + pCuboidZAxis;
	}
	if (xDist == -pCuboidSize.x &&
		zDist == -pCuboidSize.z)
	{
		return -1.0f * pCuboidXAxis - pCuboidZAxis;
	}
	if (yDist == pCuboidSize.y &&
		zDist == pCuboidSize.z)
	{
		return pCuboidYAxis + pCuboidZAxis;
	}
	if (yDist == pCuboidSize.y &&
		zDist == -pCuboidSize.z)
	{
		return pCuboidYAxis - pCuboidZAxis;
	}
	if (yDist == -pCuboidSize.y &&
		zDist == pCuboidSize.z)
	{
		return -1.0f * pCuboidYAxis + pCuboidZAxis;
	}
	if (yDist == -pCuboidSize.y &&
		zDist == -pCuboidSize.z)
	{
		return -1.0f * pCuboidYAxis - pCuboidZAxis;
	}
	if (xDist == pCuboidSize.x)
	{
		return pCuboidXAxis;
	}
	if (xDist == -pCuboidSize.x)
	{
		return -1.0f * pCuboidXAxis;
	}
	if (yDist == pCuboidSize.y)
	{
		return pCuboidYAxis;
	}
	if (yDist == -pCuboidSize.y)
	{
		return -1.0f * pCuboidYAxis;
	}
	if (zDist == pCuboidSize.z)
	{
		return pCuboidZAxis;
	}
	if (zDist == -pCuboidSize.z)
	{
		return -1.0f * pCuboidZAxis;
	}

	//TODO: Check that this is almost never reached
	return glm::vec3(0, 1, 0);
}

bool CollisionDetection::detectCollisionCuboidCuboidStep(glm::vec3 pCuboid1Center, std::vector<glm::vec3> pCuboidAxis1, glm::vec3 pCuboidSize1, glm::vec3 pCuboid2Center, std::vector<glm::vec3> pCuboidAxis2, glm::vec3 pCuboidSize2)
{
	float ra, rb;
	glm::mat3 rot, absRot;

	for (int i = 0; i < 3; i++)
	{
		for (int j = 0; j < 3; j++)
		{
			rot[i][j] = dot(pCuboidAxis1[i], pCuboidAxis2[j]);
		}
	}

	const auto t = pCuboid2Center - pCuboid1Center;

	for (auto i = 0u; i < 3; i++)
	{
		for (auto j = 0u; j < 3; j++)
		{
			absRot[i][j] = abs(rot[i][j]);
		}
	}

	{
		ra = pCuboidSize1.x;
		rb = pCuboidSize2.x * absRot[0][0] + pCuboidSize2.y * absRot[0][1] + pCuboidSize2.z * absRot[0][2];
		const auto temp = dot(t, pCuboidAxis1[0]);
		
		if (abs(temp) > ra + rb + 0.001f)
		{
			return false;
		}
	}

	{
		ra = pCuboidSize1.y;
		rb = pCuboidSize2.x * absRot[1][0] + pCuboidSize2.y * absRot[1][1] + pCuboidSize2.z * absRot[1][2];
		const auto temp = dot(t, pCuboidAxis1[1]);
		
		if (abs(temp) > ra + rb + 0.001f)
		{
			return false;
		}
	}

	{
		ra = pCuboidSize1.z;
		rb = pCuboidSize2.x * absRot[2][0] + pCuboidSize2.y * absRot[2][1] + pCuboidSize2.z * absRot[2][2];
		const auto temp = dot(t, pCuboidAxis1[2]);
		
		if (abs(temp) > ra + rb + 0.001f)
		{
			return false;
		}
	}

	{
		ra = pCuboidSize1.x * absRot[0][0] + pCuboidSize1.y * absRot[1][0] + pCuboidSize1.z * absRot[2][0];
		rb = pCuboidSize2.x;
		const auto temp = dot(t, pCuboidAxis2[0]);
		if (abs(temp) > ra + rb + 0.001f)
		{
			return false;
		}
	}

	{
		ra = pCuboidSize1.x * absRot[0][1] + pCuboidSize1.y * absRot[1][1] + pCuboidSize1.z * absRot[2][1];
		rb = pCuboidSize2.y;
		
		const auto temp = dot(t, pCuboidAxis2[1]);

		if (abs(temp) > ra + rb + 0.001f)
		{
			return false;
		}
	}

	{
		ra = pCuboidSize1.x * absRot[0][2] + pCuboidSize1.y * absRot[1][2] + pCuboidSize1.z * absRot[2][2];
		rb = pCuboidSize2.z;
		
		const auto temp = dot(t, pCuboidAxis2[2]);

		if (abs(temp) > ra + rb + 0.001f)
		{
			return false;
		}
	}

	{
		ra = pCuboidSize1.y * absRot[2][0] + pCuboidSize1.z * absRot[1][0];
		rb = pCuboidSize2.y * absRot[0][2] + pCuboidSize2.z * absRot[0][1];
		
		const auto temp = dot(t, cross(pCuboidAxis1[0], pCuboidAxis2[0]));

		if (abs(temp) > ra + rb + 0.001f)
		{
			return false;
		}
	}

	{
		ra = pCuboidSize1.y * absRot[2][1] + pCuboidSize1.z * absRot[1][1];
		rb = pCuboidSize2.x * absRot[0][2] + pCuboidSize2.z * absRot[0][0];
		
		const auto temp = dot(t, cross(pCuboidAxis1[0], pCuboidAxis2[1]));

		if (abs(temp) > ra + rb + 0.001f)
		{
			return false;
		}
	}

	{
		ra = pCuboidSize1.y * absRot[2][2] + pCuboidSize2.z * absRot[1][2];
		rb = pCuboidSize2.x * absRot[0][1] + pCuboidSize2.y * absRot[0][0];
		const auto temp = dot(t, cross(pCuboidAxis1[0], pCuboidAxis2[2]));

		if (abs(temp) > ra + rb + 0.001f)
		{
			return false;
		}
	}

	{
		ra = pCuboidSize1.x * absRot[2][0] + pCuboidSize1.z * absRot[0][0];
		rb = pCuboidSize2.y * absRot[1][2] + pCuboidSize2.z * absRot[1][1];
		const auto temp = dot(t, cross(pCuboidAxis1[1], pCuboidAxis2[0]));

		if (abs(temp) > ra + rb + 0.001f)
		{
			return false;
		}
	}

	//Typo
	
	{
		ra = pCuboidSize1.x * absRot[2][1] + pCuboidSize1.z * absRot[0][1];
		rb = pCuboidSize2.x * absRot[1][2] + pCuboidSize2.z * absRot[1][0];
		const auto temp = dot(t, cross(pCuboidAxis1[1], pCuboidAxis2[1]));

		if (abs(temp) > ra + rb + 0.001f)
		{
			return false;
		}
	}

	{
		ra = pCuboidSize1.x * absRot[2][2] + pCuboidSize1.z * absRot[0][2];
		rb = pCuboidSize2.x * absRot[1][1] + pCuboidSize2.y * absRot[1][0];
		const auto temp = dot(t, cross(pCuboidAxis1[1], pCuboidAxis2[2]));

		if (abs(temp) > ra + rb + 0.001f)
		{
			return false;
		}
	}

	{
		ra = pCuboidSize1.x * absRot[1][0] + pCuboidSize1.y * absRot[0][0];
		rb = pCuboidSize2.y * absRot[2][2] + pCuboidSize2.z * absRot[2][0];
		const auto temp = dot(t, cross(pCuboidAxis1[2], pCuboidAxis2[0]));

		if (abs(temp) > ra + rb + 0.001f)
		{
			return false;
		}
	}

	{
		ra = pCuboidSize1.x * absRot[1][1] + pCuboidSize1.y * absRot[0][1];
		rb = pCuboidSize2.x * absRot[2][2] + pCuboidSize2.z * absRot[2][0];
		const auto temp = dot(t, cross(pCuboidAxis1[2], pCuboidAxis2[1]));

		if (abs(temp) > ra + rb + 0.001f)
		{
			return false;
		}
	}

	{
		ra = pCuboidSize1.x * absRot[1][2] + pCuboidSize1.y * absRot[0][2];
		rb = pCuboidSize2.x * absRot[2][1] + pCuboidSize2.y * absRot[2][0];
		const auto temp = dot(t, cross(pCuboidAxis1[2], pCuboidAxis2[2]));

		if (abs(temp) > ra + rb + 0.001f)
		{
			return false;
		}
	}

	return true;
}

float CollisionDetection::calculateCuboidCuboidCollisionDepth(glm::vec3 pPoint, glm::vec3 pCuboidCenter, std::vector<glm::vec3> pCuboidAxis, glm::vec3 pCuboidSize, bool& pInside, glm::vec3& pCollisionPoint)
{
	const auto d = pPoint - pCuboidCenter;

	pCollisionPoint = pCuboidCenter;

	pInside = true;
	
	for (int i = 0; i < 3; i++)
	{
		float dist = dot(d, pCuboidAxis[i]);

		if (dist > pCuboidSize[i])
		{
			dist = pCuboidSize[i];
			pInside = false;
		}
		else if (dist < -pCuboidSize[i])
		{
			dist = -pCuboidSize[i];
			pInside = false;
		}

		pCollisionPoint += dist * pCuboidAxis[i];
	}

	return length(pCollisionPoint - pPoint);
}

void CollisionDetection::detectCollisionCuboidCuboid(RigidBody* pCuboid1, RigidBody* pCuboid2, ContactManifold* pManifold, float pLastCollisionTime)
{
	//TODO: Implement
	glm::vec3 cuboid1Pos;

	if (pCuboid1->getCurrentUpdateTime() < pLastCollisionTime)
	{
		const auto interValue = (pLastCollisionTime - pCuboid1->getCurrentUpdateTime()) / (1.0f - pCuboid1->getCurrentUpdateTime());
		cuboid1Pos = pCuboid1->getPos() + interValue * (pCuboid1->getNewPos() - pCuboid1->getPos());
	}
	else
	{
		cuboid1Pos = pCuboid1->getPos();
	}

	glm::vec3 cuboid2Pos;

	if (pCuboid2->getCurrentUpdateTime() < pLastCollisionTime)
	{
		const auto interValue = (pLastCollisionTime - pCuboid2->getCurrentUpdateTime()) / (1.0f - pCuboid2->getCurrentUpdateTime());
		cuboid2Pos = pCuboid2->getPos() + interValue * (pCuboid2->getNewPos() - pCuboid2->getPos());
	}
	else
	{
		cuboid2Pos = pCuboid2->getPos();
	}

	auto timeStart = pLastCollisionTime;
	auto currentTime = timeStart;
	auto timeEnd = 1.0f;
	auto firstTime = true;

	const auto cuboid1Size = pCuboid1->getSize();
	const auto cuboid2Size = pCuboid2->getSize();

	const auto cuboid1Velocity = pCuboid1->getNewPos() - cuboid1Pos;
	const auto cuboid2Velocity = pCuboid2->getNewPos() - cuboid2Pos;

	while (true)
	{
		if (firstTime)
		{
			//Time = 0
			{
				auto interValue = (currentTime - pCuboid1->getCurrentUpdateTime()) / (1.0f - pCuboid1->getCurrentUpdateTime());
				const auto cuboid1Orr = slerp(pCuboid1->getOrientation(), pCuboid1->getNewOrientation(), interValue);
				const auto cuboid1OrrMat = toMat4(cuboid1Orr);

				const auto cuboid1Normal = glm::vec3(cuboid1OrrMat * glm::vec4(0, 1, 0, 1.0f));
				const auto cuboid1Tangent = glm::vec3(cuboid1OrrMat * glm::vec4(1, 0, 0, 1.0f));
				const auto cuboid1BiTangent = glm::vec3(cuboid1OrrMat * glm::vec4(0, 0, 1, 1.0f));

				const auto tempCuboid1Pos = mix(cuboid1Pos, pCuboid1->getNewPos(), interValue);

				const auto tempCuboid1Axis = std::vector<glm::vec3>{
					cuboid1Normal, cuboid1Tangent, cuboid1BiTangent
				};

				interValue = (currentTime - pCuboid2->getCurrentUpdateTime()) / (1.0f - pCuboid2->getCurrentUpdateTime());
				const auto cuboid2Orr = slerp(pCuboid2->getOrientation(), pCuboid2->getNewOrientation(), interValue);
				const auto cuboid2OrrMat = toMat4(cuboid2Orr);

				const auto cuboid2Normal = glm::vec3(cuboid2OrrMat * glm::vec4(0, 1, 0, 1.0f));
				const auto cuboid2Tangent = glm::vec3(cuboid2OrrMat * glm::vec4(1, 0, 0, 1.0f));
				const auto cuboid2BiTangent = glm::vec3(cuboid2OrrMat * glm::vec4(0, 0, 1, 1.0f));

				const auto tempCuboid2Pos = mix(cuboid2Pos, pCuboid2->getNewPos(), interValue);

				const auto tempCuboid2Axis = std::vector<glm::vec3>{
					cuboid2Normal, cuboid2Tangent, cuboid2BiTangent
				};

				if (detectCollisionCuboidCuboidStep(tempCuboid1Pos, tempCuboid1Axis, cuboid1Size, tempCuboid2Pos, tempCuboid2Axis, cuboid2Size))
				{
					//Check if vertex or center of cuboid1 is inside cuboid2
					const auto cuboid1Points = std::vector<glm::vec3>{
						tempCuboid1Pos + cuboid1Normal * cuboid1Size.x + cuboid1Tangent * cuboid1Size.y + cuboid1BiTangent * cuboid1Size.z,
						tempCuboid1Pos + cuboid1Normal * cuboid1Size.x + cuboid1Tangent * cuboid1Size.y - cuboid1BiTangent * cuboid1Size.z,
						tempCuboid1Pos + cuboid1Normal * cuboid1Size.x - cuboid1Tangent * cuboid1Size.y + cuboid1BiTangent * cuboid1Size.z,
						tempCuboid1Pos + cuboid1Normal * cuboid1Size.x - cuboid1Tangent * cuboid1Size.y - cuboid1BiTangent * cuboid1Size.z,
						tempCuboid1Pos - cuboid1Normal * cuboid1Size.x + cuboid1Tangent * cuboid1Size.y + cuboid1BiTangent * cuboid1Size.z,
						tempCuboid1Pos - cuboid1Normal * cuboid1Size.x + cuboid1Tangent * cuboid1Size.y - cuboid1BiTangent * cuboid1Size.z,
						tempCuboid1Pos - cuboid1Normal * cuboid1Size.x - cuboid1Tangent * cuboid1Size.y + cuboid1BiTangent * cuboid1Size.z,
						tempCuboid1Pos - cuboid1Normal * cuboid1Size.x - cuboid1Tangent * cuboid1Size.y - cuboid1BiTangent * cuboid1Size.z,
						tempCuboid1Pos + cuboid1Normal * cuboid1Size.x,
						tempCuboid1Pos - cuboid1Normal * cuboid1Size.x,
						tempCuboid1Pos + cuboid1Tangent * cuboid1Size.y,
						tempCuboid1Pos - cuboid1Tangent * cuboid1Size.y,
						tempCuboid1Pos + cuboid1BiTangent * cuboid1Size.z,
						tempCuboid1Pos - cuboid1BiTangent * cuboid1Size.z
					};
					
					for (auto point : cuboid1Points)
					{
						auto inside = false;
						auto collisionPoint = glm::vec3(0.0f);
						const auto depth = calculateCuboidCuboidCollisionDepth(point, tempCuboid2Pos, tempCuboid2Axis, cuboid2Size, inside, collisionPoint);

						if (inside)
						{
							ManifoldPoint manPoint = {
								pCuboid2,
								pCuboid1,
								collisionPoint,
								collisionPoint,
								normalize(calculateCuboidCollisionNormal(tempCuboid1Pos, tempCuboid1Axis[0], tempCuboid1Axis[1], tempCuboid1Axis[2], cuboid1Size, collisionPoint)),
								0.0f,
								depth,
								CollisionType::PENETRATION
							};

							pManifold->add(manPoint);
							return;
						}
					}

					//Check if vertex or center of cuboid2 is inside cuboid1
					const auto cuboid2Points = std::vector<glm::vec3>{
						tempCuboid2Pos + cuboid2Normal * cuboid2Size.x + cuboid2Tangent * cuboid2Size.y + cuboid2BiTangent * cuboid2Size.z,
						tempCuboid2Pos + cuboid2Normal * cuboid2Size.x + cuboid2Tangent * cuboid2Size.y - cuboid2BiTangent * cuboid2Size.z,
						tempCuboid2Pos + cuboid2Normal * cuboid2Size.x - cuboid2Tangent * cuboid2Size.y + cuboid2BiTangent * cuboid2Size.z,
						tempCuboid2Pos + cuboid2Normal * cuboid2Size.x - cuboid2Tangent * cuboid2Size.y - cuboid2BiTangent * cuboid2Size.z,
						tempCuboid2Pos - cuboid2Normal * cuboid2Size.x + cuboid2Tangent * cuboid2Size.y + cuboid2BiTangent * cuboid2Size.z,
						tempCuboid2Pos - cuboid2Normal * cuboid2Size.x + cuboid2Tangent * cuboid2Size.y - cuboid2BiTangent * cuboid2Size.z,
						tempCuboid2Pos - cuboid2Normal * cuboid2Size.x - cuboid2Tangent * cuboid2Size.y + cuboid2BiTangent * cuboid2Size.z,
						tempCuboid2Pos - cuboid2Normal * cuboid2Size.x - cuboid2Tangent * cuboid2Size.y - cuboid2BiTangent * cuboid2Size.z,
						tempCuboid2Pos + cuboid2Normal * cuboid2Size.x,
						tempCuboid2Pos - cuboid2Normal * cuboid2Size.x,
						tempCuboid2Pos + cuboid2Tangent * cuboid2Size.y,
						tempCuboid2Pos - cuboid2Tangent * cuboid2Size.y,
						tempCuboid2Pos + cuboid2BiTangent * cuboid2Size.z,
						tempCuboid2Pos - cuboid2BiTangent * cuboid2Size.z
					};

					for (auto point : cuboid2Points)
					{
						auto inside = false;
						auto collisionPoint = glm::vec3(0.0f);
						const auto depth = calculateCuboidCuboidCollisionDepth(point, tempCuboid1Pos, tempCuboid1Axis, cuboid1Size, inside, collisionPoint);

						if (inside)
						{
							ManifoldPoint manPoint = {
								pCuboid1,
								pCuboid2,
								collisionPoint,
								collisionPoint,
								normalize(calculateCuboidCollisionNormal(tempCuboid2Pos, tempCuboid2Axis[0], tempCuboid2Axis[1], tempCuboid2Axis[2], cuboid2Size, collisionPoint)),
								0.0f,
								depth,
								CollisionType::PENETRATION
							};

							pManifold->add(manPoint);
							return;
						}
					}

					//Check if edge of cuboid1 is inside cuboid2 and vice versa
					const auto cuboidPairs = std::vector<glm::ivec2>{
						glm::ivec2(0, 1),
						glm::ivec2(0, 2),
						glm::ivec2(0, 4),
						glm::ivec2(3, 1),
						glm::ivec2(3, 2),
						glm::ivec2(3, 7),
						glm::ivec2(5, 1),
						glm::ivec2(5, 4),
						glm::ivec2(5, 7),
						glm::ivec2(6, 2),
						glm::ivec2(6, 7),
						glm::ivec2(6, 4)
					};

					for (auto pair1 : cuboidPairs)
					{
						for (auto pair2 : cuboidPairs)
						{
							glm::vec3 collisionPoint1;
							glm::vec3 collisionPoint2;

							calculateClosestPointBetweenLines(cuboid1Points[pair1.x], cuboid1Points[pair1.y], cuboid2Points[pair2.x], cuboid2Points[pair2.y], collisionPoint1, collisionPoint2);

							{
								auto inside = false;
								auto collisionPoint = glm::vec3(0.0f);
								const auto depth = calculateCuboidCuboidCollisionDepth(collisionPoint1, tempCuboid2Pos, tempCuboid2Axis, cuboid2Size, inside, collisionPoint);

								if (inside)
								{
									ManifoldPoint manPoint = {
										pCuboid2,
										pCuboid1,
										collisionPoint,
										collisionPoint,
										normalize(calculateCuboidCollisionNormal(tempCuboid1Pos, tempCuboid1Axis[0], tempCuboid1Axis[1], tempCuboid1Axis[2], cuboid1Size, collisionPoint1)),
										0.0f,
										depth,
										CollisionType::PENETRATION
									};

									pManifold->add(manPoint);
									return;
								}
							}

							{
								auto inside = false;
								auto collisionPoint = glm::vec3(0.0f);
								const auto depth = calculateCuboidCuboidCollisionDepth(collisionPoint2, tempCuboid1Pos, tempCuboid1Axis, cuboid1Size, inside, collisionPoint);

								if (inside)
								{
									ManifoldPoint manPoint = {
										pCuboid1,
										pCuboid2,
										collisionPoint,
										collisionPoint,
										normalize(calculateCuboidCollisionNormal(tempCuboid2Pos, tempCuboid2Axis[0], tempCuboid2Axis[1], tempCuboid2Axis[2], cuboid2Size, collisionPoint2)),
										0.0f,
										depth,
										CollisionType::PENETRATION
									};

									pManifold->add(manPoint);
									return;
								}
							}
						}
					}

					//While there is a collision dont know where
					return;
				}
			}

			//Time = 1
			{
				auto interValue = (timeEnd - pCuboid1->getCurrentUpdateTime()) / (1.0f - pCuboid1->getCurrentUpdateTime());
				const auto cuboid1Orr = slerp(pCuboid1->getOrientation(), pCuboid1->getNewOrientation(), interValue);
				const auto cuboid1OrrMat = toMat4(cuboid1Orr);

				const auto cuboid1Normal = glm::vec3(cuboid1OrrMat * glm::vec4(0, 1, 0, 1.0f));
				const auto cuboid1Tangent = glm::vec3(cuboid1OrrMat * glm::vec4(1, 0, 0, 1.0f));
				const auto cuboid1BiTangent = glm::vec3(cuboid1OrrMat * glm::vec4(0, 0, 1, 1.0f));

				const auto tempCuboid1Pos = mix(cuboid1Pos, pCuboid1->getNewPos(), interValue);

				const auto tempCuboid1Axis = std::vector<glm::vec3>{
					cuboid1Normal, cuboid1Tangent, cuboid1BiTangent
				};

				interValue = (timeEnd - pCuboid2->getCurrentUpdateTime()) / (1.0f - pCuboid2->getCurrentUpdateTime());
				const auto cuboid2Orr = slerp(pCuboid2->getOrientation(), pCuboid2->getNewOrientation(), interValue);
				const auto cuboid2OrrMat = toMat4(cuboid2Orr);

				const auto cuboid2Normal = glm::vec3(cuboid2OrrMat * glm::vec4(0, 1, 0, 1.0f));
				const auto cuboid2Tangent = glm::vec3(cuboid2OrrMat * glm::vec4(1, 0, 0, 1.0f));
				const auto cuboid2BiTangnet = glm::vec3(cuboid2OrrMat * glm::vec4(0, 0, 1, 1.0f));

				const auto tempCuboid2Pos = mix(cuboid2Pos, pCuboid2->getNewPos(), interValue);

				const auto tempCuboid2Axis = std::vector<glm::vec3>{
					cuboid2Normal, cuboid2Tangent, cuboid2BiTangnet
				};

				if (!detectCollisionCuboidCuboidStep(tempCuboid1Pos, tempCuboid1Axis, cuboid1Size, tempCuboid2Pos, tempCuboid2Axis, cuboid2Size))
				{
					return;
				}
			}

			currentTime = glm::mix(timeStart, timeEnd, 0.5f);
			firstTime = false;
		}
		else //Not First Time
		{
			auto interValue = (currentTime - pCuboid1->getCurrentUpdateTime()) / (1.0f - pCuboid1->getCurrentUpdateTime());
			const auto cuboid1Orr = slerp(pCuboid1->getOrientation(), pCuboid1->getNewOrientation(), interValue);
			const auto cuboid1OrrMat = toMat4(cuboid1Orr);

			const auto cuboid1Normal = glm::vec3(cuboid1OrrMat * glm::vec4(0, 1, 0, 1.0f));
			const auto cuboid1Tangent = glm::vec3(cuboid1OrrMat * glm::vec4(1, 0, 0, 1.0f));
			const auto cuboid1BiTangent = glm::vec3(cuboid1OrrMat * glm::vec4(0, 0, 1, 1.0f));

			const auto tempCuboid1Pos = mix(cuboid1Pos, pCuboid1->getNewPos(), interValue);

			const auto tempCuboid1Axis = std::vector<glm::vec3>{
				cuboid1Normal, cuboid1Tangent, cuboid1BiTangent
			};

			interValue = (currentTime - pCuboid2->getCurrentUpdateTime()) / (1.0f - pCuboid2->getCurrentUpdateTime());
			const auto cuboid2Orr = slerp(pCuboid2->getOrientation(), pCuboid2->getNewOrientation(), interValue);
			const auto cuboid2OrrMat = toMat4(cuboid2Orr);

			const auto cuboid2Normal = glm::vec3(cuboid2OrrMat * glm::vec4(0, 1, 0, 1.0f));
			const auto cuboid2Tangent = glm::vec3(cuboid2OrrMat * glm::vec4(1, 0, 0, 1.0f));
			const auto cuboid2BiTangent = glm::vec3(cuboid2OrrMat * glm::vec4(0, 0, 1, 1.0f));

			const auto tempCuboid2Pos = mix(cuboid2Pos, pCuboid2->getNewPos(), interValue);

			const auto tempCuboid2Axis = std::vector<glm::vec3>{
				cuboid2Normal, cuboid2Tangent, cuboid2BiTangent
			};

			if (detectCollisionCuboidCuboidStep(tempCuboid1Pos, tempCuboid1Axis, cuboid1Size, tempCuboid2Pos, tempCuboid2Axis, cuboid2Size))
			{
				timeEnd = currentTime;
				currentTime = glm::mix(timeStart, timeEnd, 0.5f);
			}
			else
			{
				//Check if any point is with 0.0005f of other cuboid
				const auto cuboid1Points = std::vector<glm::vec3>{
						tempCuboid1Pos + cuboid1Normal * cuboid1Size.x + cuboid1Tangent * cuboid1Size.y + cuboid1BiTangent * cuboid1Size.z,
						tempCuboid1Pos + cuboid1Normal * cuboid1Size.x + cuboid1Tangent * cuboid1Size.y - cuboid1BiTangent * cuboid1Size.z,
						tempCuboid1Pos + cuboid1Normal * cuboid1Size.x - cuboid1Tangent * cuboid1Size.y + cuboid1BiTangent * cuboid1Size.z,
						tempCuboid1Pos + cuboid1Normal * cuboid1Size.x - cuboid1Tangent * cuboid1Size.y - cuboid1BiTangent * cuboid1Size.z,
						tempCuboid1Pos - cuboid1Normal * cuboid1Size.x + cuboid1Tangent * cuboid1Size.y + cuboid1BiTangent * cuboid1Size.z,
						tempCuboid1Pos - cuboid1Normal * cuboid1Size.x + cuboid1Tangent * cuboid1Size.y - cuboid1BiTangent * cuboid1Size.z,
						tempCuboid1Pos - cuboid1Normal * cuboid1Size.x - cuboid1Tangent * cuboid1Size.y + cuboid1BiTangent * cuboid1Size.z,
						tempCuboid1Pos - cuboid1Normal * cuboid1Size.x - cuboid1Tangent * cuboid1Size.y - cuboid1BiTangent * cuboid1Size.z,
						tempCuboid1Pos + cuboid1Normal * cuboid1Size.x,
						tempCuboid1Pos - cuboid1Normal * cuboid1Size.x,
						tempCuboid1Pos + cuboid1Tangent * cuboid1Size.y,
						tempCuboid1Pos - cuboid1Tangent * cuboid1Size.y,
						tempCuboid1Pos + cuboid1BiTangent * cuboid1Size.z,
						tempCuboid1Pos - cuboid1BiTangent * cuboid1Size.z
				};

				for (auto point : cuboid1Points)
				{
					auto inside = false;
					auto collisionPoint = glm::vec3(0.0f);
					const auto depth = calculateCuboidCuboidCollisionDepth(point, tempCuboid2Pos, tempCuboid2Axis, cuboid2Size, inside, collisionPoint);

					if (depth < 0.1f)
					{
						ManifoldPoint manPoint = {
							pCuboid2,
							pCuboid1,
							collisionPoint,
							collisionPoint,
							normalize(calculateCuboidCollisionNormal(tempCuboid1Pos, tempCuboid1Axis[0], tempCuboid1Axis[1], tempCuboid1Axis[2], cuboid1Size, collisionPoint)),
							currentTime,
							0.0f,
							CollisionType::COLLISION
						};

						pManifold->add(manPoint);
						return;
					}
				}

				const auto cuboid2Points = std::vector<glm::vec3>{
						tempCuboid2Pos + cuboid2Normal * cuboid2Size.x + cuboid2Tangent * cuboid2Size.y + cuboid2BiTangent * cuboid2Size.z,
						tempCuboid2Pos + cuboid2Normal * cuboid2Size.x + cuboid2Tangent * cuboid2Size.y - cuboid2BiTangent * cuboid2Size.z,
						tempCuboid2Pos + cuboid2Normal * cuboid2Size.x - cuboid2Tangent * cuboid2Size.y + cuboid2BiTangent * cuboid2Size.z,
						tempCuboid2Pos + cuboid2Normal * cuboid2Size.x - cuboid2Tangent * cuboid2Size.y - cuboid2BiTangent * cuboid2Size.z,
						tempCuboid2Pos - cuboid2Normal * cuboid2Size.x + cuboid2Tangent * cuboid2Size.y + cuboid2BiTangent * cuboid2Size.z,
						tempCuboid2Pos - cuboid2Normal * cuboid2Size.x + cuboid2Tangent * cuboid2Size.y - cuboid2BiTangent * cuboid2Size.z,
						tempCuboid2Pos - cuboid2Normal * cuboid2Size.x - cuboid2Tangent * cuboid2Size.y + cuboid2BiTangent * cuboid2Size.z,
						tempCuboid2Pos - cuboid2Normal * cuboid2Size.x - cuboid2Tangent * cuboid2Size.y - cuboid2BiTangent * cuboid2Size.z,
						tempCuboid2Pos + cuboid2Normal * cuboid2Size.x,
						tempCuboid2Pos - cuboid2Normal * cuboid2Size.x,
						tempCuboid2Pos + cuboid2Tangent * cuboid2Size.y,
						tempCuboid2Pos - cuboid2Tangent * cuboid2Size.y,
						tempCuboid2Pos + cuboid2BiTangent * cuboid2Size.z,
						tempCuboid2Pos - cuboid2BiTangent * cuboid2Size.z
				};

				for (auto point : cuboid2Points)
				{
					auto inside = false;
					auto collisionPoint = glm::vec3(0.0f);
					const auto depth = calculateCuboidCuboidCollisionDepth(point, tempCuboid1Pos, tempCuboid1Axis, cuboid1Size, inside, collisionPoint);

					if (depth < 0.1f)
					{
						ManifoldPoint manPoint = {
							pCuboid1,
							pCuboid2,
							collisionPoint,
							collisionPoint,
							normalize(calculateCuboidCollisionNormal(tempCuboid2Pos, tempCuboid2Axis[0], tempCuboid2Axis[1], tempCuboid2Axis[2], cuboid2Size, collisionPoint)),
							currentTime,
							0.0f,
							CollisionType::COLLISION
						};

						pManifold->add(manPoint);
						return;
					}
				}

				const auto cuboidPairs = std::vector<glm::ivec2>{
						glm::ivec2(0, 1),
						glm::ivec2(0, 2),
						glm::ivec2(0, 4),
						glm::ivec2(3, 1),
						glm::ivec2(3, 2),
						glm::ivec2(3, 7),
						glm::ivec2(5, 1),
						glm::ivec2(5, 4),
						glm::ivec2(5, 7),
						glm::ivec2(6, 2),
						glm::ivec2(6, 7),
						glm::ivec2(6, 4)
				};

				for (auto pair1 : cuboidPairs)
				{
					for (auto pair2 : cuboidPairs)
					{
						glm::vec3 collisionPoint1;
						glm::vec3 collisionPoint2;

						calculateClosestPointBetweenLines(cuboid1Points[pair1.x], cuboid1Points[pair1.y], cuboid2Points[pair2.x], cuboid2Points[pair2.y], collisionPoint1, collisionPoint2);

						{
							auto inside = false;
							auto collisionPoint = glm::vec3(0.0f);
							const auto depth = calculateCuboidCuboidCollisionDepth(collisionPoint1, tempCuboid2Pos, tempCuboid2Axis, cuboid2Size, inside, collisionPoint);

							if (depth < 0.1f)
							{
								ManifoldPoint manPoint = {
										pCuboid2,
										pCuboid1,
										collisionPoint,
										collisionPoint,
										normalize(calculateCuboidCollisionNormal(tempCuboid1Pos, tempCuboid1Axis[0], tempCuboid1Axis[1], tempCuboid1Axis[2], cuboid1Size, collisionPoint1)),
										currentTime,
										0.0f,
										CollisionType::COLLISION
								};

								pManifold->add(manPoint);
								return;
							}
						}

						{
							auto inside = false;
							auto collisionPoint = glm::vec3(0.0f);
							const auto depth = calculateCuboidCuboidCollisionDepth(collisionPoint2, tempCuboid1Pos, tempCuboid1Axis, cuboid1Size, inside, collisionPoint);

							if (depth < 0.1f)
							{
								ManifoldPoint manPoint = {
									pCuboid1,
									pCuboid2,
									collisionPoint,
									collisionPoint,
									normalize(calculateCuboidCollisionNormal(tempCuboid2Pos, tempCuboid2Axis[0], tempCuboid2Axis[1], tempCuboid2Axis[2], cuboid2Size, collisionPoint2)),
									currentTime,
									0.0f,
									CollisionType::COLLISION
								};

								pManifold->add(manPoint);
								return;
							}
						}
					}
				}

				timeStart = currentTime;
				currentTime = glm::mix(timeStart, timeEnd, 0.5f);
			}
		}
	}
}

void CollisionDetection::detectCollisionCuboidPlane(RigidBody* pCuboid, RigidBody* pPlane, ContactManifold* pManifold, float pLastCollisionTime)
{
	glm::vec3 cuboidPos;

	if (pCuboid->getCurrentUpdateTime() < pLastCollisionTime)
	{
		const auto interValue = (pLastCollisionTime - pCuboid->getCurrentUpdateTime()) / (1.0f - pCuboid->getCurrentUpdateTime());

		cuboidPos = pCuboid->getPos() + interValue * (pCuboid->getNewPos() - pCuboid->getPos());
	}
	else
	{
		cuboidPos = pCuboid->getPos();
	}

	auto planeMat = pPlane->getMatrix();

	auto planeCenter = glm::vec3(planeMat * glm::vec4(0, 0, 0, 1.0f));
	auto planeNormal = glm::vec3(planeMat * glm::vec4(0, 1, 0, 1.0f));
	auto planeTangent = glm::vec3(planeMat * glm::vec4(1, 0, 0, 1.0f));
	auto planeBiTangent = glm::vec3(planeMat * glm::vec4(0, 0, 1, 1.0f));

	planeNormal = normalize(planeNormal - planeCenter);
	planeTangent = normalize(planeTangent - planeCenter);
	planeBiTangent = normalize(planeBiTangent - planeCenter);

	planeCenter = glm::vec3(planeMat * glm::vec4(pPlane->getPos(), 1.0f));
	const auto planeSize = glm::vec3(0.001f, pPlane->getSize().x, pPlane->getSize().z);

	auto newPlaneMat = pPlane->getNewMatrix();

	auto newPlaneCenter = glm::vec3(newPlaneMat * glm::vec4(pPlane->getPos(), 1.0f));

	auto timeStart = pLastCollisionTime;
	auto currentTime = timeStart;
	auto timeEnd = 1.0f;
	auto firstTime = true;

	auto planeAxis = std::vector<glm::vec3>{
		planeNormal, planeTangent, planeBiTangent
	};

	const auto cuboidSize = pCuboid->getSize();
	const auto cuboidVelocity = pCuboid->getNewPos() - cuboidPos;

	if (dot(planeNormal, cuboidVelocity) > 0.0f)
	{
		//Moving away so dont collide
		return;
	}

	while (true)
	{
		if (firstTime)
		{
			//Time = 0
			{
				const auto interValue = (currentTime - pCuboid->getCurrentUpdateTime()) / (1.0f - pCuboid->getCurrentUpdateTime());
				const auto cuboidOrr = slerp(pCuboid->getOrientation(), pCuboid->getNewOrientation(), interValue);
				const auto cuboidOrrMat = toMat4(cuboidOrr);

				const auto cuboidNormal = glm::vec3(cuboidOrrMat * glm::vec4(0, 1, 0, 1.0f));
				const auto cuboidTangent = glm::vec3(cuboidOrrMat * glm::vec4(1, 0, 0, 1.0f));
				const auto cuboidBiTangent = glm::vec3(cuboidOrrMat * glm::vec4(0, 0, 1, 1.0f));

				const auto tempCuboidPos = mix(cuboidPos, pCuboid->getNewPos(), interValue);
				const auto tempPlanePos = mix(planeCenter, newPlaneCenter, interValue);

				const auto tempCuboidAxis = std::vector<glm::vec3>{
					cuboidNormal, cuboidTangent, cuboidBiTangent
				};

				const auto cuboidPoints = std::vector<glm::vec3>{
					tempCuboidPos + cuboidNormal * cuboidSize.x + cuboidTangent * cuboidSize.y + cuboidBiTangent * cuboidSize.z,
					tempCuboidPos + cuboidNormal * cuboidSize.x + cuboidTangent * cuboidSize.y - cuboidBiTangent * cuboidSize.z,
					tempCuboidPos + cuboidNormal * cuboidSize.x - cuboidTangent * cuboidSize.y + cuboidBiTangent * cuboidSize.z,
					tempCuboidPos + cuboidNormal * cuboidSize.x - cuboidTangent * cuboidSize.y - cuboidBiTangent * cuboidSize.z,
					tempCuboidPos - cuboidNormal * cuboidSize.x + cuboidTangent * cuboidSize.y + cuboidBiTangent * cuboidSize.z,
					tempCuboidPos - cuboidNormal * cuboidSize.x + cuboidTangent * cuboidSize.y - cuboidBiTangent * cuboidSize.z,
					tempCuboidPos - cuboidNormal * cuboidSize.x - cuboidTangent * cuboidSize.y + cuboidBiTangent * cuboidSize.z,
					tempCuboidPos - cuboidNormal * cuboidSize.x - cuboidTangent * cuboidSize.y - cuboidBiTangent * cuboidSize.z,
				};

				for (auto point : cuboidPoints)
				{
					glm::vec3 temp;
					if (calculateCuboidPointPlaneCollision(tempCuboidPos, point, tempPlanePos, planeAxis, planeSize, temp))
					{
						auto inside = false;
						auto collisionPoint = glm::vec3(0.0);
						const auto depth = calculateCuboidCuboidCollisionDepth(point, tempPlanePos, planeAxis, planeSize, inside, collisionPoint);

						ManifoldPoint manPoint = {
							pCuboid,
							pPlane,
							collisionPoint,
							collisionPoint,
							planeNormal,
							currentTime,
							depth,
							CollisionType::PENETRATION
						};

						pManifold->add(manPoint);
						return;
					}
				}

				const auto cuboidPairs = std::vector<glm::ivec2>{
					glm::ivec2(0, 1),
					glm::ivec2(0, 2),
					glm::ivec2(0, 4),
					glm::ivec2(3, 1),
					glm::ivec2(3, 2),
					glm::ivec2(3, 7),
					glm::ivec2(5, 1),
					glm::ivec2(5, 4),
					glm::ivec2(5, 7),
					glm::ivec2(6, 2),
					glm::ivec2(6, 7),
					glm::ivec2(6, 4)
				};

				for (auto pair : cuboidPairs)
				{
					glm::vec3 collisionPoint;
					if (calculateCuboidPointPlaneCollision(cuboidPoints[pair.x], cuboidPoints[pair.y], tempPlanePos, planeAxis, planeSize, collisionPoint))
					{
						{
							const auto dist = dot(cuboidPoints[pair.x] - tempPlanePos, planeNormal);

							if (dist < 0)
							{
								ManifoldPoint manPoint = {
									pCuboid,
									pPlane,
									collisionPoint,
									collisionPoint,
									planeNormal,
									currentTime,
									dist,
									CollisionType::PENETRATION
								};

								pManifold->add(manPoint);
								return;
							}
						}

						{
							const auto dist = dot(cuboidPoints[pair.y] - tempPlanePos, planeNormal);

							if (dist < 0)
							{
								ManifoldPoint manPoint = {
									pCuboid,
									pPlane,
									collisionPoint,
									collisionPoint,
									planeNormal,
									currentTime,
									dist,
									CollisionType::PENETRATION
								};

								pManifold->add(manPoint);
								return;
							}
						}
					}
				}

				std::vector<glm::vec3> planePoints =
				{
					tempPlanePos + planeSize.y * planeTangent + planeSize.z * planeBiTangent,
					tempPlanePos + planeSize.y * planeTangent - planeSize.z * planeBiTangent,
					tempPlanePos - planeSize.y * planeTangent + planeSize.z * planeBiTangent,
					tempPlanePos - planeSize.y * planeTangent - planeSize.z * planeBiTangent
				};

				for (auto point : planePoints)
				{
					auto inside = false;
					auto collisionPoint = glm::vec3(0.0);
					const auto depth = calculateCuboidCuboidCollisionDepth(point, tempCuboidPos, tempCuboidAxis, cuboidSize, inside, collisionPoint);

					if (inside)
					{
						ManifoldPoint manPoint = {
							pCuboid,
							pPlane,
							collisionPoint,
							collisionPoint,
							planeNormal,
							currentTime,
							depth,
							CollisionType::PENETRATION
						};

						pManifold->add(manPoint);
						return;
					}
				}
			}

			//Time = 1

			{
				const auto interValue = (timeEnd - pCuboid->getCurrentUpdateTime()) / (1.0f - pCuboid->getCurrentUpdateTime());
				const auto cuboidOrr = slerp(pCuboid->getOrientation(), pCuboid->getNewOrientation(), interValue);
				const auto cuboidOrrMat = toMat4(cuboidOrr);

				const auto cuboidNormal = glm::vec3(cuboidOrrMat * glm::vec4(0, 1, 0, 1.0f));
				const auto cuboidTangent = glm::vec3(cuboidOrrMat * glm::vec4(1, 0, 0, 1.0f));
				const auto cuboidBiTangent = glm::vec3(cuboidOrrMat * glm::vec4(0, 0, 1, 1.0f));

				const auto tempCuboidPos = mix(cuboidPos, pCuboid->getNewPos(), interValue);
				const auto tempPlanePos = mix(planeCenter, newPlaneCenter, interValue);

				const auto tempCuboidAxis = std::vector<glm::vec3>{
					cuboidNormal, cuboidTangent, cuboidBiTangent
				};

				const auto cuboidPoints = std::vector<glm::vec3>{
						tempCuboidPos + cuboidNormal * cuboidSize.x + cuboidTangent * cuboidSize.y + cuboidBiTangent * cuboidSize.z,
						tempCuboidPos + cuboidNormal * cuboidSize.x + cuboidTangent * cuboidSize.y - cuboidBiTangent * cuboidSize.z,
						tempCuboidPos + cuboidNormal * cuboidSize.x - cuboidTangent * cuboidSize.y + cuboidBiTangent * cuboidSize.z,
						tempCuboidPos + cuboidNormal * cuboidSize.x - cuboidTangent * cuboidSize.y - cuboidBiTangent * cuboidSize.z,
						tempCuboidPos - cuboidNormal * cuboidSize.x + cuboidTangent * cuboidSize.y + cuboidBiTangent * cuboidSize.z,
						tempCuboidPos - cuboidNormal * cuboidSize.x + cuboidTangent * cuboidSize.y - cuboidBiTangent * cuboidSize.z,
						tempCuboidPos - cuboidNormal * cuboidSize.x - cuboidTangent * cuboidSize.y + cuboidBiTangent * cuboidSize.z,
						tempCuboidPos - cuboidNormal * cuboidSize.x - cuboidTangent * cuboidSize.y - cuboidBiTangent * cuboidSize.z,
				};

				bool ret = true;
				for (auto point : cuboidPoints)
				{
					glm::vec3 temp;
					if (calculateCuboidPointPlaneCollision(tempCuboidPos, point, tempPlanePos, planeAxis, planeSize, temp))
					{
						ret = false;
						break;
					}
				}

				if (ret)
				{
					const auto cuboidPairs = std::vector<glm::ivec2>{
						glm::ivec2(0, 1),
						glm::ivec2(0, 2),
						glm::ivec2(0, 4),
						glm::ivec2(3, 1),
						glm::ivec2(3, 2),
						glm::ivec2(3, 7),
						glm::ivec2(5, 1),
						glm::ivec2(5, 4),
						glm::ivec2(5, 7),
						glm::ivec2(6, 2),
						glm::ivec2(6, 7),
						glm::ivec2(6, 4)
					};

					for (auto pair : cuboidPairs)
					{
						glm::vec3 collisionPoint;
						if (calculateCuboidPointPlaneCollision(cuboidPoints[pair.x], cuboidPoints[pair.y], tempPlanePos, planeAxis, planeSize, collisionPoint))
						{
							ret = false;
							break;
						}
					}
				}

				if (ret)
				{
					std::vector<glm::vec3> planePoints =
					{
						tempPlanePos + planeSize.y * planeTangent + planeSize.z * planeBiTangent,
						tempPlanePos + planeSize.y * planeTangent - planeSize.z * planeBiTangent,
						tempPlanePos - planeSize.y * planeTangent + planeSize.z * planeBiTangent,
						tempPlanePos - planeSize.y * planeTangent - planeSize.z * planeBiTangent
					};

					for (auto point : planePoints)
					{
						auto inside = false;
						auto collisionPoint = glm::vec3(0.0);
						const auto depth = calculateCuboidCuboidCollisionDepth(point, tempCuboidPos, tempCuboidAxis, cuboidSize, inside, collisionPoint);

						if (inside)
						{
							ret = false;
							break;
						}
					}
				}

				if (ret)
				{
					return;
				}

				currentTime = glm::mix(timeStart, timeEnd, 0.5f);
				firstTime = false;
			}
		}
		else //Not First Time
		{
			const auto interValue = (currentTime - pCuboid->getCurrentUpdateTime()) / (1.0f - pCuboid->getCurrentUpdateTime());
			const auto cuboidOrr = slerp(pCuboid->getOrientation(), pCuboid->getNewOrientation(), interValue);
			const auto cuboidOrrMat = toMat4(cuboidOrr);

			const auto cuboidNormal = glm::vec3(cuboidOrrMat * glm::vec4(0, 1, 0, 1.0f));
			const auto cuboidTangent = glm::vec3(cuboidOrrMat * glm::vec4(1, 0, 0, 1.0f));
			const auto cuboidBiTangent = glm::vec3(cuboidOrrMat * glm::vec4(0, 0, 1, 1.0f));

			const auto tempCuboidPos = mix(cuboidPos, pCuboid->getNewPos(), interValue);
			const auto tempPlanePos = mix(planeCenter, newPlaneCenter, interValue);

			const auto tempCuboidAxis = std::vector<glm::vec3>{
				cuboidNormal, cuboidTangent, cuboidBiTangent
			};

			const auto cuboidPoints = std::vector<glm::vec3>{
				tempCuboidPos + cuboidNormal * cuboidSize.x + cuboidTangent * cuboidSize.y + cuboidBiTangent * cuboidSize.z,
				tempCuboidPos + cuboidNormal * cuboidSize.x + cuboidTangent * cuboidSize.y - cuboidBiTangent * cuboidSize.z,
				tempCuboidPos + cuboidNormal * cuboidSize.x - cuboidTangent * cuboidSize.y + cuboidBiTangent * cuboidSize.z,
				tempCuboidPos + cuboidNormal * cuboidSize.x - cuboidTangent * cuboidSize.y - cuboidBiTangent * cuboidSize.z,
				tempCuboidPos - cuboidNormal * cuboidSize.x + cuboidTangent * cuboidSize.y + cuboidBiTangent * cuboidSize.z,
				tempCuboidPos - cuboidNormal * cuboidSize.x + cuboidTangent * cuboidSize.y - cuboidBiTangent * cuboidSize.z,
				tempCuboidPos - cuboidNormal * cuboidSize.x - cuboidTangent * cuboidSize.y + cuboidBiTangent * cuboidSize.z,
				tempCuboidPos - cuboidNormal * cuboidSize.x - cuboidTangent * cuboidSize.y - cuboidBiTangent * cuboidSize.z,
			};

			bool collision = false;

			for (auto point : cuboidPoints)
			{
				glm::vec3 temp;
				if (calculateCuboidPointPlaneCollision(tempCuboidPos, point, tempPlanePos, planeAxis, planeSize, temp))
				{
					collision = true;
					break;
				}
			}

			if (!collision)
			{
				const auto cuboidPairs = std::vector<glm::ivec2>{
						glm::ivec2(0, 1),
						glm::ivec2(0, 2),
						glm::ivec2(0, 4),
						glm::ivec2(3, 1),
						glm::ivec2(3, 2),
						glm::ivec2(3, 7),
						glm::ivec2(5, 1),
						glm::ivec2(5, 4),
						glm::ivec2(5, 7),
						glm::ivec2(6, 2),
						glm::ivec2(6, 7),
						glm::ivec2(6, 4)
				};

				for (auto pair : cuboidPairs)
				{
					glm::vec3 collisionPoint;
					if (calculateCuboidPointPlaneCollision(cuboidPoints[pair.x], cuboidPoints[pair.y], tempPlanePos, planeAxis, planeSize, collisionPoint))
					{
						collision = true;
						break;
					}
				}
			}

			if (!collision)
			{
				std::vector<glm::vec3> planePoints =
				{
					tempPlanePos + planeSize.y * planeTangent + planeSize.z * planeBiTangent,
					tempPlanePos + planeSize.y * planeTangent - planeSize.z * planeBiTangent,
					tempPlanePos - planeSize.y * planeTangent + planeSize.z * planeBiTangent,
					tempPlanePos - planeSize.y * planeTangent - planeSize.z * planeBiTangent
				};

				for (auto point : planePoints)
				{
					auto inside = false;
					auto collisionPoint = glm::vec3(0.0);
					const auto depth = calculateCuboidCuboidCollisionDepth(point, tempCuboidPos, tempCuboidAxis, cuboidSize, inside, collisionPoint);

					if (inside)
					{
						collision = true;
						break;
					}
				}
			}

			if (collision)
			{
				timeEnd = currentTime;
				currentTime = glm::mix(timeStart, timeEnd, 0.5f);
			}
			else
			{
				for (auto point : cuboidPoints)
				{
					auto inside = false;
					auto collisionPoint = glm::vec3(0.0);
					const auto depth = calculateCuboidCuboidCollisionDepth(point, tempPlanePos, planeAxis, planeSize, inside, collisionPoint);

					if (depth < 0.0005f)
					{
						ManifoldPoint manPoint = {
							pCuboid,
							pPlane,
							collisionPoint,
							collisionPoint,
							planeNormal,
							currentTime,
							0.0f,
							CollisionType::COLLISION
						};

						pManifold->add(manPoint);
						return;
					}
				}

				const auto cuboidPairs = std::vector<glm::ivec2>{
						glm::ivec2(0, 1),
						glm::ivec2(0, 2),
						glm::ivec2(0, 4),
						glm::ivec2(3, 1),
						glm::ivec2(3, 2),
						glm::ivec2(3, 7),
						glm::ivec2(5, 1),
						glm::ivec2(5, 4),
						glm::ivec2(5, 7),
						glm::ivec2(6, 2),
						glm::ivec2(6, 7),
						glm::ivec2(6, 4)
				};

				for (auto pair : cuboidPairs)
				{
					glm::vec3 point;
					calculateCuboidPointPlaneCollision(cuboidPoints[pair.x], cuboidPoints[pair.y], tempPlanePos, planeAxis, planeSize, point);

					auto inside = false;
					auto collisionPoint = glm::vec3(0.0);
					const auto depth = calculateCuboidCuboidCollisionDepth(point, tempPlanePos, planeAxis, planeSize, inside, collisionPoint);

					if (depth < 0.0005f)
					{
						ManifoldPoint manPoint = {
						pCuboid,
						pPlane,
						collisionPoint,
						collisionPoint,
						planeNormal,
						currentTime,
						0.0f,
						CollisionType::COLLISION
						};

						pManifold->add(manPoint);
						return;
					}
				}

				std::vector<glm::vec3> planePoints =
				{
					tempPlanePos + planeSize.y * planeTangent + planeSize.z * planeBiTangent,
					tempPlanePos + planeSize.y * planeTangent - planeSize.z * planeBiTangent,
					tempPlanePos - planeSize.y * planeTangent + planeSize.z * planeBiTangent,
					tempPlanePos - planeSize.y * planeTangent - planeSize.z * planeBiTangent
				};

				for (auto point : planePoints)
				{
					auto inside = false;
					auto collisionPoint = glm::vec3(0.0);
					const auto depth = calculateCuboidCuboidCollisionDepth(point, tempCuboidPos, tempCuboidAxis, cuboidSize, inside, collisionPoint);

					if (depth < 0.0005f)
					{
						ManifoldPoint manPoint = {
						pCuboid,
						pPlane,
						collisionPoint,
						collisionPoint,
						planeNormal,
						currentTime,
						0.0f,
						CollisionType::COLLISION
						};

						pManifold->add(manPoint);
						return;
					}
				}

				timeStart = currentTime;
				currentTime = glm::mix(timeStart, timeEnd, 0.5f);
			}
		}
	}
}

void CollisionDetection::detectCollisionCuboidPlaneHoles(RigidBody* pCuboid, RigidBody* pPlaneHoles, ContactManifold* pManifold, float pLastCollisionTime)
{
	//TODO: Implement
}

void CollisionDetection::detectCollisionCuboidBowl(RigidBody* pCuboid, RigidBody* pBowl, ContactManifold* pManifold, float pLastCollisionTime)
{
	//TODO: Implement
}

void CollisionDetection::detectCollisionCuboidCylinder(RigidBody* pSphere, RigidBody* pCylinder, ContactManifold* pManifold, float pLastCollisionTime)
{
	//TODO: Implement
}

void CollisionDetection::detectCollisionSphereCylinder(RigidBody* pSphere, RigidBody* pCylinder, ContactManifold* pManifold, float pLastCollisionTime)
{
	glm::vec3 spherePos;

	if (pSphere->getCurrentUpdateTime() < pLastCollisionTime)
	{
		const auto interValue = (pLastCollisionTime - pSphere->getCurrentUpdateTime()) / (1.0f - pSphere->getCurrentUpdateTime());

		spherePos = mix(pSphere->getPos(), pSphere->getNewPos(), interValue);
	}
	else
	{
		spherePos = pSphere->getPos();
	}

	const auto sphereRadius = pSphere->getSize().x;

	//TODO: Double check
	const auto cylinderHeight = pCylinder->getSize().y * 0.5f;
	const auto cylinderRadius = pCylinder->getSize().x * 0.5f;

	bool rotate = true;
	
	glm::vec3 cylinderEndPoint1;

	glm::quat cylinderRotationStart;
	glm::quat cylinderRotationEnd;

	{
		const auto cylinderMat1 = pCylinder->getMatrix();
		const auto cylinderMat2 = pCylinder->getNewMatrix();

		auto cylinderCenter1 = glm::vec3(cylinderMat1 * glm::vec4(0, 0, 0, 1.0f));
		auto cylinderNormal1 = glm::vec3(cylinderMat1 * glm::vec4(0, 1, 0, 1.0f));

		cylinderNormal1 = normalize(cylinderNormal1 - cylinderCenter1);

		auto cylinder1Point1 = cylinderCenter1 + cylinderNormal1 * cylinderHeight;
		auto cylinder1Point2 = cylinderCenter1 - cylinderNormal1 * cylinderHeight;

		auto cylinderCenter2 = glm::vec3(cylinderMat2 * glm::vec4(0, 0, 0, 1.0f));
		auto cylinderNormal2 = glm::vec3(cylinderMat2 * glm::vec4(0, 1, 0, 1.0f));

		cylinderNormal2 = normalize(cylinderNormal2 - cylinderCenter2);

		auto cylinder2Point1 = cylinderCenter2 + cylinderNormal2 * cylinderHeight;
		auto cylinder2Point2 = cylinderCenter2 - cylinderNormal2 * cylinderHeight;

		cylinderEndPoint1 = (cylinder1Point2 + cylinder2Point2) / 2.0f;

		if (cylinder1Point1 == cylinder2Point1 && cylinder1Point2 == cylinder2Point2)
		{
			rotate = false;
		}
		else
		{
			cylinderRotationStart = angleAxis(glm::atan(cylinder1Point1.x, cylinder1Point1.z), glm::vec3(0, 1, 0));
			cylinderRotationEnd = angleAxis(glm::atan(cylinder2Point1.x, cylinder2Point1.z), glm::vec3(0, 1, 0));
		}
	}
	
	auto timeStart = pLastCollisionTime;
	auto currentTime = timeStart;
	auto timeEnd = 1.0f;
	auto firstTime = true;
	auto dista = 10.0f;
	
	while (true)
	{
		if (firstTime)
		{
			//Time = 0
			{
				glm::vec3 cylinderEndPoint2;

				if (rotate)
				{
					const auto interValue = (currentTime - pCylinder->getCurrentUpdateTime()) / (1.0f - pCylinder->getCurrentUpdateTime());
					const auto cylinderOrr = slerp(cylinderRotationStart, cylinderRotationEnd, interValue);
					const auto cylinderOrrMat = toMat4(cylinderOrr);

					const auto cylinderNormal = glm::vec3(cylinderOrrMat * glm::vec4(0, 0, 1, 1.0f));

					cylinderEndPoint2 = cylinderEndPoint1 + cylinderNormal * cylinderHeight * 2.0f;
				}
				else
				{
					cylinderEndPoint2 = cylinderEndPoint1 + glm::vec3(0, 1, 0) * cylinderHeight * 2.0f;
				}

				const auto cylinderCenter = (cylinderEndPoint1 + cylinderEndPoint2) / 2.0f;

				const auto interValue = (currentTime - pSphere->getCurrentUpdateTime()) / (1.0f - pSphere->getCurrentUpdateTime());
				const auto tempSpherePos = mix(spherePos, pSphere->getNewPos(), interValue);

				glm::vec3 collisionPoint;
				float depth;

				if (detectCollisionSphereCylinderStep(cylinderCenter, cylinderEndPoint1, cylinderEndPoint2, cylinderRadius, cylinderHeight, tempSpherePos, sphereRadius, depth, collisionPoint))
				{
					ManifoldPoint manPoint = {
						pSphere,
						pCylinder,
						collisionPoint,
						collisionPoint,
						normalize(tempSpherePos - collisionPoint),
						0.0f,
						sphereRadius - depth,
						CollisionType::PENETRATION
					};

					pManifold->add(manPoint);
					return;
				}
			}

			//Time = 1
			{
				glm::vec3 cylinderEndPoint2;

				if (rotate)
				{
					const auto interValue = (timeEnd - pCylinder->getCurrentUpdateTime()) / (1.0f - pCylinder->getCurrentUpdateTime());
					const auto cylinderOrr = slerp(cylinderRotationStart, cylinderRotationEnd, interValue);
					const auto cylinderOrrMat = toMat4(cylinderOrr);

					const auto cylinderNormal = glm::vec3(cylinderOrrMat * glm::vec4(0, 0, 1, 1.0f));

					cylinderEndPoint2 = cylinderEndPoint1 + cylinderNormal * cylinderHeight * 2.0f;
				}
				else
				{
					cylinderEndPoint2 = cylinderEndPoint1 + glm::vec3(0, 1, 0) * cylinderHeight * 2.0f;
				}

				const auto cylinderCenter = (cylinderEndPoint1 + cylinderEndPoint2) / 2.0f;

				const auto interValue = (timeEnd - pSphere->getCurrentUpdateTime()) / (1.0f - pSphere->getCurrentUpdateTime());
				const auto tempSpherePos = mix(spherePos, pSphere->getNewPos(), interValue);

				glm::vec3 collisionPoint;
				float depth;

				if (!detectCollisionSphereCylinderStep(cylinderCenter, cylinderEndPoint1, cylinderEndPoint2, cylinderRadius, cylinderHeight, tempSpherePos, sphereRadius, depth, collisionPoint))
				{
					return;
				}

				currentTime = glm::mix(timeStart, timeEnd, 0.5f);
				firstTime = false;
			}
		}
		else //Not First Time
		{
			glm::vec3 cylinderEndPoint2;

			if (rotate)
			{
				const auto interValue = (currentTime - pCylinder->getCurrentUpdateTime()) / (1.0f - pCylinder->getCurrentUpdateTime());
				const auto cylinderOrr = slerp(cylinderRotationStart, cylinderRotationEnd, interValue);
				const auto cylinderOrrMat = toMat4(cylinderOrr);

				const auto cylinderNormal = glm::vec3(cylinderOrrMat * glm::vec4(0, 0, 1, 1.0f));

				cylinderEndPoint2 = cylinderEndPoint1 + cylinderNormal * cylinderHeight * 2.0f;
			}
			else
			{
				cylinderEndPoint2 = cylinderEndPoint1 + glm::vec3(0, 1, 0) * cylinderHeight * 2.0f;
			}

			const auto cylinderCenter = (cylinderEndPoint1 + cylinderEndPoint2) / 2.0f;

			const auto interValue = (currentTime - pSphere->getCurrentUpdateTime()) / (1.0f - pSphere->getCurrentUpdateTime());
			const auto tempSpherePos = mix(spherePos, pSphere->getNewPos(), interValue);

			glm::vec3 collisionPoint;
			float depth;

			if (detectCollisionSphereCylinderStep(cylinderCenter, cylinderEndPoint1, cylinderEndPoint2, cylinderRadius, cylinderHeight, tempSpherePos, sphereRadius, depth, collisionPoint))
			{
				timeEnd = currentTime;
				currentTime = glm::mix(timeStart, timeEnd, 0.5f);
			}
			else
			{
				//top
				auto dist = length(tempSpherePos - collisionPoint) - sphereRadius;

				if (abs(dist) < 0.05f)
				{
					ManifoldPoint manPoint = {
						pSphere,
						pCylinder,
						collisionPoint,
						collisionPoint,
						normalize(tempSpherePos - collisionPoint),
						currentTime,
						0.0f,
						CollisionType::COLLISION
					};

					pManifold->add(manPoint);
					return;
				}

				dist = dist - cylinderRadius;

				if (abs(dist) < 0.05f)
				{
					ManifoldPoint manPoint = {
						pSphere,
						pCylinder,
						collisionPoint,
						collisionPoint,
						normalize(tempSpherePos - collisionPoint),
						currentTime,
						0.0f,
						CollisionType::COLLISION
					};

					pManifold->add(manPoint);
					return;
				}

				timeStart = currentTime;
				currentTime = glm::mix(timeStart, timeEnd, 0.5f);
			}
		}
	}
}

bool CollisionDetection::calculateCuboidPointPlaneCollision(glm::vec3 pCuboidCenter, glm::vec3 pCuboidPoint, glm::vec3 pPlaneCenter, std::vector<glm::vec3> pPlaneAxis, glm::vec3 pPlaneSize, glm::vec3 & pCollisionPoint)
{
	const auto ab = pCuboidPoint - pCuboidCenter;

	const auto d = dot(pPlaneAxis[0], pPlaneCenter);

	const auto t = (d - dot(pPlaneAxis[0], pCuboidCenter)) / dot(pPlaneAxis[0], ab);

	if (t >= 0.0f && t <= 1.0f)
	{
		pCollisionPoint = pCuboidCenter + t * ab;

		const auto sizeX = pPlaneSize.y;
		const auto sizeY = pPlaneSize.z;
		
		const auto dotX = dot(pCollisionPoint - pPlaneCenter, pPlaneAxis[1]);
		const auto dotY = dot(pCollisionPoint - pPlaneCenter, pPlaneAxis[2]);


		if (dotX <= sizeX && dotX >= -sizeX &&
			dotY <= sizeY && dotY >= -sizeY)
		{
			return true;
		}
		
		return false;
	}

	return false;
}

float CollisionDetection::calculateClosestPointBetweenLines(glm::vec3 pLine1Start, glm::vec3 pLine1End, glm::vec3 pLine2Start, glm::vec3 pLine2End, glm::vec3& pLine1Point, glm::vec3& pLine2Point)
{
	float s;
	float t;
	
	const auto d1 = pLine1End - pLine1Start;
	const auto d2 = pLine2End - pLine2Start;
	const auto r = pLine1Start - pLine1End;

	const auto a = dot(d1, d1);
	const auto e = dot(d2, d2);
	const auto f = dot(d2, r);

	if (a == 0.0f && e == 0.0f)
	{
		s = 0.0f;
		t = 0.0f;
		pLine1Point = pLine1Start;
		pLine2Point = pLine2Start;
		return length(pLine1Point - pLine2Point);
	}

	if (a == 0.0f)
	{
		s = 0.0f;
		t = f / e;
		t = glm::clamp(t, 0.0f, 1.0f);
	}
	else
	{
		const auto c = dot(d1, r);

		if (e == 0.0f)
		{
			t = 0.0f;
			s = glm::clamp(-c / a, 0.0f, 1.0f);
		}
		else
		{
			const auto b = dot(d1, d2);

			const auto denom = a * e - b * b;

			if (denom != 0.0f)
			{
				s = glm::clamp((b * f - c * e) / denom, 0.0f, 1.0f);
			}
			else
			{
				s = 0.0f;
			}

			t = (b * s + f) / e;

			if (t < 0.0f)
			{
				t = 0.0f;
				s = glm::clamp(-c / a, 0.0f, 1.0f);
			}
			else if (t > 1.0f)
			{
				t = 1.0f;
				s = glm::clamp((b - c) / a, 0.0f, 1.0f);
			}
		}
	}

	pLine1Point = pLine1Start + d1 * s;
	pLine2Point = pLine2Start + d2 * t;
	
	return length(pLine1Point - pLine2Point);
}

//https://www10.cs.fau.de/publications/theses/2010/Suenkel_BT_2010.pdf

bool CollisionDetection::detectCollisionSphereCylinderStep(glm::vec3 pCylinderCenter, glm::vec3 pCylinderEndPoint1, glm::vec3 pCylinderEndPoint2, float pCylinderRadius, float pCylinderHeight, glm::vec3 pSpherePos, float pSphereRadius, float & distance, glm::vec3 & pCollisionPoint)
{
	const auto cylinderNormal = normalize(pCylinderEndPoint1 - pCylinderEndPoint2);

	const auto proj = dot(cylinderNormal, pSpherePos - pCylinderCenter);

	if (proj > pCylinderHeight)
	{
		//Pass End Point 1

		auto endCapVector = cross(cross(cylinderNormal, pSpherePos - pCylinderEndPoint1), cylinderNormal);

		if (length(endCapVector) != 0)
		{
			endCapVector = normalize(endCapVector);
		}

		const auto lineStart = pCylinderEndPoint1 + endCapVector * pCylinderRadius;
		const auto lineEnd = pCylinderEndPoint1 - endCapVector * pCylinderRadius;

		glm::vec3 temp;

		calculateClosestPointBetweenLines(lineStart, lineEnd, pSpherePos, pSpherePos, pCollisionPoint, temp);

		distance = length(pCollisionPoint - pSpherePos);

		if (distance < 1.0f)
		{
			int i = 0;
		}

		return distance < pSphereRadius;
	}
	if (proj < -pCylinderHeight)
	{
		//Pass End Point 2

		auto endCapVector = cross(cross(cylinderNormal, pSpherePos - pCylinderEndPoint2), cylinderNormal);

		if (length(endCapVector) != 0)
		{
			endCapVector = normalize(endCapVector);
		}

		const auto lineStart = pCylinderEndPoint2 + endCapVector * pCylinderRadius;
		const auto lineEnd = pCylinderEndPoint2 - endCapVector * pCylinderRadius;

		glm::vec3 temp;

		calculateClosestPointBetweenLines(lineStart, lineEnd, pSpherePos, pSpherePos, pCollisionPoint, temp);

		distance = length(pCollisionPoint - pSpherePos);

		if (distance < 1.0f)
		{
			int i = 0;
		}
		
		return distance < pSphereRadius;
	}
	
	//With Cylinder End Points
	glm::vec3 temp;
	
	calculateClosestPointBetweenLines(pCylinderEndPoint1, pCylinderEndPoint2, pSpherePos, pSpherePos, pCollisionPoint, temp);

	distance = length(pCollisionPoint - pSpherePos);

	return distance < pCylinderRadius + pSphereRadius;
}
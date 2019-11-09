#pragma once

#include "RigidBody.h"

class Plane final : public RigidBody
{
public:
	Plane(float pMass, glm::vec3 pSize, glm::vec3 pPos, glm::vec3 pAngularVelocity, glm::vec3 pVelocity);
	~Plane() = default;

	Plane(const Plane &) = delete;
	Plane(Plane &&) = delete;
	Plane & operator= (const Plane &) = delete;
	Plane & operator= (Plane &&) = delete;

	void render(Shader * pShader) const override;
};


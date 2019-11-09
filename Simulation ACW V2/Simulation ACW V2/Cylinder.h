#pragma once

#include "RigidBody.h"
#include "Shader.h"

class Cylinder final : public RigidBody
{
public:
	Cylinder(glm::vec3 pSize, float pMass, glm::vec3 pPos, glm::vec3 pAngularVelocity, glm::vec3 pVelocity);
	~Cylinder() = default;

	Cylinder(const Cylinder &) = delete;
	Cylinder(Cylinder &&) = delete;
	Cylinder & operator= (const Cylinder &) = delete;
	Cylinder & operator= (Cylinder &&) = delete;

	void render(Shader * pShader) const override;
};
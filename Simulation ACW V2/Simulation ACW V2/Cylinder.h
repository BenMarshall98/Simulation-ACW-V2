#pragma once

#include "RigidBody.h"
#include "Shader.h"

class Cylinder final : public RigidBody
{
public:
	Cylinder(Vector3F pSize, float pMass, Vector3F pPos, Vector3F pAngularVelocity, Vector3F pVelocity);
	~Cylinder() = default;

	Cylinder(const Cylinder &) = delete;
	Cylinder(Cylinder &&) = delete;
	Cylinder & operator= (const Cylinder &) = delete;
	Cylinder & operator= (Cylinder &&) = delete;

	void render(Shader * pShader) const override;
};
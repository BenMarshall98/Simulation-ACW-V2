#pragma once

#include "Shader.h"
#include "RigidBody.h"

class Sphere final : public RigidBody
{
public:
	Sphere(float pRadius, float pMass, Vector3F pPos, Vector3F pAngularVelocity, Vector3F pVelocity);
	~Sphere();

	Sphere(const Sphere &) = delete;
	Sphere(Sphere &&) = delete;
	Sphere & operator= (const Sphere &) = delete;
	Sphere & operator= (Sphere &&) = delete;

	void render(Shader * pShader) const override;
};


#pragma once

#include "Vector3f.h"
#include "RigidBody.h"

class Plane final : public RigidBody
{
public:
	Plane(float pMass, Vector3F pSize, Vector3F pPos, Vector3F pAngularVelocity, Vector3F pVelocity);
	~Plane() = default;

	Plane(const Plane &) = delete;
	Plane(Plane &&) = delete;
	Plane & operator= (const Plane &) = delete;
	Plane & operator= (Plane &&) = delete;

	void render(Shader * pShader) const override;
};


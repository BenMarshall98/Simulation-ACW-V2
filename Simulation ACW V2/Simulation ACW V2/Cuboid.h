#pragma once

#include "Shader.h"
#include "RigidBody.h"

class Cuboid final : public RigidBody
{
	Cuboid(Vector3F pSize, float pMass, Vector3F pPos, Vector3F pAngularVelocity, Vector3F pVelocity);
	~Cuboid();
	
	Cuboid(const Cuboid &) = delete;
	Cuboid(Cuboid &&) = delete;
	Cuboid & operator= (const Cuboid &) = delete;
	Cuboid & operator= (Cuboid &&) = delete;
	
	void render(Shader * pShader) const override;
};
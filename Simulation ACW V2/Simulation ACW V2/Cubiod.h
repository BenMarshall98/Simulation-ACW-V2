#pragma once

#include "Shader.h"
#include "RigidBody.h"

class Cubiod final : public RigidBody
{
	Cubiod(Vector3F pSize, float pMass, Vector3F pPos, Vector3F pAngularVelocity, Vector3F pVelocity);
	~Cubiod();
	
	Cubiod(const Cubiod &) = delete;
	Cubiod(Cubiod &&) = delete;
	Cubiod & operator= (const Cubiod &) = delete;
	Cubiod & operator= (Cubiod &&) = delete;
	
	void render(Shader & pShader) const override;
};
#pragma once

#include "Shader.h"
#include "RigidBody.h"

class Bowl final : public RigidBody
{
public:
	Bowl(Vector3F pSize, float pMass, Vector3F pPos, Vector3F pAngulerVelocity, Vector3F pVelocity);
	~Bowl() = default;

	Bowl(const Bowl &) = delete;
	Bowl(Bowl &&) = delete;
	Bowl & operator= (const Bowl &) = delete;
	Bowl & operator= (Bowl &&) = delete;

	void render(Shader * pShader) const override;
};
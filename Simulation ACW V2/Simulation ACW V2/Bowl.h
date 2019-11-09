#pragma once

#include "Shader.h"
#include "RigidBody.h"

class Bowl final : public RigidBody
{
public:
	Bowl(glm::vec3 pSize, float pMass, glm::vec3 pPos, glm::vec3 pAngularVelocity, glm::vec3 pVelocity);
	~Bowl() = default;

	Bowl(const Bowl &) = delete;
	Bowl(Bowl &&) = delete;
	Bowl & operator= (const Bowl &) = delete;
	Bowl & operator= (Bowl &&) = delete;

	void render(Shader * pShader) const override;
};
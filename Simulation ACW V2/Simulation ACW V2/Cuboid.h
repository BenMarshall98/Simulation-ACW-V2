#pragma once

#include "Shader.h"
#include "RigidBody.h"

class Cuboid final : public RigidBody
{
public:
	Cuboid(glm::vec3 pSize, float pMass, glm::vec3 pPos, glm::vec3 pAngularVelocity, glm::vec3 pVelocity);
	~Cuboid();
	
	Cuboid(const Cuboid &) = delete;
	Cuboid(Cuboid &&) = delete;
	Cuboid & operator= (const Cuboid &) = delete;
	Cuboid & operator= (Cuboid &&) = delete;
	
	void render(Shader * pShader) const override;
};
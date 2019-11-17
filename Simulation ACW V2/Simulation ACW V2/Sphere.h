#pragma once

#include "Shader.h"
#include "RigidBody.h"

class Sphere final : public RigidBody
{
private:
	glm::vec3 mColour;
	
public:
	Sphere(float pRadius, float pMass, glm::vec3 pPos, glm::vec3 pAngularVelocity, glm::vec3 pVelocity, glm::vec3 pColour);
	~Sphere();

	Sphere(const Sphere &) = delete;
	Sphere(Sphere &&) = delete;
	Sphere & operator= (const Sphere &) = delete;
	Sphere & operator= (Sphere &&) = delete;

	void render(Shader * pShader) const override;
};


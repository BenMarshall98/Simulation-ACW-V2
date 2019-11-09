#pragma once

#include "Shader.h"
#include "RigidBody.h"

class PlaneHoles : public RigidBody
{
public:
	PlaneHoles(glm::vec3 pSize, float pMass, glm::vec3 pPos, glm::vec3 pAngularVelocity, glm::vec3 pVelocity);
	~PlaneHoles();

	PlaneHoles(const PlaneHoles &) = delete;
	PlaneHoles(PlaneHoles &&) = delete;
	PlaneHoles & operator= (const PlaneHoles &) = delete;
	PlaneHoles & operator= (PlaneHoles &&) = delete;

	void render(Shader * pShader) const override;
};


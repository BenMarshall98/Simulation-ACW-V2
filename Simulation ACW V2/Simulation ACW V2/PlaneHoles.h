#pragma once

#include "Vector2f.h"
#include "Vector3f.h"
#include "Shader.h"
#include "RigidBody.h"

class PlaneHoles : public RigidBody
{
public:
	PlaneHoles(Vector3F pSize, float pMass, Vector3F pPos, Vector3F pRotationAxis, float pRotationAngle, Vector3F pVelocity);
	~PlaneHoles();

	PlaneHoles(const PlaneHoles &) = delete;
	PlaneHoles(PlaneHoles &&) = delete;
	PlaneHoles & operator= (const PlaneHoles &) = delete;
	PlaneHoles & operator= (PlaneHoles &&) = delete;

	void render(Shader * pShader) const override;
};


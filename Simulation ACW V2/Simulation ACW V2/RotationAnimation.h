#pragma once

#include "SceneGraphAnimation.h"
#include "Vector3f.h"
#include <functional>

class RotationAnimation final : public SceneGraphAnimation
{
	std::function<void(Vector3F&, float&, float, float)> mFunction;
	float mAngleSpeed = 45;

public:
	RotationAnimation(std::function<void(Vector3F &, float&, float, float)> pFunction, char pKey1, char pKey2);
	~RotationAnimation();

	void keyPressed(char pKey) override;

	void callFunction(Vector3F & pRotationAxis, float & pRotationAngle, float pDt);
};
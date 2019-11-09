#pragma once

#include "glm/glm.hpp"
#include "SceneGraphAnimation.h"
#include <functional>

class RotationAnimation final : public SceneGraphAnimation
{
	std::function<void(glm::vec3&, float&, float, float)> mFunction;
	float mAngleSpeed = 45;

public:
	RotationAnimation(std::function<void(glm::vec3 &, float&, float, float)> pFunction, char pKey1, char pKey2);
	~RotationAnimation();

	void keyPressed(char pKey) override;

	void callFunction(glm::vec3 & pRotationAxis, float & pRotationAngle, float pDt);
};
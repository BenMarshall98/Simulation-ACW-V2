#pragma once

#include "SceneGraphAnimation.h"
#include "glm/glm.hpp"
#include <functional>

class TranslationAnimation final : public SceneGraphAnimation
{
	std::function<void(glm::vec3&, float, bool)> mFunction;
	bool mDirection = true;

public:
	TranslationAnimation(std::function<void(glm::vec3 &, float, bool)> pFunction, char pKey1, char pKey2);
	~TranslationAnimation();

	void keyPressed(char pKey) override;

	void callFunction(glm::vec3 & pTranslation, float pDt);
};
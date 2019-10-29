#pragma once

#include "SceneGraphAnimation.h"
#include "Vector3f.h"
#include <functional>

class TranslationAnimation final : public SceneGraphAnimation
{
	std::function<void(Vector3F&, float, bool)> mFunction;
	bool mDirection = true;

public:
	TranslationAnimation(std::function<void(Vector3F &, float, bool)> pFunction, char pKey1, char pKey2);
	~TranslationAnimation();

	void keyPressed(char pKey) override;

	void callFunction(Vector3F & pTranslation, float pDt);
};
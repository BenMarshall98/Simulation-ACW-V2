#include "TranslationAnimation.h"

TranslationAnimation::TranslationAnimation(std::function<void(Vector3F&, float, bool)> pFunction, char pKey1, char pKey2) :
	SceneGraphAnimation(pKey1, pKey2), mFunction(pFunction)
{
}

TranslationAnimation::~TranslationAnimation()
{
}

void TranslationAnimation::keyPressed(char pKey)
{
	if (pKey == mKey1)
	{
		mDirection = true;
	}
	else if (pKey == mKey2)
	{
		mDirection = false;
	}
}

void TranslationAnimation::callFunction(Vector3F& pTranslation, float pDt)
{
	mFunction(pTranslation, pDt, mDirection);
}
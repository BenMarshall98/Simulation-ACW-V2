#include "RotationAnimation.h"

RotationAnimation::RotationAnimation(std::function<void(glm::vec3&, float&, float, float)> pFunction, char pKey1, char pKey2) :
	SceneGraphAnimation(pKey1, pKey2), mFunction(pFunction)
{
}

RotationAnimation::~RotationAnimation()
{
}

void RotationAnimation::keyPressed(char pKey)
{
	if (pKey == mKey1)
	{
		mAngleSpeed++;
	}
	else if (pKey == mKey2)
	{
		mAngleSpeed--;
	}
}

void RotationAnimation::callFunction(glm::vec3& pRotationAxis, float& pRotationAngle, float pDt)
{
	mFunction(pRotationAxis, pRotationAngle, mAngleSpeed, pDt);
}
#include "RotationNode.h"

RotationNode::RotationNode(glm::vec3 pRotationAxis, float mRotationAngle, RotationAnimation * pAnimation) :
	mRotationAxis(pRotationAxis), mRotationAngle(mRotationAngle), mAnimation(pAnimation)
{
}

RotationNode::~RotationNode()
{

}

glm::mat4 RotationNode::updateNode(float pDt)
{
	if (mAnimation)
	{
		mAnimation->callFunction(mRotationAxis, mRotationAngle, pDt);
	}

	return glm::mat4::createRotation(mRotationAxis, mRotationAngle);
}
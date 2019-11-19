#include "RotationNode.h"
#include <glm/ext/matrix_transform.inl>

RotationNode::RotationNode(glm::vec3 pRotationAxis, float mRotationAngle, RotationAnimation * pAnimation) :
	mRotationAxis(pRotationAxis), mRotationAngle(glm::radians(mRotationAngle)), mAnimation(pAnimation)
{
}

RotationNode::~RotationNode()
{
	delete mAnimation;
}

glm::mat4 RotationNode::updateNode(float pDt)
{
	if (mAnimation)
	{
		mAnimation->callFunction(mRotationAxis, mRotationAngle, pDt);
	}

	return rotate(glm::mat4(1.0f), mRotationAngle, mRotationAxis);
}
#include "RotationNode.h"

RotationNode::RotationNode(Vector3F pRotationAxis, float mRotationAngle, RotationAnimation * pAnimation) :
	mRotationAxis(pRotationAxis), mRotationAngle(mRotationAngle), mAnimation(pAnimation)
{
}

RotationNode::~RotationNode()
{

}

Matrix4F RotationNode::updateNode(float pDt)
{
	if (mAnimation)
	{
		mAnimation->callFunction(mRotationAxis, mRotationAngle, pDt);
	}

	return Matrix4F::createRotation(mRotationAxis, mRotationAngle);
}
#include "TranslationNode.h"

TranslationNode::TranslationNode(Vector3F pTranslation, TranslationAnimation * pAnimation) :
	mTranslation(pTranslation), mAnimation(pAnimation)
{
}

TranslationNode::~TranslationNode()
{
}

Matrix4F TranslationNode::updateNode(float pDt)
{
	if (mAnimation)
	{
		mAnimation->callFunction(mTranslation, pDt);
	}

	return Matrix4F::createTranslation(mTranslation);
}

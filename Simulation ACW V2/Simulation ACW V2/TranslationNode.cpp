#include "TranslationNode.h"

TranslationNode::TranslationNode(glm::vec3 pTranslation, TranslationAnimation * pAnimation) :
	mTranslation(pTranslation), mAnimation(pAnimation)
{
}

TranslationNode::~TranslationNode()
{
}

glm::mat4 TranslationNode::updateNode(float pDt)
{
	if (mAnimation)
	{
		mAnimation->callFunction(mTranslation, pDt);
	}

	return glm::mat4::createTranslation(mTranslation);
}

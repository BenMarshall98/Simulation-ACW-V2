#include "TranslationNode.h"
#include <glm/ext/matrix_transform.inl>

TranslationNode::TranslationNode(glm::vec3 pTranslation, TranslationAnimation * pAnimation) :
	mTranslation(pTranslation), mAnimation(pAnimation)
{
}

TranslationNode::~TranslationNode()
{
	delete mAnimation;
}

glm::mat4 TranslationNode::updateNode(float pDt)
{
	if (mAnimation)
	{
		mAnimation->callFunction(mTranslation, pDt);
	}

	return translate(glm::mat4(1.0f), mTranslation);
}

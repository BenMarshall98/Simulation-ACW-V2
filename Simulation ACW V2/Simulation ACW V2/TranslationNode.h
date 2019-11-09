#pragma once

#include "SceneGraphNode.h"
#include "TranslationAnimation.h"

class TranslationNode final : public SceneGraphNode
{
	TranslationAnimation * mAnimation;
	glm::vec3 mTranslation;

public:
	TranslationNode(glm::vec3 pTranslation, TranslationAnimation * pAnimation = nullptr);
	~TranslationNode();

	glm::mat4 updateNode(float pDt) override;
};
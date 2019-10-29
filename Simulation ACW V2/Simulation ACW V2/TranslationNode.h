#pragma once

#include "SceneGraphNode.h"
#include "TranslationAnimation.h"

class TranslationNode final : public SceneGraphNode
{
	TranslationAnimation * mAnimation;
	Vector3F mTranslation;

public:
	TranslationNode(Vector3F pTranslation, TranslationAnimation * pAnimation = nullptr);
	~TranslationNode();

	Matrix4F updateNode(float pDt) override;
};
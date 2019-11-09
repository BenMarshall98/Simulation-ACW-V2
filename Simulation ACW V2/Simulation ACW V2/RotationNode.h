#pragma once

#include "SceneGraphNode.h"
#include "RotationAnimation.h"

class RotationNode final : public SceneGraphNode
{
	RotationAnimation * mAnimation;
	glm::vec3 mRotationAxis;
	float mRotationAngle;

public:
	RotationNode(glm::vec3 pRotationAxis, float mRotationAngle, RotationAnimation * pAnimation = nullptr);
	~RotationNode();

	glm::mat4 updateNode(float pDt) override;
};
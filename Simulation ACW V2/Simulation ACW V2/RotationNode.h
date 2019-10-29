#pragma once

#include "SceneGraphNode.h"
#include "RotationAnimation.h"

class RotationNode final : public SceneGraphNode
{
	RotationAnimation * mAnimation;
	Vector3F mRotationAxis;
	float mRotationAngle;

public:
	RotationNode(Vector3F pRotationAxis, float mRotationAngle, RotationAnimation * pAnimation = nullptr);
	~RotationNode();

	Matrix4F updateNode(float pDt) override;
};
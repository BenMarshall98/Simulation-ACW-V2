#pragma once

#include "SceneGraphNode.h"

class IdentityNode final : public SceneGraphNode
{
public:
	IdentityNode();
	~IdentityNode();

	Matrix4F updateNode(float pDt) override;
};


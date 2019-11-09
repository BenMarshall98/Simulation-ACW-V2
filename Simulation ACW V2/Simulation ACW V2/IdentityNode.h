#pragma once

#include "SceneGraphNode.h"

class IdentityNode final : public SceneGraphNode
{
public:
	IdentityNode();
	~IdentityNode();

	glm::mat4 updateNode(float pDt) override;
};


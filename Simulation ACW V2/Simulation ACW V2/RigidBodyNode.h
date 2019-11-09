#pragma once

#include "SceneGraphNode.h"
#include "RigidBody.h"

class RigidBodyNode final : public SceneGraphNode
{
	RigidBody * mRigidBody;

public:
	RigidBodyNode(RigidBody * pRigidBody);
	~RigidBodyNode();

	glm::mat4 updateNode(float pDt) override;
};


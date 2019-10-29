#pragma once

#include "SceneGraphNode.h"
#include "RigidBody.h"

class RigidBodyNode final : public SceneGraphNode
{
	RigidBody * mRigidBody;

public:
	RigidBodyNode(RigidBody * pRigidBody);
	~RigidBodyNode();

	Matrix4F updateNode(float pDt) override;
};


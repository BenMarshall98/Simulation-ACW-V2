#include "RigidBodyNode.h"

RigidBodyNode::RigidBodyNode(RigidBody * pRigidBody) :
	mRigidBody(pRigidBody)
{
	mRigidBody->setSceneGraphNode(this);
}

RigidBodyNode::~RigidBodyNode()
{
	delete mRigidBody;
}

glm::mat4 RigidBodyNode::updateNode(float pDt)
{
	return glm::mat4(1.0f);
}
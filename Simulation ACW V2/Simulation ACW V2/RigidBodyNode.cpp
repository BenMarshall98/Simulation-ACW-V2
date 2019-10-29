#include "RigidBodyNode.h"

RigidBodyNode::RigidBodyNode(RigidBody * pRigidBody) :
	mRigidBody(pRigidBody)
{
	mRigidBody->setSceneGraphNode(this);
}

RigidBodyNode::~RigidBodyNode()
{
}

Matrix4F RigidBodyNode::updateNode(float pDt)
{
	return Matrix4F::createIdentity();
}
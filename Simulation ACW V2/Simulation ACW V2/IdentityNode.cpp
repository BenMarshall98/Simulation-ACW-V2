#include "IdentityNode.h"

IdentityNode::IdentityNode()
{
}

IdentityNode::~IdentityNode()
{
}

glm::mat4 IdentityNode::updateNode(float pDt)
{
	return glm::mat4(1.0f);
}

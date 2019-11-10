#include "IdentityNode.h"

IdentityNode::IdentityNode()
{
}

IdentityNode::~IdentityNode()
{
}

glm::mat4 IdentityNode::updateNode(float)
{
	return glm::mat4(1.0f);
}

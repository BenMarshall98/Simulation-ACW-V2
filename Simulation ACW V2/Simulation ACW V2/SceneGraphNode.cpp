#include "SceneGraphNode.h"

SceneGraphNode::SceneGraphNode()
{

}

SceneGraphNode::~SceneGraphNode()
{

}

void SceneGraphNode::addChild(SceneGraphNode* pChild)
{
	mChildren.push_back(pChild);
}

void SceneGraphNode::updateSceneGraph(float pDt, Matrix4F pWorldMatrix)
{
	mCurrentMatrix = pWorldMatrix * updateNode(pDt);

	for (auto& child : mChildren)
	{
		child->updateSceneGraph(pDt, mCurrentMatrix);
	}
}
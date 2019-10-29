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
	mUpdateMatrix = pWorldMatrix * updateNode(pDt);

	for (auto& child : mChildren)
	{
		child->updateSceneGraph(pDt, mUpdateMatrix);
	}
}

void SceneGraphNode::swap()
{
	mRenderMatrix = mUpdateMatrix;

	for (auto& child : mChildren)
	{
		child->swap();
	}
}

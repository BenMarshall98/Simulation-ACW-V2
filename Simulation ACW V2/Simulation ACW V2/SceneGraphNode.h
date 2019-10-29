#pragma once
#include "Matrix4f.h"

#include <vector>

class SceneGraphNode
{
	std::vector<SceneGraphNode *> mChildren;

protected:
	Matrix4F mUpdateMatrix;
	Matrix4F mRenderMatrix;
	

public:
	SceneGraphNode();
	virtual ~SceneGraphNode();

	void addChild(SceneGraphNode * pChild);
	void updateSceneGraph(float pDt, Matrix4F pWorldMatrix);
	virtual Matrix4F updateNode(float pDt) = 0;
	void swap();

	Matrix4F getUpdateMatrix()
	{
		return mUpdateMatrix;
	}

	Matrix4F getRenderMatrix()
	{
		return mRenderMatrix;
	}
};
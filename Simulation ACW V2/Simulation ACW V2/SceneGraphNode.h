#pragma once
#include "Matrix4f.h"

#include <vector>

class SceneGraphNode
{
	std::vector<SceneGraphNode *> mChildren;

protected:
	Matrix4F mCurrentMatrix;

public:
	SceneGraphNode();
	virtual ~SceneGraphNode();

	void addChild(SceneGraphNode * pChild);
	void updateSceneGraph(float pDt, Matrix4F pWorldMatrix);
	virtual Matrix4F updateNode(float pDt) = 0;

	Matrix4F getMatrix()
	{
		return mCurrentMatrix;
	}
};
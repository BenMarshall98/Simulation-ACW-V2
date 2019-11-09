#pragma once

#include "glm/glm.hpp"

#include <vector>

class SceneGraphNode
{
	std::vector<SceneGraphNode *> mChildren;

protected:
	glm::mat4 mUpdateMatrix;
	glm::mat4 mRenderMatrix;
	

public:
	SceneGraphNode();
	virtual ~SceneGraphNode();

	void addChild(SceneGraphNode * pChild);
	void updateSceneGraph(float pDt, glm::mat4 pWorldMatrix);
	virtual glm::mat4 updateNode(float pDt) = 0;
	void swap();

	glm::mat4 getUpdateMatrix()
	{
		return mUpdateMatrix;
	}

	glm::mat4 getRenderMatrix()
	{
		return mRenderMatrix;
	}
};
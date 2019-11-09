#pragma once

#include <vector>
#include "Model.h"

class OctreeModel : public Model
{
	unsigned int mVao, mEbo, mVbo;
	std::vector<glm::vec3> positions;
	std::vector<unsigned int> indices;

public:
	OctreeModel();
	~OctreeModel();

	void render() override;
};


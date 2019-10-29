#pragma once

#include <vector>
#include "Vector3f.h"
#include "Model.h"

class OctreeModel : public Model
{
	unsigned int mVao, mEbo, mVbo;
	std::vector<Vector3F> positions;
	std::vector<unsigned int> indices;

public:
	OctreeModel();
	~OctreeModel();

	void render() override;
};


#pragma once

#include <vector>

#include "glm/glm.hpp"

class Model
{
public:
	Model(std::vector<glm::vec3> pPositions,
		std::vector<glm::vec2> pTexCoords, std::vector<unsigned int> pIndices);
	Model();
	virtual ~Model() = default;

	Model(const Model &) = delete;
	Model(Model &&) = delete;
	Model & operator= (const Model &) = delete;
	Model & operator= (Model &&) = delete;

	virtual void render();

	static Model * createSphere();
	static Model * createPlane();
	static Model * createCylinder();
	static Model * createBowl();
	static Model * createPlaneWithHoles();
	static Model * createCube();

private:
	unsigned int mVao, mEbo;
	unsigned int mVbo[2];

	std::vector<glm::vec3> mPosition;
	std::vector<glm::vec2> mTexCoords;
	std::vector<unsigned int> mIndices;

protected:
	static Model * mLastModel;
};


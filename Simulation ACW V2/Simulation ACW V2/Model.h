#pragma once

#include <vector>
#include "Vector3f.h"
#include "Vector2f.h"

class Model
{
public:
	Model(std::vector<Vector3F> pPositions,
		std::vector<Vector2F> pTexCoords, std::vector<unsigned int> pIndices);
	Model();
	~Model() = default;

	Model(const Model &) = delete;
	Model(Model &&) = delete;
	Model & operator= (const Model &) = delete;
	Model & operator= (Model &&) = delete;

	virtual void render();

	static Model * createSphere();
	static Model * createPlane();
	static Model * createCylinder();
	static Model * CreateBowl();
	static Model * CreatePlaneWithHoles();

private:
	unsigned int mVao, mEbo;
	unsigned int mVbo[2];

	std::vector<Vector3F> mPosition;
	std::vector<Vector2F> mTexCoords;
	std::vector<unsigned int> mIndices;

protected:
	static Model * lastModel;
};


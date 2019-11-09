#include "OctreeModel.h"
#include "gl.h"

OctreeModel::OctreeModel() : Model()
{
	positions = {
		Vector3F(1.0f, 1.0f, 1.0f),
		Vector3F(1.0f, 1.0f, -1.0f),
		Vector3F(1.0f, -1.0f, 1.0f),
		Vector3F(1.0f, -1.0f, -1.0f),
		Vector3F(-1.0f, 1.0f, 1.0f),
		Vector3F(-1.0f, 1.0f, -1.0f),
		Vector3F(-1.0f, -1.0f, 1.0f),
		Vector3F(-1.0f, -1.0f, -1.0f)
	};

	indices = {
		0, 1,
		0, 2,
		2, 3,
		1, 3,
		4, 0,
		1, 5,
		2, 6,
		3, 7,
		4, 5,
		4, 6,
		6, 7,
		7, 5
	};

	glGenVertexArrays(1, &mVao);
	glGenBuffers(1, &mVbo);
	glGenBuffers(1, &mEbo);

	glBindVertexArray(mVao);

	glBindBuffer(GL_ARRAY_BUFFER, mVbo);
	glBufferData(GL_ARRAY_BUFFER, positions.size() * sizeof(Vector3F), &positions[0], GL_STATIC_DRAW);

	glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 3 * sizeof(float), nullptr);
	glEnableVertexAttribArray(0);


	glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, mEbo);
	glBufferData(GL_ELEMENT_ARRAY_BUFFER, indices.size() * sizeof(unsigned int), &indices[0], GL_STATIC_DRAW);

	glBindBuffer(GL_ARRAY_BUFFER, 0);

	glBindVertexArray(0);
}

OctreeModel::~OctreeModel()
{

}

void OctreeModel::render()
{
	if (this != mLastModel)
	{
		glBindVertexArray(mVao);
		mLastModel = this;
	}

	glDrawElements(GL_LINES, indices.size(), GL_UNSIGNED_INT, nullptr);
}


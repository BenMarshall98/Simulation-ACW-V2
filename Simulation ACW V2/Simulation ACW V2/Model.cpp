#include "Model.h"
#include <utility>
#include <vector>
#include "Vector3f.h"
#include "Vector2f.h"
#include "gl.h"
#include <cmath>
#include <corecrt_math_defines.h>

Model * Model::lastModel = nullptr;

Model::Model(std::vector<Vector3F> pPositions, std::vector<Vector2F> pTexCoords,
	std::vector<unsigned int> pIndices) : mPosition(std::move(pPositions)), mTexCoords(std::move(pTexCoords)),
	mIndices(std::move(pIndices))
{
	glGenVertexArrays(1, &mVao);
	glGenBuffers(2, mVbo);
	glGenBuffers(1, &mEbo);

	glBindVertexArray(mVao);

	glBindBuffer(GL_ARRAY_BUFFER, mVbo[0]);
	glBufferData(GL_ARRAY_BUFFER, mPosition.size() * sizeof(Vector3F), &mPosition[0], GL_STATIC_DRAW);

	glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 3 * sizeof(float), nullptr);
	glEnableVertexAttribArray(0);

	glBindBuffer(GL_ARRAY_BUFFER, mVbo[1]);
	glBufferData(GL_ARRAY_BUFFER, mTexCoords.size() * sizeof(Vector2F), &mTexCoords[0], GL_STATIC_DRAW);

	glVertexAttribPointer(1, 2, GL_FLOAT, GL_FALSE, 2 * sizeof(float), nullptr);
	glEnableVertexAttribArray(1);

	glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, mEbo);
	glBufferData(GL_ELEMENT_ARRAY_BUFFER, mIndices.size() * sizeof(unsigned int), &mIndices[0], GL_STATIC_DRAW);

	glBindBuffer(GL_ARRAY_BUFFER, 0);

	glBindVertexArray(0);
}

Model::Model()
{

}

void Model::render()
{
	if (this != lastModel)
	{
		glBindVertexArray(mVao);
		lastModel = this;
	}

	glDrawElements(GL_TRIANGLES, mIndices.size(), GL_UNSIGNED_INT, nullptr);
}

Model* Model::createSphere()
{
	static Model * model = nullptr;

	if (model)
	{
		return model;
	}

	const auto segments = 30;

	std::vector<Vector3F> positions;
	std::vector<Vector2F> texCoords;
	std::vector<unsigned int> indices;

	const auto angle = static_cast<float>(M_PI) / segments;

	for (auto i = 0; i <= segments; i++)
	{
		for (auto j = 0; j < segments * 2; j++)
		{
			const auto x = cos(j * angle) * sin(i * angle);
			const auto y = sin(j * angle) * sin(i * angle);
			const auto z = cos(i * angle);

			positions.emplace_back(Vector3F(x, y, z));

			const auto u = 1.0f / (segments * 2) * j;
			const auto v = 1.0f / segments * i;

			texCoords.emplace_back(Vector2F(u, v));
		}
	}

	auto count = 0;
	auto limit = segments * 2 * 2;

	for (auto i = 0; i < segments * 2; i++)
	{
		indices.push_back(count);
		indices.push_back(count + segments * 2);

		if (count + segments * 2 + 1 == limit)
		{
			indices.push_back(count + 1);
		}
		else
		{
			indices.push_back(count + segments * 2 + 1);
		}

		count++;
	}

	for (auto i = 1; i < segments - 1; i++)
	{
		limit += segments * 2;

		for (auto j = 0; j < segments * 2; j++)
		{
			indices.push_back(count);
			indices.push_back(count + segments * 2);

			if (count + segments * 2 + 1 == limit)
			{
				indices.push_back(count + 1);
			}
			else
			{
				indices.push_back(count + segments * 2 + 1);
			}

			indices.push_back(count);

			if (count + segments * 2 + 1 == limit)
			{
				indices.push_back(count + 1);
				indices.push_back(count + 1 - segments * 2);
			}
			else
			{
				indices.push_back(count + segments * 2 + 1);
				indices.push_back(count + 1);
			}
			count++;
		}
	}

	for (auto i = 0; i < segments * 2; i++)
	{
		indices.push_back(count);
		indices.push_back(count + segments * 2);

		if (count + 1 == limit)
		{
			indices.push_back(count + 1 - segments * 2);
		}
		else
		{
			indices.push_back(count + 1);
		}
		count++;
	}

	model = new Model(positions, texCoords, indices);
	return model;
}

Model* Model::createPlane()
{
	static Model * model = nullptr;

	if (model)
	{
		return model;
	}

	const std::vector<Vector3F> positions = {
		Vector3F(1, 0, 1),
		Vector3F(1, 0, -1),
		Vector3F(-1, 0, -1),
		Vector3F(-1, 0, 1)
	};

	const std::vector<Vector2F> texCoords = {
		Vector2F(1, 1),
		Vector2F(1, 0),
		Vector2F(0, 0),
		Vector2F(0, 1)
	};

	const std::vector<unsigned int> indices = {
		0, 1, 3,
		1, 2, 3
	};

	model = new Model(positions, texCoords, indices);
	return model;
}

Model* Model::createCylinder()
{
	static Model* model = nullptr;

	if (model)
	{
		return model;
	}

	std::vector<Vector3F> positions;
	std::vector<Vector2F> texCoords;
	std::vector<unsigned int> indices;

	const auto segments = 30;

	const auto angle = 2.0f * static_cast<float>(M_PI) / segments;

	positions.emplace_back(Vector3F(0.0f, 0.5f, 0.0f));
	texCoords.emplace_back(Vector2F(0.5f, 0.5f));

	for (auto i = 0; i <= segments; i++)
	{
		const auto x = 0.5f * cos(angle * i);
		const auto z = 0.5f * sin(angle * i);

		positions.emplace_back(Vector3F(x, 0.5f, z));
		texCoords.emplace_back(Vector2F(0.5f + x, 0.5f + z));
	}

	for (auto i = 0; i <= segments; i++)
	{
		const auto x = 0.5f * cos(angle * i);
		const auto z = 0.5f * sin(angle * i);

		positions.emplace_back(Vector3F(x, 0.5f, z));
		texCoords.emplace_back(Vector2F(0.5f + x, 0.5f + z));
	}

	for (auto i = 0; i <= segments; i++)
	{
		const auto x = 0.5f * cos(angle * i);
		const auto z = 0.5f * sin(angle * i);

		positions.emplace_back(Vector3F(x, -0.5f, z));
		texCoords.emplace_back(Vector2F(0.5f + x, 0.5f + z));
	}

	positions.emplace_back(Vector3F(0.0f, -0.5f, 0.0f));
	texCoords.emplace_back(Vector2F(0.5f, 0.5f));

	for (auto i = 0; i <= segments; i++)
	{
		const auto x = 0.5f * cos(angle * i);
		const auto z = 0.5f * sin(angle * i);

		positions.emplace_back(Vector3F(x, -0.5f, z));
		texCoords.emplace_back(Vector2F(0.5f + x, 0.5f + z));
	}

	auto count = 1;

	for (auto i = 0; i < segments; i++)
	{
		indices.push_back(0);
		indices.push_back(count);
		indices.push_back(count + 1);
		count++;
	}

	count += 2;

	for (auto i = 0; i < segments; i++)
	{
		indices.push_back(count);
		indices.push_back(count + segments);
		indices.push_back(count + segments + 1);

		indices.push_back(count);
		indices.push_back(count + segments + 1);
		indices.push_back(count + 1);
		count++;
	}

	count += segments;

	for (auto i = 0; i <= segments + 1; i++)
	{
		indices.push_back(static_cast<unsigned int>(positions.size()) - segments - 2);
		indices.push_back(count);
		indices.push_back(count + 1);
		count++;
	}

	model = new Model(positions, texCoords, indices);

	return model;
}

Model* Model::CreatePlaneWithHoles()
{
	static Model * model = nullptr;

	if (model)
	{
		return model;
	}

	std::vector<Vector3F> positions = {
		Vector3F(-5, 0, 5),
		Vector3F(-4, 0, 5),
		Vector3F(-2, 0, 5),
		Vector3F(-1, 0, 5),
		Vector3F(1, 0, 5),
		Vector3F(2, 0, 5),
		Vector3F(4, 0, 5),
		Vector3F(5, 0, 5),

		Vector3F(-5, 0, 4),
		Vector3F(-4, 0, 4),
		Vector3F(-2, 0, 4),
		Vector3F(-1, 0, 4),
		Vector3F(1, 0, 4),
		Vector3F(2, 0, 4),
		Vector3F(4, 0, 4),
		Vector3F(5, 0, 4),

		Vector3F(-5, 0, 2),
		Vector3F(-4, 0, 2),
		Vector3F(-2, 0, 2),
		Vector3F(-1, 0, 2),
		Vector3F(1, 0, 2),
		Vector3F(2, 0, 2),
		Vector3F(4, 0, 2),
		Vector3F(5, 0, 2),

		Vector3F(-5, 0, 1),
		Vector3F(-4, 0, 1),
		Vector3F(-2, 0, 1),
		Vector3F(-1, 0, 1),
		Vector3F(1, 0, 1),
		Vector3F(2, 0, 1),
		Vector3F(4, 0, 1),
		Vector3F(5, 0, 1),

		Vector3F(-5, 0, -1),
		Vector3F(-4, 0, -1),
		Vector3F(-2, 0, -1),
		Vector3F(-1, 0, -1),
		Vector3F(1, 0, -1),
		Vector3F(2, 0, -1),
		Vector3F(4, 0, -1),
		Vector3F(5, 0, -1),

		Vector3F(-5, 0, -2),
		Vector3F(-4, 0, -2),
		Vector3F(-2, 0, -2),
		Vector3F(-1, 0, -2),
		Vector3F(1, 0, -2),
		Vector3F(2, 0, -2),
		Vector3F(4, 0, -2),
		Vector3F(5, 0, -2),

		Vector3F(-5, 0, -4),
		Vector3F(-4, 0, -4),
		Vector3F(-2, 0, -4),
		Vector3F(-1, 0, -4),
		Vector3F(1, 0, -4),
		Vector3F(2, 0, -4),
		Vector3F(4, 0, -4),
		Vector3F(5, 0, -4),

		Vector3F(-5, 0, -5),
		Vector3F(-4, 0, -5),
		Vector3F(-2, 0, -5),
		Vector3F(-1, 0, -5),
		Vector3F(1, 0, -5),
		Vector3F(2, 0, -5),
		Vector3F(4, 0, -5),
		Vector3F(5, 0, -5)
	};

	const std::vector<Vector3F> centersPosition = {
		Vector3F(-3, 0, 3),
		Vector3F(0, 0, 3),
		Vector3F(3, 0, 3),
		Vector3F(-3, 0, 0),
		Vector3F(3, 0, 0),
		Vector3F(-3, 0, -3),
		Vector3F(0, 0, -3),
		Vector3F(3, 0, -3),
	};

	const auto segments = 20;

	static_assert(segments % 4 == 0, "Number of segments must be mod 4");

	const auto angle = static_cast<float>(2 * M_PI) / segments;

	for (auto center : centersPosition)
	{
		for (auto j = 0; j < segments; j++)
		{
			const auto x = cos(j * angle);
			const auto z = sin(j * angle);

			positions.push_back(center + Vector3F(x, 0, z));
		}
	}

	std::vector<Vector2F> texCoords = {
		Vector2F(0.0f, 1.0f),
		Vector2F(0.1f, 1.0f),
		Vector2F(0.3f, 1.0f),
		Vector2F(0.4f, 1.0f),
		Vector2F(0.6f, 1.0f),
		Vector2F(0.7f, 1.0f),
		Vector2F(0.9f, 1.0f),
		Vector2F(1.0f, 1.0f),

		Vector2F(0.0f, 0.9f),
		Vector2F(0.1f, 0.9f),
		Vector2F(0.3f, 0.9f),
		Vector2F(0.4f, 0.9f),
		Vector2F(0.6f, 0.9f),
		Vector2F(0.7f, 0.9f),
		Vector2F(0.9f, 0.9f),
		Vector2F(1.0f, 0.9f),

		Vector2F(0.0f, 0.7f),
		Vector2F(0.1f, 0.7f),
		Vector2F(0.3f, 0.7f),
		Vector2F(0.4f, 0.7f),
		Vector2F(0.6f, 0.7f),
		Vector2F(0.7f, 0.7f),
		Vector2F(0.9f, 0.7f),
		Vector2F(1.0f, 0.7f),

		Vector2F(0.0f, 0.6f),
		Vector2F(0.1f, 0.6f),
		Vector2F(0.3f, 0.6f),
		Vector2F(0.4f, 0.6f),
		Vector2F(0.6f, 0.6f),
		Vector2F(0.7f, 0.6f),
		Vector2F(0.9f, 0.6f),
		Vector2F(1.0f, 0.6f),

		Vector2F(0.0f, 0.4f),
		Vector2F(0.1f, 0.4f),
		Vector2F(0.3f, 0.4f),
		Vector2F(0.4f, 0.4f),
		Vector2F(0.6f, 0.4f),
		Vector2F(0.7f, 0.4f),
		Vector2F(0.9f, 0.4f),
		Vector2F(1.0f, 0.4f),

		Vector2F(0.0f, 0.3f),
		Vector2F(0.1f, 0.3f),
		Vector2F(0.3f, 0.3f),
		Vector2F(0.4f, 0.3f),
		Vector2F(0.6f, 0.3f),
		Vector2F(0.7f, 0.3f),
		Vector2F(0.9f, 0.3f),
		Vector2F(1.0f, 0.3f),

		Vector2F(0.0f, 0.1f),
		Vector2F(0.1f, 0.1f),
		Vector2F(0.3f, 0.1f),
		Vector2F(0.4f, 0.1f),
		Vector2F(0.6f, 0.1f),
		Vector2F(0.7f, 0.1f),
		Vector2F(0.9f, 0.1f),
		Vector2F(1.0f, 0.1f),

		Vector2F(0.0f, 0.0f),
		Vector2F(0.1f, 0.0f),
		Vector2F(0.3f, 0.0f),
		Vector2F(0.4f, 0.0f),
		Vector2F(0.6f, 0.0f),
		Vector2F(0.7f, 0.0f),
		Vector2F(0.9f, 0.0f),
		Vector2F(1.0f, 0.0f)
	};

	const std::vector<Vector2F> centersTexCoords = {
		Vector2F(0.2f, 0.8f),
		Vector2F(0.5f, 0.8f),
		Vector2F(0.8f, 0.8f),
		Vector2F(0.2f, 0.5f),
		Vector2F(0.8f, 0.5f),
		Vector2F(0.2f, 0.2f),
		Vector2F(0.5f, 0.2f),
		Vector2F(0.8f, 0.2f)
	};

	for (auto center : centersTexCoords)
	{
		for (auto j = 0; j < segments; j++)
		{
			const auto u = 0.1f * cos(j * angle);
			const auto v = 0.1f * sin(j * angle);

			texCoords.push_back(center + Vector2F(u, v));
		}
	}

	std::vector<unsigned int> indices =
	{
		0, 8, 9,
		0, 9, 1,
		1, 9, 10,
		1, 10, 2,
		2, 10, 11,
		2, 11, 3,
		3, 11, 12,
		3, 12, 4,
		4, 12, 13,
		4, 13, 5,
		5, 13, 14,
		5, 14, 6,
		6, 14, 15,
		6, 15, 7,

		8, 16, 17,
		8, 17, 9,
		10, 18, 19,
		10, 19, 11,
		12, 20, 21,
		12, 21, 13,
		14, 22, 23,
		14, 23, 15,

		16, 24, 25,
		16, 25, 17,
		17, 25, 26,
		17, 26, 18,
		18, 26, 27,
		18, 27, 19,
		19, 27, 28,
		19, 28, 20,
		20, 28, 29,
		20, 29, 21,
		21, 29, 30,
		21, 30, 22,
		22, 30, 31,
		22, 31, 23,

		24, 32, 33,
		24, 33, 25,
		26, 34, 35,
		26, 35, 27,
		27, 35, 36,
		27, 36, 28,
		28, 36, 37,
		28, 37, 29,
		30, 38, 39,
		30, 39, 31,

		32, 40, 41,
		32, 41, 33,
		33, 41, 42,
		33, 42, 34,
		34, 42, 43,
		34, 43, 35,
		35, 43, 44,
		35, 44, 36,
		36, 44, 45,
		36, 45, 37,
		37, 45, 46,
		37, 46, 38,
		38, 46, 47,
		38, 47, 39,

		40, 48, 49,
		40, 49, 41,
		42, 50, 51,
		42, 51, 43,
		44, 52, 53,
		44, 53, 45,
		46, 54, 55,
		46, 55, 47,

		48, 56, 57,
		48, 57, 49,
		49, 57, 58,
		49, 58, 50,
		50, 58, 59,
		50, 59, 51,
		51, 59, 60,
		51, 60, 52,
		52, 60, 61,
		52, 61, 53,
		53, 61, 62,
		53, 62, 54,
		54, 62, 63,
		54, 63, 55
	};

	const std::vector<unsigned int> centerIndices =
	{
		10, 9, 17, 18,
		12, 11, 19, 20,
		14, 13, 21, 22,
		26, 25, 33, 34,
		30, 29, 37, 38,
		42, 41, 49, 50,
		44, 43, 51, 52,
		46, 45, 53, 54
	};

	int count = 1;

	for (auto center : centerIndices)
	{
		for (auto j = 0; j < segments / 4; j++)
		{
			indices.push_back(center);

			if (count % segments == 0)
			{
				indices.push_back(63 + count - segments + 1);
			}
			else
			{
				indices.push_back(63 + count + 1);
			}

			indices.push_back(63 + count);

			count++;
		}
	}

	model = new Model(positions, texCoords, indices);

	return model;
}

Model* Model::CreateBowl()
{
	static Model * model = nullptr;

	if (model)
	{
		return model;
	}

	std::vector<Vector3F> positions;
	std::vector<Vector2F> texCoords;

	const int segments = 20;

	const auto maxAngle = acos(0.8f);
	const auto angle1 = static_cast<float>(2 * M_PI) / segments;
	const auto angle2 = maxAngle / segments;

	for (auto i = 0; i <= segments; i++)
	{
		for (auto j = 0; j < segments * 2; j++)
		{
			const auto x = cos(j * angle1) * sin(i * angle2);
			const auto y = sin(j * angle1) * sin(i * angle2);
			const auto z = cos(i * angle2);

			positions.emplace_back(Vector3F(x, y, z));

			const auto u = 1.0f / (segments * 2) * j;
			const auto v = 1.0f / segments * i;

			texCoords.emplace_back(Vector2F(u, v));
		}
	}

	std::vector<unsigned int> indices;

	auto count = 0;
	auto limit = segments * 2 * 2;

	for (auto i = 0; i < segments * 2; i++)
	{
		indices.push_back(count);
		indices.push_back(count + segments * 2);

		if (count + segments * 2 + 1 == limit)
		{
			indices.push_back(count + 1);
		}
		else
		{
			indices.push_back(count + segments * 2 + 1);
		}

		count++;
	}

	for (auto i = 1; i < segments - 1; i++)
	{
		limit += segments * 2;

		for (auto j = 0; j < segments * 2; j++)
		{
			indices.push_back(count);
			indices.push_back(count + segments * 2);

			if (count + segments * 2 + 1 == limit)
			{
				indices.push_back(count + 1);
			}
			else
			{
				indices.push_back(count + segments * 2 + 1);
			}

			indices.push_back(count);

			if (count + segments * 2 + 1 == limit)
			{
				indices.push_back(count + 1);
				indices.push_back(count + 1 - segments * 2);
			}
			else
			{
				indices.push_back(count + segments * 2 + 1);
				indices.push_back(count + 1);
			}
			count++;
		}
	}

	model = new Model(positions, texCoords, indices);

	return model;
}


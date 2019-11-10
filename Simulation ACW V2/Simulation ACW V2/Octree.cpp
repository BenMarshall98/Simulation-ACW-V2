#include "Octree.h"
#include "Game.h"
#include <algorithm>
#include <iostream>

//TODO: Change the size to reflect the actual size of the balls

OctreeModel * Octree::model = nullptr;// new OctreeModel();
Shader * Octree::shader = nullptr;// new Shader("octreeVertex.vert", "octreeFragment.frag");

Octree::Octree(glm::vec3 pCenter, glm::vec3 pSize, Octree * pParent) : center(pCenter), size(pSize), parent(pParent)
{
	if (!model)
	{
		model = new OctreeModel();
		shader = new Shader("octreeVertex.vert", "octreeFragment.frag");
	}
}

Octree::~Octree()
{
	for (int i = 0; i < neighbours.size(); i++)
	{
		neighbours[i]->DeleteNeighbour(this);
	}
}

bool Octree::AddRigidBody(RigidBody * rigidBody)
{
	glm::vec3 bodyPos = rigidBody->getPos();

	if (bodyPos.x < center.x - size.x ||
		bodyPos.x > center.x + size.x ||
		bodyPos.y < center.y - size.y ||
		bodyPos.y > center.y + size.y ||
		bodyPos.z < center.z - size.z ||
		bodyPos.z > center.z + size.z)
	{
		return false;
	}

	if (size.x > 2 &&
		size.y > 2 &&
		size.z > 2)
	{
		if (children[0])
		{
			for (int i = 0; i < 8; i++)
			{
				if (children[i]->AddRigidBody(rigidBody))
				{
					break;
				}
			}
		}
		else
		{
			glm::vec3 childSize = size / 2.0f;
			glm::vec3 centers[8] =
			{
				center + glm::vec3(childSize.x * 1.0f, childSize.y * 1.0f, childSize.z * 1.0f),
				center + glm::vec3(childSize.x * 1.0f, childSize.y * 1.0f, childSize.z * -1.0f),
				center + glm::vec3(childSize.x * 1.0f, childSize.y * -1.0f, childSize.z * 1.0f),
				center + glm::vec3(childSize.x * 1.0f, childSize.y * -1.0f, childSize.z * -1.0f),
				center + glm::vec3(childSize.x * -1.0f, childSize.y * 1.0f, childSize.z * 1.0f),
				center + glm::vec3(childSize.x * -1.0f, childSize.y * 1.0f, childSize.z * -1.0f),
				center + glm::vec3(childSize.x * -1.0f, childSize.y * -1.0f, childSize.z * 1.0f),
				center + glm::vec3(childSize.x * -1.0f, childSize.y * -1.0f, childSize.z * -1.0f)
			};


			bool found = false;
			for (int i = 0; i < 8; i++)
			{
				children[i] = new Octree(centers[i], childSize, this);
				if (!found && children[i]->AddRigidBody(rigidBody))
				{
					found = true;
				}
			}

			for (int i = 0; i < 8; i++)
			{
				children[i]->GetNeighbours();
			}
		}
	}
	else
	{
		rigidBodies.push_back(rigidBody);
	}
	return true;
}

bool Octree::Render()
{
	bool render = false;

	if (rigidBodies.size() != 0)
	{
		render = true;
	}

	if (children[0])
	{
		for (int i = 0; i < 8; i++)
		{
			render |= children[i]->Render();
		}
	}

	//if (render)
	{
		shader->useShader();

		auto perspective = glm::perspective(45.0f, 600.0f / 600.0f, 0.1f, 1000.0f);
		auto view = Game::camera->getViewMatrix();

		auto perspectiveLocation = glGetUniformLocation(shader->getShaderId(), "perspective");
		glUniformMatrix4fv(perspectiveLocation, 1, GL_FALSE, &perspective[0][0]);

		auto viewLocation = glGetUniformLocation(shader->getShaderId(), "view");
		glUniformMatrix4fv(viewLocation, 1, GL_FALSE, &view[0][0]);

		const auto translation = translate(glm::mat4(1.0f), center);
		const auto scale = glm::scale(glm::mat4(1.0f), size);

		auto modelMat = translation * scale;

		const auto modelLocation = glGetUniformLocation(shader->getShaderId(), "model");
		glUniformMatrix4fv(modelLocation, 1, GL_FALSE, &modelMat[0][0]);
		model->render();
	}

	return render;
}

void Octree::UpdateTree()
{
	checked = false;
	for (int i = 0; i < rigidBodies.size(); i++)
	{
		glm::vec3 bodyPos = rigidBodies[i]->getPos();

		if (bodyPos.x < center.x - size.x ||
			bodyPos.x > center.x + size.x ||
			bodyPos.y < center.y - size.y ||
			bodyPos.y > center.y + size.y ||
			bodyPos.z < center.z - size.z ||
			bodyPos.z > center.z + size.z)
		{
			if (parent)
			{
				parent->MoveBody(rigidBodies[i]);
				rigidBodies.erase(rigidBodies.begin() + i);
				i--;
			}
			else
			{
				delete rigidBodies[i];
				rigidBodies.erase(rigidBodies.begin() + i);
				i--;

			}
		}
	}

	if (children[0])
	{
		for (int i = 0; i < 8; i++)
		{
			children[i]->UpdateTree();
		}

		bool deleteChildren = true;

		for (int i = 0; i < 8; i++)
		{
			if (children[i]->HasChildren() || children[i]->NumOfRigidBodies() > 0)
			{
				deleteChildren = false;
				break;
			}
		}

		if (deleteChildren)
		{
			for (int i = 0; i < 8; i++)
			{
				delete children[i];
				children[i] = nullptr;
			}
		}
	}
}

void Octree::MoveBody(RigidBody * rigidBody)
{
	glm::vec3 bodyPos = rigidBody->getPos();

	if (bodyPos.x < center.x - size.x ||
		bodyPos.x > center.x + size.x ||
		bodyPos.y < center.y - size.y ||
		bodyPos.y > center.y + size.y ||
		bodyPos.z < center.z - size.z ||
		bodyPos.z > center.z + size.z)
	{
		if (parent)
		{
			parent->MoveBody(rigidBody);
		}
		else
		{
			delete rigidBody;
		}
	}
	else
	{
		if (size.x > 2 &&
			size.y > 2 &&
			size.z > 2)
		{
			if (children[0])
			{
				for (int i = 0; i < 8; i++)
				{
					if (children[i]->AddRigidBody(rigidBody))
					{
						break;
					}
				}
			}
			else
			{
				glm::vec3 childSize = size / 2.0f;
				glm::vec3 centers[8] =
				{
					center + glm::vec3(childSize.x * 1.0f, childSize.y * 1.0f, childSize.z * 1.0f),
					center + glm::vec3(childSize.x * 1.0f, childSize.y * 1.0f, childSize.z * -1.0f),
					center + glm::vec3(childSize.x * 1.0f, childSize.y * -1.0f, childSize.z * 1.0f),
					center + glm::vec3(childSize.x * 1.0f, childSize.y * -1.0f, childSize.z * -1.0f),
					center + glm::vec3(childSize.x * -1.0f, childSize.y * 1.0f, childSize.z * 1.0f),
					center + glm::vec3(childSize.x * -1.0f, childSize.y * 1.0f, childSize.z * -1.0f),
					center + glm::vec3(childSize.x * -1.0f, childSize.y * -1.0f, childSize.z * 1.0f),
					center + glm::vec3(childSize.x * -1.0f, childSize.y * -1.0f, childSize.z * -1.0f)
				};


				bool found = false;
				for (int i = 0; i < 8; i++)
				{
					children[i] = new Octree(centers[i], childSize, this);
					if (!found && children[i]->AddRigidBody(rigidBody))
					{
						found = true;
					}
				}

				for (int i = 0; i < 8; i++)
				{
					children[i]->GetNeighbours();
				}
			}
		}
		else
		{
			rigidBodies.push_back(rigidBody);
		}
	}
}

void Octree::GetRigidBodies(std::vector<RigidBody*> & pRigidBodies)
{
	pRigidBodies.insert(pRigidBodies.end(), rigidBodies.begin(), rigidBodies.end());

	if (children[0])
	{
		for (int i = 0; i < 8; i++)
		{
			children[i]->GetRigidBodies(pRigidBodies);
		}
	}
}

void Octree::GetNeighbours()
{
	Octree * grandParent = parent;

	while (grandParent->parent)
	{
		grandParent = grandParent->parent;
	}

	grandParent->FindNeighbour(center, size, neighbours, this);
}

void Octree::FindNeighbour(glm::vec3 pCenter, glm::vec3 pSize, std::vector<Octree*>& pNeighbours, Octree* pNeighbour)
{
	if (this == pNeighbour)
	{
		return;
	}

	if (size.x > pSize.x)
	{
		for (int i = 0; i < 8 && children[i]; i++)
		{
			children[i]->FindNeighbour(pCenter, pSize, pNeighbours, pNeighbour);
		}
	}
	else if (size.x == pSize.x)
	{
		if (center == pCenter + glm::vec3(pSize.x * 2, pSize.y * 2, pSize.z * 2) ||
			center == pCenter + glm::vec3(pSize.x * 2, pSize.y * 2, pSize.z * 0) ||
			center == pCenter + glm::vec3(pSize.x * 2, pSize.y * 2, pSize.z * -2) ||
			center == pCenter + glm::vec3(pSize.x * 2, pSize.y * 0, pSize.z * 2) ||
			center == pCenter + glm::vec3(pSize.x * 2, pSize.y * 0, pSize.z * 0) ||
			center == pCenter + glm::vec3(pSize.x * 2, pSize.y * 0, pSize.z * -2) ||
			center == pCenter + glm::vec3(pSize.x * 2, pSize.y * -2, pSize.z * 2) ||
			center == pCenter + glm::vec3(pSize.x * 2, pSize.y * -2, pSize.z * 0) ||
			center == pCenter + glm::vec3(pSize.x * 2, pSize.y * -2, pSize.z * -2) ||
			center == pCenter + glm::vec3(pSize.x * 0, pSize.y * 2, pSize.z * 2) ||
			center == pCenter + glm::vec3(pSize.x * 0, pSize.y * 2, pSize.z * 0) ||
			center == pCenter + glm::vec3(pSize.x * 0, pSize.y * 2, pSize.z * -2) ||
			center == pCenter + glm::vec3(pSize.x * 0, pSize.y * 0, pSize.z * 2) ||
			center == pCenter + glm::vec3(pSize.x * 0, pSize.y * 0, pSize.z * -2) ||
			center == pCenter + glm::vec3(pSize.x * 0, pSize.y * -2, pSize.z * 2) ||
			center == pCenter + glm::vec3(pSize.x * 0, pSize.y * -2, pSize.z * 0) ||
			center == pCenter + glm::vec3(pSize.x * 0, pSize.y * -2, pSize.z * -2) ||
			center == pCenter + glm::vec3(pSize.x * -2, pSize.y * 2, pSize.z * 2) ||
			center == pCenter + glm::vec3(pSize.x * -2, pSize.y * 2, pSize.z * 0) ||
			center == pCenter + glm::vec3(pSize.x * -2, pSize.y * 2, pSize.z * -2) ||
			center == pCenter + glm::vec3(pSize.x * -2, pSize.y * 0, pSize.z * 2) ||
			center == pCenter + glm::vec3(pSize.x * -2, pSize.y * 0, pSize.z * 0) ||
			center == pCenter + glm::vec3(pSize.x * -2, pSize.y * 0, pSize.z * -2) ||
			center == pCenter + glm::vec3(pSize.x * -2, pSize.y * -2, pSize.z * 2) ||
			center == pCenter + glm::vec3(pSize.x * -2, pSize.y * -2, pSize.z * 0) ||
			center == pCenter + glm::vec3(pSize.x * -2, pSize.y * -2, pSize.z * -2))
		{
			if (std::find(pNeighbours.begin(), pNeighbours.end(), this) == pNeighbours.end())
			{
				pNeighbours.push_back(this);
			}

			if (std::find(neighbours.begin(), neighbours.end(), pNeighbour) == neighbours.end()) {
				neighbours.push_back(pNeighbour);
			}
		}
	}
}

void Octree::DeleteNeighbour(Octree* neighbour)
{
	auto i = std::find(neighbours.begin(), neighbours.end(), neighbour);

	if (i != neighbours.end())
	{
		neighbours.erase(i);
	}
}


void Octree::GetPossibleCollisions(std::vector<PossibleCollision>& pPossibleCollisions)
{
	if (rigidBodies.size() == 0)
	{
		if (children[0])
		{
			for (int i = 0; i < 8; i++)
			{
				children[i]->GetPossibleCollisions(pPossibleCollisions);
			}
		}
	}
	else
	{
		for (int i = 0; i < rigidBodies.size(); i++)
		{
			for (int j = i + 1; j < rigidBodies.size(); j++)
			{
				pPossibleCollisions.emplace_back(PossibleCollision{ rigidBodies[i], rigidBodies[j] });
			}
		}

		checked = true;

		for (int i = 0; i < rigidBodies.size(); i++)
		{
			for (int j = 0; j < neighbours.size(); j++)
			{
				if (!neighbours[j]->checked)
				{
					for (int k = 0; k < neighbours[j]->rigidBodies.size(); k++)
					{
						pPossibleCollisions.emplace_back(PossibleCollision{ rigidBodies[i], neighbours[j]->rigidBodies[k] });
					}
				}
			}
		}
	}
}

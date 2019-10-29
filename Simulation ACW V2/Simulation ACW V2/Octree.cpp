#include "Octree.h"
#include "Matrix4f.h"
#include "Game.h"
#include <algorithm>
#include <iostream>

//TODO: Change the size to reflect the actual size of the balls

OctreeModel * Octree::model = nullptr;// new OctreeModel();
Shader * Octree::shader = nullptr;// new Shader("octreeVertex.vert", "octreeFragment.frag");

Octree::Octree(Vector3F pCenter, Vector3F pSize, Octree * pParent) : center(pCenter), size(pSize), parent(pParent)
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
	Vector3F bodyPos = rigidBody->getPos();

	if (bodyPos.getX() < center.getX() - size.getX() ||
		bodyPos.getX() > center.getX() + size.getX() ||
		bodyPos.getY() < center.getY() - size.getY() ||
		bodyPos.getY() > center.getY() + size.getY() ||
		bodyPos.getZ() < center.getZ() - size.getZ() ||
		bodyPos.getZ() > center.getZ() + size.getZ())
	{
		return false;
	}

	if (size.getX() > 2 &&
		size.getY() > 2 &&
		size.getZ() > 2)
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
			Vector3F childSize = size / 2;
			Vector3F centers[8] =
			{
				center + Vector3F(childSize.getX() * 1.0f, childSize.getY() * 1.0f, childSize.getZ() * 1.0f),
				center + Vector3F(childSize.getX() * 1.0f, childSize.getY() * 1.0f, childSize.getZ() * -1.0f),
				center + Vector3F(childSize.getX() * 1.0f, childSize.getY() * -1.0f, childSize.getZ() * 1.0f),
				center + Vector3F(childSize.getX() * 1.0f, childSize.getY() * -1.0f, childSize.getZ() * -1.0f),
				center + Vector3F(childSize.getX() * -1.0f, childSize.getY() * 1.0f, childSize.getZ() * 1.0f),
				center + Vector3F(childSize.getX() * -1.0f, childSize.getY() * 1.0f, childSize.getZ() * -1.0f),
				center + Vector3F(childSize.getX() * -1.0f, childSize.getY() * -1.0f, childSize.getZ() * 1.0f),
				center + Vector3F(childSize.getX() * -1.0f, childSize.getY() * -1.0f, childSize.getZ() * -1.0f)
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

		auto perspective = Matrix4F::createPerspective(45.0f, 600.0f / 600.0f, 0.1f, 1000.0f);
		auto view = Game::camera->GetViewMatrix();

		auto perspectiveLocation = glGetUniformLocation(shader->getShaderId(), "perspective");
		perspective.useMatrix(perspectiveLocation);

		auto viewLocation = glGetUniformLocation(shader->getShaderId(), "view");
		view.useMatrix(viewLocation);

		const auto translation = Matrix4F::createTranslation(center);
		const auto scale = Matrix4F::createScale(size);

		auto modelMat = translation * scale;

		const auto modelLocation = glGetUniformLocation(shader->getShaderId(), "model");
		modelMat.useMatrix(modelLocation);
		model->render();
	}

	return render;
}

void Octree::UpdateTree()
{
	checked = false;
	for (int i = 0; i < rigidBodies.size(); i++)
	{
		Vector3F bodyPos = rigidBodies[i]->getPos();

		if (bodyPos.getX() < center.getX() - size.getX() ||
			bodyPos.getX() > center.getX() + size.getX() ||
			bodyPos.getY() < center.getY() - size.getY() ||
			bodyPos.getY() > center.getY() + size.getY() ||
			bodyPos.getZ() < center.getZ() - size.getZ() ||
			bodyPos.getZ() > center.getZ() + size.getZ())
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
	Vector3F bodyPos = rigidBody->getPos();

	if (bodyPos.getX() < center.getX() - size.getX() ||
		bodyPos.getX() > center.getX() + size.getX() ||
		bodyPos.getY() < center.getY() - size.getY() ||
		bodyPos.getY() > center.getY() + size.getY() ||
		bodyPos.getZ() < center.getZ() - size.getZ() ||
		bodyPos.getZ() > center.getZ() + size.getZ())
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
		if (size.getX() > 2 &&
			size.getY() > 2 &&
			size.getZ() > 2)
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
				Vector3F childSize = size / 2;
				Vector3F centers[8] =
				{
					center + Vector3F(childSize.getX() * 1.0f, childSize.getY() * 1.0f, childSize.getZ() * 1.0f),
					center + Vector3F(childSize.getX() * 1.0f, childSize.getY() * 1.0f, childSize.getZ() * -1.0f),
					center + Vector3F(childSize.getX() * 1.0f, childSize.getY() * -1.0f, childSize.getZ() * 1.0f),
					center + Vector3F(childSize.getX() * 1.0f, childSize.getY() * -1.0f, childSize.getZ() * -1.0f),
					center + Vector3F(childSize.getX() * -1.0f, childSize.getY() * 1.0f, childSize.getZ() * 1.0f),
					center + Vector3F(childSize.getX() * -1.0f, childSize.getY() * 1.0f, childSize.getZ() * -1.0f),
					center + Vector3F(childSize.getX() * -1.0f, childSize.getY() * -1.0f, childSize.getZ() * 1.0f),
					center + Vector3F(childSize.getX() * -1.0f, childSize.getY() * -1.0f, childSize.getZ() * -1.0f)
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

void Octree::FindNeighbour(Vector3F pCenter, Vector3F pSize, std::vector<Octree*>& pNeighbours, Octree* pNeighbour)
{
	if (this == pNeighbour)
	{
		return;
	}

	if (size.getX() > pSize.getX())
	{
		for (int i = 0; i < 8 && children[i]; i++)
		{
			children[i]->FindNeighbour(pCenter, pSize, pNeighbours, pNeighbour);
		}
	}
	else if (size.getX() == pSize.getX())
	{
		if (center == pCenter + Vector3F(pSize.getX() * 2, pSize.getY() * 2, pSize.getZ() * 2) ||
			center == pCenter + Vector3F(pSize.getX() * 2, pSize.getY() * 2, pSize.getZ() * 0) ||
			center == pCenter + Vector3F(pSize.getX() * 2, pSize.getY() * 2, pSize.getZ() * -2) ||
			center == pCenter + Vector3F(pSize.getX() * 2, pSize.getY() * 0, pSize.getZ() * 2) ||
			center == pCenter + Vector3F(pSize.getX() * 2, pSize.getY() * 0, pSize.getZ() * 0) ||
			center == pCenter + Vector3F(pSize.getX() * 2, pSize.getY() * 0, pSize.getZ() * -2) ||
			center == pCenter + Vector3F(pSize.getX() * 2, pSize.getY() * -2, pSize.getZ() * 2) ||
			center == pCenter + Vector3F(pSize.getX() * 2, pSize.getY() * -2, pSize.getZ() * 0) ||
			center == pCenter + Vector3F(pSize.getX() * 2, pSize.getY() * -2, pSize.getZ() * -2) ||
			center == pCenter + Vector3F(pSize.getX() * 0, pSize.getY() * 2, pSize.getZ() * 2) ||
			center == pCenter + Vector3F(pSize.getX() * 0, pSize.getY() * 2, pSize.getZ() * 0) ||
			center == pCenter + Vector3F(pSize.getX() * 0, pSize.getY() * 2, pSize.getZ() * -2) ||
			center == pCenter + Vector3F(pSize.getX() * 0, pSize.getY() * 0, pSize.getZ() * 2) ||
			center == pCenter + Vector3F(pSize.getX() * 0, pSize.getY() * 0, pSize.getZ() * -2) ||
			center == pCenter + Vector3F(pSize.getX() * 0, pSize.getY() * -2, pSize.getZ() * 2) ||
			center == pCenter + Vector3F(pSize.getX() * 0, pSize.getY() * -2, pSize.getZ() * 0) ||
			center == pCenter + Vector3F(pSize.getX() * 0, pSize.getY() * -2, pSize.getZ() * -2) ||
			center == pCenter + Vector3F(pSize.getX() * -2, pSize.getY() * 2, pSize.getZ() * 2) ||
			center == pCenter + Vector3F(pSize.getX() * -2, pSize.getY() * 2, pSize.getZ() * 0) ||
			center == pCenter + Vector3F(pSize.getX() * -2, pSize.getY() * 2, pSize.getZ() * -2) ||
			center == pCenter + Vector3F(pSize.getX() * -2, pSize.getY() * 0, pSize.getZ() * 2) ||
			center == pCenter + Vector3F(pSize.getX() * -2, pSize.getY() * 0, pSize.getZ() * 0) ||
			center == pCenter + Vector3F(pSize.getX() * -2, pSize.getY() * 0, pSize.getZ() * -2) ||
			center == pCenter + Vector3F(pSize.getX() * -2, pSize.getY() * -2, pSize.getZ() * 2) ||
			center == pCenter + Vector3F(pSize.getX() * -2, pSize.getY() * -2, pSize.getZ() * 0) ||
			center == pCenter + Vector3F(pSize.getX() * -2, pSize.getY() * -2, pSize.getZ() * -2))
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

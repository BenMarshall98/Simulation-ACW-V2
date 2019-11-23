#include "Octree.h"
#include "Game.h"
#include <algorithm>
#include <iostream>
#include "GLFWWindow.h"

//TODO: Change the size to reflect the actual size of the balls

OctreeModel * Octree::mModel = nullptr;// new OctreeModel();
Shader * Octree::mShader = nullptr;// new Shader("octreeVertex.vert", "octreeFragment.frag");

Octree::Octree(const glm::vec3 pCenter, const glm::vec3 pSize, Octree * pParent) : parent(pParent), size(pSize), center(pCenter)
{
	if (!mModel)
	{
		mModel = new OctreeModel();
		mShader = new Shader("octreeVertex.vert", "octreeFragment.frag");
	}
}

Octree::~Octree()
{
	for (auto rigidBody : rigidBodies)
	{
		delete rigidBody;
	}

	for (auto child : children)
	{
		delete child;
	}
	
	for (auto& neighbour : neighbours)
	{
		neighbour->deleteNeighbour(this);
	}
}

bool Octree::addRigidBody(RigidBody * pRigidBody)
{
	const auto bodyPos = pRigidBody->getPos();

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
			for (auto& i : children)
			{
				if (i->addRigidBody(pRigidBody))
				{
					break;
				}
			}
		}
		else
		{
			const auto childSize = size / 2.0f;
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


			auto found = false;
			for (auto i = 0; i < 8; i++)
			{
				children[i] = new Octree(centers[i], childSize, this);
				if (!found && children[i]->addRigidBody(pRigidBody))
				{
					found = true;
				}
			}

			for (auto& i : children)
			{
				i->getNeighbours();
			}
		}
	}
	else
	{
		rigidBodies.push_back(pRigidBody);
	}
	return true;
}

bool Octree::render()
{
	if (Game::getOctreeDisable())
	{
		return false;
	}
	auto render = false;

	if (!rigidBodies.empty())
	{
		render = true;
	}

	if (children[0])
	{
		for (auto& i : children)
		{
			render |= i->render();
		}
	}

	if (render)
	{
		mShader->useShader();

		auto perspective = glm::perspective(45.0f, static_cast<float>(GLFWWindow::instance()->getWidth()) / static_cast<float>(GLFWWindow::instance()->getHeight()), 0.1f, 1000.0f);
		auto view = Game::mCamera->getViewMatrix();

		const auto perspectiveLocation = glGetUniformLocation(mShader->getShaderId(), "perspective");
		glUniformMatrix4fv(perspectiveLocation, 1, GL_FALSE, &perspective[0][0]);

		const auto viewLocation = glGetUniformLocation(mShader->getShaderId(), "view");
		glUniformMatrix4fv(viewLocation, 1, GL_FALSE, &view[0][0]);

		const auto translation = translate(glm::mat4(1.0f), center);
		const auto scale = glm::scale(glm::mat4(1.0f), size);

		auto modelMat = translation * scale;

		const auto modelLocation = glGetUniformLocation(mShader->getShaderId(), "model");
		glUniformMatrix4fv(modelLocation, 1, GL_FALSE, &modelMat[0][0]);
		mModel->render();
	}

	return render;
}

void Octree::updateTree()
{
	checked = false;
	for (auto i = 0u; i < rigidBodies.size(); i++)
	{
		const auto bodyPos = rigidBodies[i]->getPos();

		if (isnan(bodyPos.x))
		{
			delete rigidBodies[i];
			rigidBodies.erase(rigidBodies.begin() + i);
			i--;
			continue;;
		}

		if (bodyPos.x < center.x - size.x ||
			bodyPos.x > center.x + size.x ||
			bodyPos.y < center.y - size.y ||
			bodyPos.y > center.y + size.y ||
			bodyPos.z < center.z - size.z ||
			bodyPos.z > center.z + size.z)
		{
			if (parent)
			{
				parent->moveBody(rigidBodies[i]);
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
		for (auto& i : children)
		{
			i->updateTree();
		}

		auto deleteChildren = true;

		for (auto& i : children)
		{
			if (i->HasChildren() || i->NumOfRigidBodies() > 0)
			{
				deleteChildren = false;
				break;
			}
		}

		if (deleteChildren)
		{
			for (auto& i : children)
			{
				delete i;
				i = nullptr;
			}
		}
	}
}

void Octree::moveBody(RigidBody * pRigidBody)
{
	const auto bodyPos = pRigidBody->getPos();

	if (bodyPos.x < center.x - size.x ||
		bodyPos.x > center.x + size.x ||
		bodyPos.y < center.y - size.y ||
		bodyPos.y > center.y + size.y ||
		bodyPos.z < center.z - size.z ||
		bodyPos.z > center.z + size.z)
	{
		if (parent)
		{
			parent->moveBody(pRigidBody);
		}
		else
		{
			delete pRigidBody;
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
				for (auto& i : children)
				{
					if (i->addRigidBody(pRigidBody))
					{
						break;
					}
				}
			}
			else
			{
				const auto childSize = size / 2.0f;
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


				auto found = false;
				for (auto i = 0; i < 8; i++)
				{
					children[i] = new Octree(centers[i], childSize, this);
					if (!found && children[i]->addRigidBody(pRigidBody))
					{
						found = true;
					}
				}

				for (auto& i : children)
				{
					i->getNeighbours();
				}
			}
		}
		else
		{
			rigidBodies.push_back(pRigidBody);
		}
	}
}

void Octree::getRigidBodies(std::vector<RigidBody*> & pRigidBodies)
{
	pRigidBodies.insert(pRigidBodies.end(), rigidBodies.begin(), rigidBodies.end());

	if (children[0])
	{
		for (auto& i : children)
		{
			i->getRigidBodies(pRigidBodies);
		}
	}
}

void Octree::getNeighbours()
{
	auto grandParent = parent;

	while (grandParent->parent)
	{
		grandParent = grandParent->parent;
	}

	grandParent->findNeighbour(center, size, neighbours, this);
}

void Octree::findNeighbour(const glm::vec3 pCenter, const glm::vec3 pSize, std::vector<Octree*>& pNeighbours, Octree* pNeighbour)
{
	if (this == pNeighbour)
	{
		return;
	}

	if (size.x > pSize.x)
	{
		for (auto i = 0; i < 8 && children[i]; i++)
		{
			children[i]->findNeighbour(pCenter, pSize, pNeighbours, pNeighbour);
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

void Octree::deleteNeighbour(Octree* pNeighbour)
{
	const auto i = std::find(neighbours.begin(), neighbours.end(), pNeighbour);

	if (i != neighbours.end())
	{
		neighbours.erase(i);
	}
}


void Octree::getPossibleCollisions(std::vector<PossibleCollision>& pPossibleCollisions)
{
	if (rigidBodies.empty())
	{
		if (children[0])
		{
			for (auto& i : children)
			{
				i->getPossibleCollisions(pPossibleCollisions);
			}
		}
	}
	else
	{
		for (auto i = 0u; i < rigidBodies.size(); i++)
		{
			for (auto j = i + 1u; j < rigidBodies.size(); j++)
			{
				pPossibleCollisions.emplace_back(PossibleCollision{ rigidBodies[i], rigidBodies[j] });
			}
		}

		checked = true;

		for (auto& rigidBody : rigidBodies)
		{
			for (auto& neighbour : neighbours)
			{
				if (!neighbour->checked)
				{
					for (auto& neighbourRigidBody : neighbour->rigidBodies)
					{
						pPossibleCollisions.emplace_back(PossibleCollision{rigidBody, neighbourRigidBody});
					}
				}
			}
		}
	}
}

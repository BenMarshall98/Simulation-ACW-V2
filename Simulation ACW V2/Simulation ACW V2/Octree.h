#pragma once

#include <vector>

#include "RigidBody.h"
#include "OctreeModel.h"

struct PossibleCollision
{
	RigidBody * rigidBody1;
	RigidBody * rigidBody2;
};

class Octree
{
	static OctreeModel * model;
	static Shader * shader;
	Octree * children[8];
	std::vector<RigidBody *> rigidBodies;
	std::vector<Octree *> neighbours;
	Octree * parent;

	glm::vec3 size;
	glm::vec3 center;
	bool checked = false;

public:
	Octree(glm::vec3 pCenter, glm::vec3 pSize, Octree * pParent = nullptr);
	~Octree();

	bool AddRigidBody(RigidBody * rigidBody);
	bool Render();
	void UpdateTree();
	void MoveBody(RigidBody * rigidBody);
	void GetRigidBodies(std::vector<RigidBody *> & pRigidBodies);

	int NumOfRigidBodies()
	{
		return rigidBodies.size();
	}

	bool HasChildren()
	{
		return children[0];
	}

	void GetNeighbours();
	void DeleteNeighbour(Octree * neighbour);
	void FindNeighbour(glm::vec3 center, glm::vec3 size, std::vector<Octree *> & neighbours, Octree * neighbour);
	void GetPossibleCollisions(std::vector<PossibleCollision> & pPossibleCollisions);
};


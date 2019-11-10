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
	static OctreeModel * mModel;
	static Shader * mShader;
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

	bool addRigidBody(RigidBody * pRigidBody);
	bool render();
	void updateTree();
	void moveBody(RigidBody * pRigidBody);
	void getRigidBodies(std::vector<RigidBody *> & pRigidBodies);

	int NumOfRigidBodies()
	{
		return rigidBodies.size();
	}

	bool HasChildren()
	{
		return children[0];
	}

	void getNeighbours();
	void deleteNeighbour(Octree * pNeighbour);
	void findNeighbour(glm::vec3 pCenter, glm::vec3 pSize, std::vector<Octree *> & pNeighbours, Octree * pNeighbour);
	void getPossibleCollisions(std::vector<PossibleCollision> & pPossibleCollisions);
};


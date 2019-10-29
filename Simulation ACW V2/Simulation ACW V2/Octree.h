#pragma once

#include <vector>

#include "Vector3f.h"
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

	Vector3F size;
	Vector3F center;
	bool checked = false;

public:
	Octree(Vector3F pCenter, Vector3F pSize, Octree * pParent = nullptr);
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
	void FindNeighbour(Vector3F center, Vector3F size, std::vector<Octree *> & neighbours, Octree * neighbour);
	void GetPossibleCollisions(std::vector<PossibleCollision> & pPossibleCollisions);
};


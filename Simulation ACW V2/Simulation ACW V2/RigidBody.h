#pragma once

#include "Shader.h"
#include "Model.h"
#include "gl.h"
#include "glm/glm.hpp"
#include "glm/gtx/quaternion.hpp"

class SceneGraphNode;

struct State
{
	glm::vec3 pos;
	glm::vec3 vel;
	glm::vec3 angVel;
	glm::quat orientation;
};

struct Derivative
{
	glm::vec3 dVel;
	glm::vec3 dAcc;
	glm::vec3 dAngVel;
	glm::vec3 dAngAcc;
};

enum class ObjectType
{
	PLANE,
	SPHERE,
	PLANEHOLES,
	CYLINDER,
	BOWL,
	CUBOID
};

class RigidBody
{
public:
	RigidBody(glm::vec3 pSize, float pMass, glm::vec3 pPos, glm::vec3 pAngularVelocity, glm::vec3 pVelocity, ObjectType pType,  glm::mat3 pImpulseTensor);
	virtual ~RigidBody()
	{
		int i = 0;
	}

	RigidBody(const RigidBody &) = delete;
	RigidBody(RigidBody &&) = delete;
	RigidBody & operator= (const RigidBody &) = delete;
	RigidBody & operator= (RigidBody &&) = delete;

	Derivative evaluate(const State & initial, float time, float dt, const Derivative & derivative);
	glm::vec3 acceleration(const State& state, float time);
	glm::vec3 angularAcceleeration(const State& state, float time);
	void integrate(State & state, float time, float dt);
	void calculatePhysics(float pDt, float pCurrentUpdateTime);
	void update();
	void updateRender();
	void setPos(glm::vec3 pPos);
	void setVel(glm::vec3 pVel);
	void setAngularVel(glm::vec3 pAngularVel);
	void setOrientation(glm::quat pOrientation);
	void setNewPos(glm::vec3 pPos);
	void setNewVel(glm::vec3 pVel);
	void setNewAngularVel(glm::vec3 pAngularVel);
	void setNewOrientation(glm::quat pOrientation);
	void setMass(float pMass);
	void setSceneGraphNode(SceneGraphNode * pParent);

	glm::vec3 getPos() const;
	glm::vec3 getRenderPos() const;
	glm::vec3 getNewPos() const;
	glm::vec3 getVel() const;
	glm::vec3 getNewVel() const;
	glm::quat getOrientation() const;
	glm::quat getRenderOrientation() const;
	glm::quat getNewOrientation() const;
	glm::vec3 getAngularVelocity() const;
	glm::vec3 getNewAngularVelocity() const;
	glm::vec3 getSize() const;
	glm::mat4 getMatrix() const;
	glm::mat4 getNewMatrix() const;
	glm::mat3 getInverseImpulseTenser() const;
	glm::mat3 getImpulseTenser() const;
	float getCurrentUpdateTime() const;

	ObjectType getObjectType() const;

	float getMass() const;

	int getID() const
	{
		return mObjectId;
	}
	
	void resetPos();

	virtual void render(Shader * pShader) const = 0;

protected:
	glm::quat mRotation;
	glm::quat mRenderRotation;
	glm::quat mNewRotation;
	glm::mat3 mImpulseTensor;
	glm::mat3 mInverseImpulseTensor;

	glm::vec3 mPos;
	glm::vec3 mRenderPos;
	glm::vec3 mNewPos;
	glm::vec3 mVelocity;
	glm::vec3 mNewVelocity;
	glm::vec3 mSize;
	glm::vec3 mAngularVelocity;
	glm::vec3 mNewAngularVelocity;

	SceneGraphNode * mParent = nullptr;
	float mMass;
	float mLastUpdateTime;
	int mObjectId;
	Shader * mShader;
	Model * mModel;
	
private:
	ObjectType mType;

protected:
	static int mCountId;
};


#pragma once

#include "Vector3f.h"
#include "Shader.h"
#include "Model.h"
#include "gl.h"
#include "Matrix4f.h"

class SceneGraphNode;

struct State
{
	Vector3F pos;
	Vector3F vel;
};

struct Derivative
{
	Vector3F dVel;
	Vector3F dAcc;
};

enum class ObjectType
{
	PLANE,
	SPHERE,
	PLANEHOLES,
	CYLINDER,
	BOWL
};

class RigidBody
{
public:
	RigidBody(Vector3F pSize, float pMass, Vector3F pPos, Vector3F pRotationAxis, float pRotationAngle, Vector3F pVelocity, ObjectType pType);
	virtual ~RigidBody() = default;

	RigidBody(const RigidBody &) = delete;
	RigidBody(RigidBody &&) = delete;
	RigidBody & operator= (const RigidBody &) = delete;
	RigidBody & operator= (RigidBody &&) = delete;

	static Derivative evaluate(const State & initial, float time, float dt, const Derivative & derivative);
	static Vector3F acceleration(const State& state, float time);
	static void integrate(State & state, float time, float dt);
	void calculatePhysics(float pDt);
	void update();
	void updateRender();
	void setPos(Vector3F pPos);
	void setVel(Vector3F pVel);
	void setNewPos(Vector3F pPos);
	void setNewVel(Vector3F pVel);
	void setMass(float pMass);
	void setSceneGraphNode(SceneGraphNode * pParent);

	Vector3F getPos() const;
	Vector3F getRenderPos() const;
	Vector3F getNewPos() const;
	Vector3F getVel() const;
	Vector3F getNewVel() const;
	Vector3F getSize() const;
	Matrix4F getMatrix() const;

	ObjectType getObjectType() const;

	float getMass() const;
	void resetPos();

	virtual void render(Shader * pShader) const = 0;

private:
	ObjectType mType;

protected:
	SceneGraphNode * mParent = nullptr;
	float mMass;
	Vector3F mPos;
	Vector3F mRenderPos;
	Vector3F mNewPos;
	Vector3F mVelocity;
	Vector3F mNewVelocity;
	Vector3F mSize;
	Vector3F mRotationAxis;
	float mRotationAngle;
	int mObjectId;
	GLuint mTexture;
	Shader * mShader;
	Model * mModel;

	static int mCountId;
};


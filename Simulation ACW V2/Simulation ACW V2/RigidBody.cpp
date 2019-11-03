#include "RigidBody.h"
#include "corecrt_math_defines.h"
#include "SceneGraphNode.h"

int RigidBody::mCountId = 0;

RigidBody::RigidBody(const Vector3F pSize, const float pMass, const Vector3F pPos, const Vector3F pAngularVelocity, const Vector3F pVelocity, const ObjectType pType) :
	mSize(pSize), mMass(pMass), mPos(pPos), mAngularVelocity(pAngularVelocity), mVelocity(pVelocity), mType(pType), mObjectId(mCountId), mNewPos(pPos), mNewVelocity(pVelocity),
	mNewAngularVelocity(pAngularVelocity), mRotation(Matrix3F()), mNewRotation(Matrix3F())
{
	mCountId++;
}

void RigidBody::setPos(const Vector3F pPos)
{
	mPos = pPos;
}

void RigidBody::setNewPos(const Vector3F pPos)
{
	mNewPos = pPos;
}

void RigidBody::setVel(const Vector3F pVel)
{
	mVelocity = pVel;
}

void RigidBody::setNewVel(const Vector3F pVel)
{
	mNewVelocity = pVel;
}

void RigidBody::setAngularVel(Vector3F pAngularVel)
{
	mAngularVelocity = pAngularVel;
}

void RigidBody::setNewAngularVel(Vector3F pAngularVel)
{
	mNewAngularVelocity = pAngularVel;
}

void RigidBody::setMass(const float pMass)
{
	mMass = pMass;
}

void RigidBody::setSceneGraphNode(SceneGraphNode* pParent)
{
	mParent = pParent;
}


Vector3F RigidBody::acceleration(const State& state, float time)
{
	return Vector3F(0.0f, -9.81f, 0.0f);
}

Derivative RigidBody::evaluate(const State& initial, float time, float dt, const Derivative& derivative)
{
	State state;
	state.pos = initial.pos + derivative.dVel * dt;
	state.vel = initial.vel + derivative.dAcc * dt;
	state.orientation = (initial.orientation + Matrix3F::createSkew(derivative.dAngVel) * initial.orientation * dt).normaliseColumns();
	state.angVel = initial.angVel + derivative.dAngAcc * dt;

	Derivative output;
	output.dVel = state.vel;
	output.dAcc = acceleration(state, time + dt);
	output.dAngVel = state.angVel;
	output.dAngAcc = Vector3F(0, 0, 0);
	return output;
}

void RigidBody::integrate(State& state, float time, float dt)
{
	Derivative k1, k2, k3, k4;

	k1 = evaluate(state, time, 0.0f, Derivative());
	k2 = evaluate(state, time, dt * 0.5f, k1);
	k3 = evaluate(state, time, dt * 0.5f, k2);
	k4 = evaluate(state, time, dt, k3);

	Vector3F dPos = 1.0f / 6.0f * (k1.dVel + 2.0f * (k2.dVel + k3.dVel) + k4.dVel);
	Vector3F dVel = 1.0f / 6.0f * (k1.dAcc + 2.0f * (k2.dAcc + k3.dAcc) + k4.dAcc);
	Vector3F dRot = 1.0f / 6.0f * (k1.dAngVel + 2.0f * (k2.dAngVel + k3.dAngVel) + k4.dAngVel);
	Vector3F dAngVel = 1.0f / 6.0f * (k1.dAngAcc + 2.0f * (k2.dAngAcc + k3.dAngAcc) + k4.dAngAcc);

	state.pos = state.pos + dPos * dt;
	state.vel = state.vel + dVel * dt;
	state.orientation = (state.orientation + Matrix3F::createSkew(dRot) * state.orientation * dt).normaliseColumns();
	state.angVel = state.angVel + dAngVel * dt;
}


void RigidBody::calculatePhysics(const float pDt, const float pLastUpdateTime)
{
	if (mMass == -1.0f)
	{
		//Do nothing
		return;
	}

	mLastUpdateTime = pLastUpdateTime;

	State state = { mPos, mVelocity, mAngularVelocity, mRotation};

	integrate(state, 0.0f, pDt);

	mNewPos = state.pos;
	mNewVelocity = state.vel;
	mNewAngularVelocity = state.angVel;
	mNewRotation = state.orientation;
}

void RigidBody::resetPos()
{
	mNewPos = mPos;
}

void RigidBody::update()
{
	mVelocity = mNewVelocity;
	mPos = mNewPos;
	mRotation = mNewRotation;
	mAngularVelocity = mNewAngularVelocity;
}

Vector3F RigidBody::getSize() const
{
	return mSize;
}


float RigidBody::getMass() const
{
	return mMass;
}

Vector3F RigidBody::getPos() const
{
	return mPos;
}

Vector3F RigidBody::getRenderPos() const
{
	return mRenderPos;
}

void RigidBody::updateRender()
{
	mRenderPos = mPos;
	mRenderRotation = mRotation;
	mLastUpdateTime = 0.0f;
}


Vector3F RigidBody::getNewPos() const
{
	return mNewPos;
}

Matrix3F RigidBody::getOrientation() const
{
	return mRotation;
}

Matrix3F RigidBody::getRenderOrientation() const
{
	return mRenderRotation;
}

Matrix3F RigidBody::getNewOrientation() const
{
	return mNewRotation;
}

Vector3F RigidBody::getAngularVelocity() const
{
	return mAngularVelocity;
}

Vector3F RigidBody::getNewAngularVelocity() const
{
	return mNewAngularVelocity;
}

Vector3F RigidBody::getVel() const
{
	return mVelocity;
}

Vector3F RigidBody::getNewVel() const
{
	return mNewVelocity;
}

float RigidBody::getCurrentUpdateTime() const
{
	return mLastUpdateTime;
}

Matrix4F RigidBody::getMatrix() const
{
	auto modelMat = Matrix4F();

	if (mParent)
	{
		modelMat = modelMat * mParent->getRenderMatrix();
	}

	const auto translation = Matrix4F::createTranslation(mPos);
	const auto scale = Matrix4F::createScale(mSize);
	const auto rotation = Matrix4F(mRotation);

	modelMat = modelMat * translation * rotation * scale;

	return modelMat;
}

Matrix4F RigidBody::getNewMatrix() const
{
	auto modelMat = Matrix4F();

	if (mParent)
	{
		modelMat = modelMat * mParent->getUpdateMatrix();
	}

	const auto translation = Matrix4F::createTranslation(mNewPos);
	const auto scale = Matrix4F::createScale(mSize);
	const auto rotation = Matrix4F(mRotation);

	modelMat = modelMat * translation * rotation * scale;

	return modelMat;
}

ObjectType RigidBody::getObjectType() const
{
	return mType;
}

#include "RigidBody.h"
#include "corecrt_math_defines.h"
#include "SceneGraphNode.h"

int RigidBody::mCountId = 0;

RigidBody::RigidBody(const glm::vec3 pSize, const float pMass, const glm::vec3 pPos, const glm::vec3 pAngularVelocity, const glm::vec3 pVelocity, const ObjectType pType, const glm::mat3 pImpulseTensor) :
	mSize(pSize), mMass(pMass), mPos(pPos), mAngularVelocity(pAngularVelocity), mVelocity(pVelocity), mType(pType), mObjectId(mCountId), mNewPos(pPos), mNewVelocity(pVelocity),
	mNewAngularVelocity(pAngularVelocity), mRotation(glm::mat3(1.0f)), mNewRotation(glm::mat3(1.0f)), mImpulseTensor(pImpulseTensor), mInverseImpulseTensor(inverse(pImpulseTensor))
{
	mCountId++;
}

void RigidBody::setPos(const glm::vec3 pPos)
{
	mPos = pPos;
}

void RigidBody::setNewPos(const glm::vec3 pPos)
{
	mNewPos = pPos;
}

void RigidBody::setVel(const glm::vec3 pVel)
{
	mVelocity = pVel;
}

void RigidBody::setNewVel(const glm::vec3 pVel)
{
	mNewVelocity = pVel;
}

void RigidBody::setAngularVel(glm::vec3 pAngularVel)
{
	mAngularVelocity = pAngularVel;
}

void RigidBody::setNewAngularVel(glm::vec3 pAngularVel)
{
	mNewAngularVelocity = pAngularVel;
}

void RigidBody::setOrientation(glm::quat pOrientation)
{
	mRotation = pOrientation;
}

void RigidBody::setNewOrientation(glm::quat pOrientation)
{
	mNewRotation = pOrientation;
}

void RigidBody::setMass(const float pMass)
{
	mMass = pMass;
}

void RigidBody::setSceneGraphNode(SceneGraphNode* pParent)
{
	mParent = pParent;
}


glm::vec3 RigidBody::acceleration(const State& state, float time)
{
	const auto mag = length(mVelocity);
	const auto drag = (-mVelocity * (0.0005f * mag + 0.0005f * mag * mag)) / mMass;
	const auto gravity = glm::vec3(0.0f, -9.81f, 0.0f);
	return gravity + drag;
}

glm::vec3 RigidBody::angularAcceleeration(const State& state, float time)
{
	const auto mag = length(mAngularVelocity);
	const auto drag = (-mAngularVelocity * (0.0005f * mag + 0.0005f * mag * mag)) / mMass;
	return drag;
}

Derivative RigidBody::evaluate(const State& initial, float time, float dt, const Derivative& derivative)
{
	State state;
	state.pos = initial.pos + derivative.dVel * dt;
	state.vel = initial.vel + derivative.dAcc * dt;

	glm::vec3 changeAngle = derivative.dAngVel * dt;
	glm::quat changeQuat = glm::quat(0.0f, changeAngle);
	state.orientation = glm::normalize(initial.orientation + 0.5f * changeQuat * initial.orientation);
	state.angVel = initial.angVel + derivative.dAngAcc * dt;

	Derivative output;
	output.dVel = state.vel;
	output.dAcc = acceleration(state, time + dt);
	output.dAngVel = state.angVel;
	output.dAngAcc = angularAcceleeration(state, time + dt);
	return output;
}

void RigidBody::integrate(State& state, float time, float dt)
{
	Derivative k1, k2, k3, k4;

	k1 = evaluate(state, time, 0.0f, Derivative());
	k2 = evaluate(state, time, dt * 0.5f, k1);
	k3 = evaluate(state, time, dt * 0.5f, k2);
	k4 = evaluate(state, time, dt, k3);

	auto dPos = 1.0f / 6.0f * (k1.dVel + 2.0f * (k2.dVel + k3.dVel) + k4.dVel);
	auto dVel = 1.0f / 6.0f * (k1.dAcc + 2.0f * (k2.dAcc + k3.dAcc) + k4.dAcc);
	auto dRot = 1.0f / 6.0f * (k1.dAngVel + 2.0f * (k2.dAngVel + k3.dAngVel) + k4.dAngVel);
	auto dAngVel = 1.0f / 6.0f * (k1.dAngAcc + 2.0f * (k2.dAngAcc + k3.dAngAcc) + k4.dAngAcc);

	state.pos = state.pos + dPos * dt;
	state.vel = state.vel + dVel * dt;

	auto changeAngle = dRot * dt;
	auto changeQuat = glm::quat(0.0f, changeAngle);
	state.orientation = glm::normalize(state.orientation + 0.5f * changeQuat * state.orientation);
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

glm::vec3 RigidBody::getSize() const
{
	return mSize;
}


float RigidBody::getMass() const
{
	return mMass;
}

glm::vec3 RigidBody::getPos() const
{
	return mPos;
}

glm::vec3 RigidBody::getRenderPos() const
{
	return mRenderPos;
}

void RigidBody::updateRender()
{
	mRenderPos = mPos;
	mRenderRotation = mRotation;
	mLastUpdateTime = 0.0f;
}


glm::vec3 RigidBody::getNewPos() const
{
	return mNewPos;
}

glm::quat RigidBody::getOrientation() const
{
	return mRotation;
}

glm::quat RigidBody::getRenderOrientation() const
{
	return mRenderRotation;
}

glm::quat RigidBody::getNewOrientation() const
{
	return mNewRotation;
}

glm::vec3 RigidBody::getAngularVelocity() const
{
	return mAngularVelocity;
}

glm::vec3 RigidBody::getNewAngularVelocity() const
{
	return mNewAngularVelocity;
}

glm::vec3 RigidBody::getVel() const
{
	return mVelocity;
}

glm::vec3 RigidBody::getNewVel() const
{
	return mNewVelocity;
}

float RigidBody::getCurrentUpdateTime() const
{
	return mLastUpdateTime;
}

glm::mat4 RigidBody::getMatrix() const
{
	auto modelMat = glm::mat4(1.0f);

	if (mParent)
	{
		modelMat = modelMat * mParent->getRenderMatrix();
	}

	const auto translation = translate(glm::mat4(1.0f), mPos);
	const auto scale = glm::scale(glm::mat4(1.0f), mSize);
	const auto rotation = toMat4(mRotation);

	modelMat = modelMat * translation * rotation * scale;

	return modelMat;
}

glm::mat3 RigidBody::getImpulseTenser() const
{
	return mImpulseTensor;
}

glm::mat3 RigidBody::getInverseImpulseTenser() const
{
	return mInverseImpulseTensor;
}


glm::mat4 RigidBody::getNewMatrix() const
{
	auto modelMat = glm::mat4(1.0f);

	if (mParent)
	{
		modelMat = modelMat * mParent->getUpdateMatrix();
	}

	const auto translation = translate(glm::mat4(1.0f), mNewPos);
	const auto scale = glm::scale(glm::mat4(1.0f), mSize);
	const auto rotation = toMat4(mNewRotation);

	modelMat = modelMat * translation * rotation * scale;

	return modelMat;
}

ObjectType RigidBody::getObjectType() const
{
	return mType;
}

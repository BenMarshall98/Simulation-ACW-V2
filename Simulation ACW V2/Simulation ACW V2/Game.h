#pragma once
#include "Shader.h"
#include <vector>
#include "Camera.h"
#include "RigidBody.h"
#include "IdentityNode.h"
#include "HoldingContainer.h"
#include "ContactManifold.h"
#include <condition_variable>
#include <map>

class Game
{
	std::mutex m;
	std::condition_variable cv;
	bool ready = false;
	bool processed = false;
	bool end = false;

	std::map<RigidBody *, std::vector<RigidBody *>> mPossibleRigidBodyCollisions;
	Shader * mPlaneShader;
	Shader * mSphereShader;
	IdentityNode * sceneGraph;
	HoldingContainer * mHoldingContainer;
	std::vector<RigidBody *> octreeBody;
	std::vector<RigidBody *> sceneBody;
	std::vector<PossibleCollision> mPossibleCollisions;
	Octree * octree;
	ContactManifold * mManifold;
	
	static bool mReset;
	static bool mPause;
	static bool mAddSphere;
	static float mTimeScale;
	static float mFriction;
	static float mSphereElasticity;
	static float mSphereSize;
	static float mDt;
	
	void update();
	void simulationLoop();
	void calculateObjectPhysics() const;
	void dynamicCollisionDetection() const;
	void dynamicCollisionResponse() const;
	void updateObjectPhysics() const;
	void updateObjectRender() const;
	void render() const;
	void reset() const;
	void swap();

public:
	static Camera * camera;
	
	Game();
	~Game();

	Game(const Game &) = delete;
	Game(Game &&) = delete;
	Game & operator= (const Game &) = delete;
	Game & operator= (Game &&) = delete;

	void run();

	static float getRenderDt()
	{
		return mDt;
	}

	static void setPause()
	{
		mPause = !mPause;
	}

	static void setReset()
	{
		mReset = true;
	}

	static void changeTimeScale(bool pDirection)
	{
		if (pDirection)
		{
			mTimeScale += 0.1f;
		}
		else if (mTimeScale > 0.0f)
		{
			mTimeScale -= 0.1f;
		}
		else
		{
			mTimeScale = 0.0f;
		}
	}

	static void changeFriction(bool pDirection)
	{
		if (pDirection)
		{
			mFriction += 0.1f;
		}
		else if (mFriction > 0.0f)
		{
			mFriction -= 0.1f;
		}
		else
		{
			mFriction = 0.0f;
		}
	}

	static void changeSphereElasticty(bool pDirection)
	{
		if (pDirection && mSphereElasticity < 1.0f)
		{
			mSphereElasticity += 0.1f;
		}
		else if (!pDirection && mSphereElasticity > 0.1f)
		{
			mSphereElasticity -= 0.1f;
		}
	}

	static void changeSphereSize(bool pDirection)
	{
		if (pDirection)
		{
			mSphereSize += 0.1f;
		}
		else if (mSphereSize > 0.1f)
		{
			mSphereSize -= 0.1f;
		}
	}

	static float getUpdateDt()
	{
		return mDt * mTimeScale;
	}


	static float getSphereSize()
	{
		return mSphereSize;
	}

	static void addSphere()
	{
		mAddSphere = true;
	}
};

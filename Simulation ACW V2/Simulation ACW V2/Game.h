#pragma once
#include "Shader.h"
#include <vector>
#include "Camera.h"
#include "RigidBody.h"
#include "IdentityNode.h"
#include "HoldingContainer.h"
#include "ContactManifold.h"
#include <condition_variable>

class Game
{
	std::mutex mMutex;
	std::condition_variable mCv;
	bool mReady = false;
	bool mProcessed = false;
	bool mEnd = false;

	Shader * mPlaneShader;
	Shader * mSphereShader;
	IdentityNode * mSceneGraph;
	HoldingContainer * mHoldingContainer;
	std::vector<RigidBody *> mOctreeBody;
	std::vector<RigidBody *> mSceneBody;
	std::vector<PossibleCollision> mPossibleCollisions;
	Octree * mOctree;
	ContactManifold * mManifold;
	
	static bool mReset;
	static bool mPause;
	static bool mAddSphere;
	static bool mAddCube;
	static bool mAngularDisable;
	static bool mOctreeDisable;
	static float mTimeScale;
	static float mFriction;
	static float mSphereElasticity;
	static float mSphereSize;
	static float mCuboidSize;
	static float mDt;
	
	void update();
	void simulationLoop();
	void calculateObjectPhysics() const;
	void dynamicCollisionDetection() const;
	void dynamicCollisionResponse() const;
	void updateObjectPhysics() const;
	void updateObjectRender() const;
	void render() const;
	void reset();
	void swap();
	void clear();
	void build();
	void display();

public:
	static Camera * mCamera;
	
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

	static void setAngularDisable()
	{
		mAngularDisable = !mAngularDisable;
	}

	static void setOctreeDisable()
	{
		mOctreeDisable = !mOctreeDisable;
	}

	static void changeTimeScale(const bool pDirection)
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

	static void changeFriction(const bool pDirection)
	{
		if (pDirection)
		{
			mFriction += 0.1f;
		}
		else if (mFriction > 0.01f)
		{
			mFriction -= 0.1f;
		}
		else
		{
			mFriction = 0.0f;
		}
	}

	static float getSphereElasticity()
	{
		return mSphereElasticity;
	}

	static float getSphereFriction()
	{
		return mFriction;
	}

	static void changeSphereElasticity(const bool pDirection)
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

	static void changeSphereSize(const bool pDirection)
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

	static void changeCuboidSize(const bool pDirection)
	{
		if (pDirection)
		{
			mCuboidSize += 0.1f;
		}
		else if (mCuboidSize > 0.1f)
		{
			mCuboidSize -= 0.1f;
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

	static float getCuboidSize()
	{
		return mCuboidSize;
	}

	static bool getAngularDisable()
	{
		return mAngularDisable;
	}

	static bool getOctreeDisable()
	{
		return mOctreeDisable;
	}

	static void addSphere()
	{
		mAddSphere = true;
	}

	static void addCube()
	{
		mAddCube = true;
	}
};

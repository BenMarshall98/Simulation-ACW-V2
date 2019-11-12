#include "Game.h"
#include "gl.h"
#include <GLFW/glfw3.h>
#include "GLFWWindow.h"
#include <string>
#include "Shader.h"
#include "TranslationNode.h"
#include "RigidBodyNode.h"
#include "RotationAnimation.h"
#include "RotationNode.h"
#include "Plane.h"
#include "Bowl.h"
#include "Cylinder.h"
#include "PlaneHoles.h"
#include "CollisionDetection.h"
#include "CollisionResponse.h"
#include <thread>

float Game::mDt = 0.0f;
float Game::mTimeScale = 1.0f;
float Game::mFriction = 0.1f;
float Game::mSphereElasticity = 0.8f;
float Game::mSphereSize = 0.5f;
bool Game::mReset = false;
bool Game::mPause = false;
bool Game::mAddSphere = false;
bool Game::mAddCube = false;

Camera * Game::mCamera = new Camera(glm::vec3(0, 25, 50), glm::vec3(0, 1, 0), glm::vec3(0, 0, 0));

Game::Game()
{
	mSphereShader = new Shader("sphereVertex.vert", "sphereFragment.frag");
	mPlaneShader = new Shader("planeVertex.vert", "planeFragment.frag");

	mOctree = new Octree(glm::vec3(0, 0, 0), glm::vec3(25, 25, 25));
	mHoldingContainer = new HoldingContainer(mOctree, glm::vec3(0, 18, 0), glm::vec3(2, 2, 2));
	mManifold = new ContactManifold();
	
	mSceneGraph = new IdentityNode();

	auto planeHolesTranslation = new TranslationNode(glm::vec3(0, -2, 0));
	mSceneGraph->addChild(planeHolesTranslation);

	const auto planeHolesAnimation = new TranslationAnimation(
		[](glm::vec3& pVector, const float pDt, const bool pDirection)
	{
		if (!pDirection)
		{
			auto newX = pVector.x + pDt;

			if (newX > 15.0f)
			{
				newX = 15.0f;
			}

			pVector = glm::vec3(newX, pVector.y, pVector.z);
		}
		else
		{
			auto newX = pVector.x - pDt;

			if (newX < 0.0f)
			{
				newX = 0.0f;
			}

			pVector = glm::vec3(newX, pVector.y, pVector.z);
		}
	}
	, '4', '3');

	auto planeHolesTranslationWithAnimation = new TranslationNode(glm::vec3(0, 0, 0), planeHolesAnimation);
	planeHolesTranslation->addChild(planeHolesTranslationWithAnimation);

	const auto planeHoles = new PlaneHoles(glm::vec3(1, 1, 1), -1, glm::vec3(0, 0, 0), glm::vec3(0, 0, 0), glm::vec3(0, 0, 0));
	const auto planeHolesNode = new RigidBodyNode(planeHoles);
	planeHolesTranslationWithAnimation->addChild(planeHolesNode);
	mSceneBody.push_back(planeHoles);

	const auto propellerAnimation = new RotationAnimation(
		[](glm::vec3&, float & pRotationAngle, const float pDt, const float pAngleChange)
	{
		pRotationAngle += pDt * pAngleChange;
	}
	, 'R', 'F');

	auto propellerRotationWithAnimation = new RotationNode(glm::vec3(0, 1, 0), 0, propellerAnimation);

	planeHolesTranslationWithAnimation->addChild(propellerRotationWithAnimation);

	auto propellerCentreTranslation = new TranslationNode(glm::vec3(0, 3, 0));

	propellerRotationWithAnimation->addChild(propellerCentreTranslation);

	auto cylinder = new Cylinder(glm::vec3(1, 1, 1), -1, glm::vec3(0, 0, 0), glm::vec3(0, 0, 0), glm::vec3(0, 0, 0));

	const auto propellerCentreNode = new RigidBodyNode(cylinder);

	propellerCentreTranslation->addChild(propellerCentreNode);
	mSceneBody.push_back(cylinder);

	auto propellerBladeTranslation = new TranslationNode(glm::vec3(0, 6, 0));

	propellerRotationWithAnimation->addChild(propellerBladeTranslation);

	auto propellerBladeRotation = new RotationNode(glm::vec3(1, 0, 0), 90);

	propellerBladeTranslation->addChild(propellerBladeRotation);

	auto propellerBlade1Rotation = new RotationNode(glm::vec3(0, 0, 1), 0);

	propellerBladeRotation->addChild(propellerBlade1Rotation);

	auto propellerBlade1Translation = new TranslationNode(glm::vec3(0, 1.5f, 0));

	propellerBlade1Rotation->addChild(propellerBlade1Translation);

	cylinder = new Cylinder(glm::vec3(1, 1, 1), -1, glm::vec3(0, 0, 0), glm::vec3(0, 0, 0),  glm::vec3(0, 0, 0));

	const auto propellerBlade1Node = new RigidBodyNode(cylinder);

	propellerBlade1Translation->addChild(propellerBlade1Node);
	mSceneBody.push_back(cylinder);

	auto propellerBlade2Rotation = new RotationNode(glm::vec3(0, 0, 1), 120);

	propellerBladeRotation->addChild(propellerBlade2Rotation);

	auto propellerBlade2Translation = new TranslationNode(glm::vec3(0, 1.5f, 0));

	propellerBlade2Rotation->addChild(propellerBlade2Translation);

	cylinder = new Cylinder(glm::vec3(1, 1, 1), -1, glm::vec3(0, 0, 0), glm::vec3(0, 0, 0), glm::vec3(0, 0, 0));

	const auto propellerBlade2Node = new RigidBodyNode(cylinder);

	propellerBlade2Translation->addChild(propellerBlade2Node);
	mSceneBody.push_back(cylinder);

	auto propellerBlade3Rotation = new RotationNode(glm::vec3(0, 0, 1), 240);

	propellerBladeRotation->addChild(propellerBlade3Rotation);

	auto propellerBlade3Translation = new TranslationNode(glm::vec3(0, 1.5f, 0));

	propellerBlade3Rotation->addChild(propellerBlade3Translation);

	cylinder = new Cylinder(glm::vec3(1, 1, 1), -1, glm::vec3(0, 0, 0), glm::vec3(0, 0, 0), glm::vec3(0, 0, 0));

	const auto propellerBlade3Node = new RigidBodyNode(cylinder);

	propellerBlade3Translation->addChild(propellerBlade3Node);
	mSceneBody.push_back(cylinder);

	auto planeTranslation = new TranslationNode(glm::vec3(0, -8, 0));

	mSceneGraph->addChild(planeTranslation);

	const auto planeAnimation = new TranslationAnimation(
		[](glm::vec3&pVector, const float pDt, const bool pDirection)
	{
		if (!pDirection)
		{
			auto newX = pVector.x + pDt;

			if (newX > 15.0f)
			{
				newX = 15.0f;
			}

			pVector = glm::vec3(newX, pVector.y, pVector.z);
		}
		else
		{
			auto newX = pVector.x - pDt;

			if (newX < 0.0f)
			{
				newX = 0.0f;
			}

			pVector = glm::vec3(newX, pVector.y, pVector.z);
		}
	}
	, '6', '5');

	auto planeTranslationWithAnimation = new TranslationNode(glm::vec3(0, 0, 0), planeAnimation);

	planeTranslation->addChild(planeTranslationWithAnimation);

	auto plane = new Plane(-1, glm::vec3(5, 1, 5), glm::vec3(0, 0, 0), glm::vec3(0, 0, 0), glm::vec3(0, 0, 0));

	const auto planeNode = new RigidBodyNode(plane);

	planeTranslationWithAnimation->addChild(planeNode);
	mSceneBody.push_back(plane);

	auto leftPlaneTranslation = new TranslationNode(glm::vec3(-5, 0, 0));

	mSceneGraph->addChild(leftPlaneTranslation);

	auto leftPlaneRotation = new RotationNode(glm::vec3(0, 0, 1), 90);

	leftPlaneTranslation->addChild(leftPlaneRotation);

	plane = new Plane(-1, glm::vec3(10, 1, 5), glm::vec3(0, 0, 0), glm::vec3(0, 0, 0), glm::vec3(0, 0, 0));

	const auto leftPlaneNode = new RigidBodyNode(plane);

	leftPlaneRotation->addChild(leftPlaneNode);
	mSceneBody.push_back(plane);

	auto rightPlaneTranslation = new TranslationNode(glm::vec3(5, 0, 0));

	mSceneGraph->addChild(rightPlaneTranslation);

	auto rightPlaneRotation = new RotationNode(glm::vec3(0, 0, 1), -90);

	rightPlaneTranslation->addChild(rightPlaneRotation);

	plane = new Plane(-1, glm::vec3(10, 1, 5), glm::vec3(0, 0, 0), glm::vec3(0, 0, 0), glm::vec3(0, 0, 0));

	const auto rightPlaneNode = new RigidBodyNode(plane);

	rightPlaneRotation->addChild(rightPlaneNode);
	mSceneBody.push_back(plane);

	auto frontPlaneTranslation = new TranslationNode(glm::vec3(0, 0, 5));

	mSceneGraph->addChild(frontPlaneTranslation);

	auto frontPlaneRotation = new RotationNode(glm::vec3(1, 0, 0), 90);

	frontPlaneTranslation->addChild(frontPlaneRotation);

	plane = new Plane(-1, glm::vec3(5, 1, 10), glm::vec3(0, 0, 0), glm::vec3(0, 0, 0), glm::vec3(0, 0, 0));

	const auto frontPlaneNode = new RigidBodyNode(plane);

	frontPlaneRotation->addChild(frontPlaneNode);

	mSceneBody.push_back(plane);

	auto backPlaneTranslation = new TranslationNode(glm::vec3(0, 0, -5));

	mSceneGraph->addChild(backPlaneTranslation);

	auto backPlaneRotation = new RotationNode(glm::vec3(1, 0, 0), -90);

	backPlaneTranslation->addChild(backPlaneRotation);

	plane = new Plane(-1, glm::vec3(5, 1, 10), glm::vec3(0, 0, 0), glm::vec3(0, 0, 0), glm::vec3(0, 0, 0));

	const auto backPlaneNode = new RigidBodyNode(plane);

	backPlaneRotation->addChild(backPlaneNode);

	mSceneBody.push_back(plane);

	auto bowlTranslation = new TranslationNode(glm::vec3(0, 10, 0));

	mSceneGraph->addChild(bowlTranslation);

	auto bowlRotation = new RotationNode(glm::vec3(1, 0, 0), 90);

	bowlTranslation->addChild(bowlRotation);

	const auto bowl = new Bowl(glm::vec3(25, 25, 25), -1, glm::vec3(0, 0, 0), glm::vec3(0, 0, 0), glm::vec3(0, 0, 0));

	const auto bowlNode = new RigidBodyNode(bowl);

	bowlRotation->addChild(bowlNode);

	mSceneBody.push_back(bowl);
}

Game::~Game()
{
}

void Game::run()
{
	swap();
	std::thread updateThread(&Game::update, this);

	auto lastTime = static_cast<float>(glfwGetTime());
	while (GLFWWindow::instance()->windowEvents())
	{
		if (mAddSphere)
		{
			mHoldingContainer->addSphere();
			mAddSphere = false;
		}

		if (mAddCube)
		{
			mHoldingContainer->addCube();
			mAddCube = false;
		}

		swap();
		
		if (!mPause)
		{
			{
				std::lock_guard<std::mutex> lock(mMutex);
				mReady = true;
				mProcessed = false;
			}

			mCv.notify_one();

			render();

			{
				std::unique_lock<std::mutex> lock(mMutex);
				mCv.wait(lock, [this] {return mProcessed; });
			}
		}
		else
		{
			render();
		}

		const auto now = static_cast<float>(glfwGetTime());

		mDt = now - lastTime;

		OutputDebugString((std::to_string(mDt) + "\n").c_str());
		lastTime = now;
	}

	{
		std::lock_guard<std::mutex>lock(mMutex);
		mReady = true;
		mEnd = true;
	}

	mCv.notify_one();

	updateThread.join();
}

void Game::swap()
{
	mOctreeBody.clear();
	mOctreeBody.shrink_to_fit();
	mHoldingContainer->update();
	mOctree->updateTree();
	mOctree->getRigidBodies(mOctreeBody);
	updateObjectRender();
	mSceneGraph->swap();
}

void Game::update()
{
	while(true)
	{
		std::unique_lock<std::mutex> lock(mMutex);
		mCv.wait(lock, [this] {return mReady; });

		if (mEnd)
		{
			lock.unlock();
			return;
		}

		simulationLoop();

		mProcessed = true;
		mReady = false;
		lock.unlock();
		mCv.notify_one();
	}
}

void Game::simulationLoop()
{
	// Calculate the physic calculations on all objects (e.g. new position, velocity, etc)
	calculateObjectPhysics();

	// Clear the manifold so that we can calculate all collisions for this simulation loop
	mManifold->clear();
	mPossibleCollisions.clear();
	mPossibleCollisions.shrink_to_fit();
	mOctree->getPossibleCollisions(mPossibleCollisions);
	mPossibleRigidBodyCollisions.clear();

	for (auto& possibleCollision : mPossibleCollisions)
	{
		mPossibleRigidBodyCollisions.emplace(
			std::pair < RigidBody *, RigidBody *>(possibleCollision.rigidBody1, possibleCollision.rigidBody2)
		);

		mPossibleRigidBodyCollisions.emplace(
			std::pair < RigidBody *, RigidBody *>(possibleCollision.rigidBody2, possibleCollision.rigidBody1)
		);
	}

	// Find dynamic collisions for all objects and add to contact manifold
	dynamicCollisionDetection();

	mManifold->sort();

	// Handle dynamic collision responses using the contact manifold
	dynamicCollisionResponse();

	// Update the physics calculations on all objects (e.g. new position, velocity, etc)
	updateObjectPhysics();
}

void Game::calculateObjectPhysics() const
{
	mSceneGraph->updateSceneGraph(getUpdateDt(), glm::mat4(1.0f));

	for (auto octreeRigidBody : mOctreeBody)
	{
		octreeRigidBody->calculatePhysics(getUpdateDt(), 0.0f);
	}
}

void Game::dynamicCollisionDetection() const
{
	OutputDebugString((std::to_string(mPossibleCollisions.size()) + "\n").c_str());
	for (auto sceneRigidBody : mSceneBody)
	{
		for (auto octreeRigidBody : mOctreeBody)
		{
			CollisionDetection::dynamicCollisionDetection(sceneRigidBody, octreeRigidBody, mManifold, 0.0f);
		}
	}

	for (auto possibleCollision : mPossibleCollisions)
	{
		CollisionDetection::dynamicCollisionDetection(possibleCollision.rigidBody1, possibleCollision.rigidBody2, mManifold, 0.0f);
	}
}

void Game::dynamicCollisionResponse() const
{
	OutputDebugString((std::to_string(mManifold->getNumPoints()) + "\n").c_str());

	auto lastCollisionTime = 0.0f;
	while(lastCollisionTime < 1.0f && mManifold->getNumPoints())
	{
		auto moved1 = false;
		auto moved2 = false;
		auto point = mManifold->getPoint(0);
		lastCollisionTime = point.mTime;

		if (lastCollisionTime == 0.0f)
		{
			//auto i = 0;
		}
		
		CollisionResponse::dynamicCollisionResponse(point, moved1, moved2);

		mManifold->remove(0);

		const auto timeLeft = getUpdateDt() - getUpdateDt() * lastCollisionTime;

		auto rigidBody1 = point.mContactId1;

		if (moved1)
		{
			rigidBody1->update();
			rigidBody1->calculatePhysics(timeLeft, lastCollisionTime);

		}

		auto rigidBody2 = point.mContactId2;

		if (moved2)
		{
			rigidBody2->update();
			rigidBody2->calculatePhysics(timeLeft, lastCollisionTime);
		}
		//continue;

		//float timeLeft = getUpdateDt() - (getUpdateDt() * lastCollisionTime);

		//RigidBody * rigidBody1 = point.mContactId1;
		
		//if (moved1)
		//{
		//	rigidBody1->update();
		//	rigidBody1->calculatePhysics(timeLeft, lastCollisionTime);

		//	for (int i = 0; i < mManifold->getNumPoints(); i++)
		//	{
		//		auto checkPoint = mManifold->getPoint(i);

		//		if (checkPoint.mContactId1 == rigidBody1 || checkPoint.mContactId2 == rigidBody1)
		//		{
		//			mManifold->remove(i);
		//			i--;
		//		}
		//	}
		//}

		////RigidBody * rigidBody2 = point.mContactId2;

		//if (moved2)
		//{
		//	rigidBody2->update();
		//	rigidBody2->calculatePhysics(timeLeft, lastCollisionTime);

		//	for (int i = 0; i < mManifold->getNumPoints(); i++)
		//	{
		//		auto checkPoint = mManifold->getPoint(i);

		//		if (checkPoint.mContactId1 == rigidBody2 || checkPoint.mContactId2 == rigidBody2)
		//		{
		//			mManifold->remove(i);
		//			i--;
		//		}
		//	}
		//}

		//if (moved1)
		//{
		//	for (int i = 0; i < sceneBody.size(); i++)
		//	{
		//		CollisionDetection::dynamicCollisionDetection(sceneBody[i], rigidBody1, mManifold, lastCollisionTime);
		//	}

		//	auto result = mPossibleRigidBodyCollisions.equal_range(rigidBody1);

		//	for (auto it = result.first; it != result.second; ++it)
		//	{
		//		CollisionDetection::dynamicCollisionDetection(it->first, it->second, mManifold, lastCollisionTime);
		//	}
		//}

		//if (moved2)
		//{
		//	for (int i = 0; i < sceneBody.size(); i++)
		//	{
		//		CollisionDetection::dynamicCollisionDetection(sceneBody[i], rigidBody2, mManifold, lastCollisionTime);
		//	}

		//	auto result = mPossibleRigidBodyCollisions.equal_range(rigidBody2);

		//	for (auto it = result.first; it != result.second; ++it)
		//	{
		//		if (moved1 && it->second == rigidBody1)
		//		{
		//			continue;
		//		}
		//		
		//		CollisionDetection::dynamicCollisionDetection(it->first, it->second, mManifold, lastCollisionTime);
		//	}
		//}

		//mManifold->sort();

		//if (moved1)
		//{
		//	for (int i = 0; i < mManifold->getNumPoints(); i++)
		//	{
		//		auto checkPoint = mManifold->getPoint(i);

		//		if (checkPoint.mContactId1 == rigidBody1 || checkPoint.mContactId2 == rigidBody1 && checkPoint.mTime == lastCollisionTime)
		//		{
		//			mManifold->remove(i);
		//			i--;
		//		}
		//	}
		//}

		//if (moved2)
		//{
		//	for (int i = 0; i < mManifold->getNumPoints(); i++)
		//	{
		//		auto checkPoint = mManifold->getPoint(i);

		//		if (checkPoint.mContactId1 == rigidBody2 || checkPoint.mContactId2 == rigidBody2 && checkPoint.mTime == lastCollisionTime)
		//		{
		//			mManifold->remove(i);
		//			i--;
		//		}
		//	}
		//}
	}
}

void Game::updateObjectPhysics() const
{
	for (auto sceneRigidBody : mSceneBody)
	{
		sceneRigidBody->update();
	}

	for (auto octreeRigidBody : mOctreeBody)
	{
		octreeRigidBody->update();
	}
}

void Game::updateObjectRender() const
{
	for (auto i : mSceneBody)
	{
		i->updateRender();
	}

	for (auto i : mOctreeBody)
	{
		i->updateRender();
	}
}

void Game::render() const
{
	mCamera->update();

	auto perspective = glm::perspective(45.0f, static_cast<float>(GLFWWindow::instance()->getWidth()) / static_cast<float>(GLFWWindow::instance()->getHeight()), 0.1f, 1000.0f);
	auto view = mCamera->getViewMatrix();

	mSphereShader->useShader();

	auto perspectiveLocation = glGetUniformLocation(mSphereShader->getShaderId(), "perspective");
	glUniformMatrix4fv(perspectiveLocation, 1, GL_FALSE, &perspective[0][0]);

	auto viewLocation = glGetUniformLocation(mSphereShader->getShaderId(), "view");
	glUniformMatrix4fv(viewLocation, 1, GL_FALSE, &view[0][0]);

	for (auto i : mOctreeBody)
	{
		i->render(mSphereShader);
	}

	mPlaneShader->useShader();

	perspectiveLocation = glGetUniformLocation(mPlaneShader->getShaderId(), "perspective");
	glUniformMatrix4fv(perspectiveLocation, 1, GL_FALSE, &perspective[0][0]);

	viewLocation = glGetUniformLocation(mPlaneShader->getShaderId(), "view");
	glUniformMatrix4fv(viewLocation, 1, GL_FALSE, &view[0][0]);

	glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);
	
	for (auto i : mSceneBody)
	{
		i->render(mPlaneShader);
	}

	glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
	
	mOctree->render();
}

#include "Game.h"
#include "gl.h"
#include <GLFW/glfw3.h>
#include "GLFWWindow.h"
#include <string>
#include "Shader.h"
#include "Matrix4f.h"
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

Camera * Game::camera = new Camera(Vector3F(0, 25, 50), Vector3F(0, 1, 0), Vector3F(0, 0, 0));

Game::Game()
{
	mSphereShader = new Shader("sphereVertex.vert", "sphereFragment.frag");
	mPlaneShader = new Shader("planeVertex.vert", "planeFragment.frag");

	octree = new Octree(Vector3F(0, 0, 0), Vector3F(25, 25, 25));
	mHoldingContainer = new HoldingContainer(octree, Vector3F(0, 18, 0), Vector3F(2, 2, 2));
	mManifold = new ContactManifold();
	
	sceneGraph = new IdentityNode();

	TranslationNode * planeHolesTranslation = new TranslationNode(Vector3F(0, -2, 0));
	sceneGraph->addChild(planeHolesTranslation);

	TranslationAnimation * planeHolesAnimation = new TranslationAnimation(
		[](Vector3F& pVector, float pDt, bool pDirection)
	{
		if (!pDirection)
		{
			float newX = pVector.getX() + pDt;

			if (newX > 15.0f)
			{
				newX = 15.0f;
			}

			pVector = Vector3F(newX, pVector.getY(), pVector.getZ());
		}
		else
		{
			float newX = pVector.getX() - pDt;

			if (newX < 0.0f)
			{
				newX = 0.0f;
			}

			pVector = Vector3F(newX, pVector.getY(), pVector.getZ());
		}
	}
	, '4', '3');

	TranslationNode * planeHolesTranslationWithAnimation = new TranslationNode(Vector3F(0, 0, 0), planeHolesAnimation);
	planeHolesTranslation->addChild(planeHolesTranslationWithAnimation);

	PlaneHoles * planeHoles = new PlaneHoles(Vector3F(1, 1, 1), -1, Vector3F(0, 0, 0), Vector3F(0, 0, 0), Vector3F(0, 0, 0));
	RigidBodyNode * planeHolesNode = new RigidBodyNode(planeHoles);
	planeHolesTranslationWithAnimation->addChild(planeHolesNode);
	sceneBody.push_back(planeHoles);

	RotationAnimation * propellerAnimation = new RotationAnimation(
		[](Vector3F& pRotationAxis, float & pRotationAngle, float pDt, float pAngleChange)
	{
		pRotationAngle += pDt * pAngleChange;
	}
	, 'R', 'F');

	RotationNode * propellerRotationWithAnimation = new RotationNode(Vector3F(0, 1, 0), 0, propellerAnimation);

	planeHolesTranslationWithAnimation->addChild(propellerRotationWithAnimation);

	TranslationNode * propellerCentreTranslation = new TranslationNode(Vector3F(0, 3, 0));

	propellerRotationWithAnimation->addChild(propellerCentreTranslation);

	Cylinder * cylinder = new Cylinder(Vector3F(1, 1, 1), -1, Vector3F(0, 0, 0), Vector3F(0, 0, 0), Vector3F(0, 0, 0));

	RigidBodyNode * propellerCentreNode = new RigidBodyNode(cylinder);

	propellerCentreTranslation->addChild(propellerCentreNode);
	sceneBody.push_back(cylinder);

	TranslationNode * propellerBladeTranslation = new TranslationNode(Vector3F(0, 6, 0));

	propellerRotationWithAnimation->addChild(propellerBladeTranslation);

	RotationNode * propellerBladeRotation = new RotationNode(Vector3F(1, 0, 0), 90);

	propellerBladeTranslation->addChild(propellerBladeRotation);

	RotationNode * propellerBlade1Rotation = new RotationNode(Vector3F(0, 0, 1), 0);

	propellerBladeRotation->addChild(propellerBlade1Rotation);

	TranslationNode * propellerBlade1Translation = new TranslationNode(Vector3F(0, 1.5f, 0));

	propellerBlade1Rotation->addChild(propellerBlade1Translation);

	cylinder = new Cylinder(Vector3F(1, 1, 1), -1, Vector3F(0, 0, 0), Vector3F(0, 0, 0),  Vector3F(0, 0, 0));

	RigidBodyNode * propellerBlade1Node = new RigidBodyNode(cylinder);

	propellerBlade1Translation->addChild(propellerBlade1Node);
	sceneBody.push_back(cylinder);

	RotationNode * propellerBlade2Rotation = new RotationNode(Vector3F(0, 0, 1), 120);

	propellerBladeRotation->addChild(propellerBlade2Rotation);

	TranslationNode * propellerBlade2Translation = new TranslationNode(Vector3F(0, 1.5f, 0));

	propellerBlade2Rotation->addChild(propellerBlade2Translation);

	cylinder = new Cylinder(Vector3F(1, 1, 1), -1, Vector3F(0, 0, 0), Vector3F(0, 0, 0), Vector3F(0, 0, 0));

	RigidBodyNode * propellerBlade2Node = new RigidBodyNode(cylinder);

	propellerBlade2Translation->addChild(propellerBlade2Node);
	sceneBody.push_back(cylinder);

	RotationNode * propellerBlade3Rotation = new RotationNode(Vector3F(0, 0, 1), 240);

	propellerBladeRotation->addChild(propellerBlade3Rotation);

	TranslationNode * propellerBlade3Translation = new TranslationNode(Vector3F(0, 1.5f, 0));

	propellerBlade3Rotation->addChild(propellerBlade3Translation);

	cylinder = new Cylinder(Vector3F(1, 1, 1), -1, Vector3F(0, 0, 0), Vector3F(0, 0, 0), Vector3F(0, 0, 0));

	RigidBodyNode * propellerBlade3Node = new RigidBodyNode(cylinder);

	propellerBlade3Translation->addChild(propellerBlade3Node);
	sceneBody.push_back(cylinder);

	TranslationNode * planeTranslation = new TranslationNode(Vector3F(0, -8, 0));

	sceneGraph->addChild(planeTranslation);

	TranslationAnimation * planeAnimation = new TranslationAnimation(
		[](Vector3F&pVector, float pDt, bool pDirection)
	{
		if (!pDirection)
		{
			float newX = pVector.getX() + pDt;

			if (newX > 15.0f)
			{
				newX = 15.0f;
			}

			pVector = Vector3F(newX, pVector.getY(), pVector.getZ());
		}
		else
		{
			float newX = pVector.getX() - pDt;

			if (newX < 0.0f)
			{
				newX = 0.0f;
			}

			pVector = Vector3F(newX, pVector.getY(), pVector.getZ());
		}
	}
	, '6', '5');

	TranslationNode * planeTranslationWithAnimation = new TranslationNode(Vector3F(0, 0, 0), planeAnimation);

	planeTranslation->addChild(planeTranslationWithAnimation);

	Plane * plane = new Plane(-1, Vector3F(5, 1, 5), Vector3F(0, 0, 0), Vector3F(0, 0, 0), Vector3F(0, 0, 0));

	RigidBodyNode * planeNode = new RigidBodyNode(plane);

	planeTranslationWithAnimation->addChild(planeNode);
	sceneBody.push_back(plane);

	TranslationNode * leftPlaneTranslation = new TranslationNode(Vector3F(-5, 0, 0));

	sceneGraph->addChild(leftPlaneTranslation);

	RotationNode * leftPlaneRotation = new RotationNode(Vector3F(0, 0, 1), 90);

	leftPlaneTranslation->addChild(leftPlaneRotation);

	plane = new Plane(-1, Vector3F(10, 1, 5), Vector3F(0, 0, 0), Vector3F(0, 0, 0), Vector3F(0, 0, 0));

	RigidBodyNode * leftPlaneNode = new RigidBodyNode(plane);

	leftPlaneRotation->addChild(leftPlaneNode);
	sceneBody.push_back(plane);

	TranslationNode * rightPlaneTranslation = new TranslationNode(Vector3F(5, 0, 0));

	sceneGraph->addChild(rightPlaneTranslation);

	RotationNode * rightPlaneRotation = new RotationNode(Vector3F(0, 0, 1), -90);

	rightPlaneTranslation->addChild(rightPlaneRotation);

	plane = new Plane(-1, Vector3F(10, 1, 5), Vector3F(0, 0, 0), Vector3F(0, 0, 0), Vector3F(0, 0, 0));

	RigidBodyNode * rightPlaneNode = new RigidBodyNode(plane);

	rightPlaneRotation->addChild(rightPlaneNode);
	sceneBody.push_back(plane);

	TranslationNode * frontPlaneTranslation = new TranslationNode(Vector3F(0, 0, 5));

	sceneGraph->addChild(frontPlaneTranslation);

	RotationNode * frontPlaneRotation = new RotationNode(Vector3F(1, 0, 0), 90);

	frontPlaneTranslation->addChild(frontPlaneRotation);

	plane = new Plane(-1, Vector3F(5, 1, 10), Vector3F(0, 0, 0), Vector3F(0, 0, 0), Vector3F(0, 0, 0));

	RigidBodyNode * frontPlaneNode = new RigidBodyNode(plane);

	frontPlaneRotation->addChild(frontPlaneNode);

	sceneBody.push_back(plane);

	TranslationNode * backPlaneTranslation = new TranslationNode(Vector3F(0, 0, -5));

	sceneGraph->addChild(backPlaneTranslation);

	RotationNode * backPlaneRotation = new RotationNode(Vector3F(1, 0, 0), -90);

	backPlaneTranslation->addChild(backPlaneRotation);

	plane = new Plane(-1, Vector3F(5, 1, 10), Vector3F(0, 0, 0), Vector3F(0, 0, 0), Vector3F(0, 0, 0));

	RigidBodyNode * backPlaneNode = new RigidBodyNode(plane);

	backPlaneRotation->addChild(backPlaneNode);

	sceneBody.push_back(plane);

	TranslationNode * bowlTranslation = new TranslationNode(Vector3F(0, 10, 0));

	sceneGraph->addChild(bowlTranslation);

	RotationNode * bowlRotation = new RotationNode(Vector3F(1, 0, 0), 90);

	bowlTranslation->addChild(bowlRotation);

	Bowl * bowl = new Bowl(Vector3F(25, 25, 25), -1, Vector3F(0, 0, 0), Vector3F(0, 0, 0), Vector3F(0, 0, 0));

	RigidBodyNode * bowlNode = new RigidBodyNode(bowl);

	bowlRotation->addChild(bowlNode);

	sceneBody.push_back(bowl);
}

Game::~Game()
{
}

void Game::run()
{
	swap();
	std::thread update_thread(&Game::update, this);
	
	double lastTime = glfwGetTime();
	while (GLFWWindow::instance()->windowEvents())
	{
		if (mAddSphere)
		{
			mHoldingContainer->addRigidBody();
			mAddSphere = false;
		}

		swap();
		
		if (!mPause)
		{
			//std::thread upda = std::thread(&Game::update, this);
			//render();
			//upda.join();

			{
				std::lock_guard<std::mutex> lock(m);
				ready = true;
				processed = false;
			}

			cv.notify_one();

			render();

			{
				std::unique_lock<std::mutex> lock(m);
				cv.wait(lock, [this] {return processed; });
			}

			//update();
			//render();
		}
		else
		{
			render();
		}

		/*update();
		render();*/

		double now = glfwGetTime();

		mDt = now - lastTime;

		OutputDebugString((std::to_string(mDt) + "\n").c_str());
		lastTime = now;
	}

	{
		std::lock_guard<std::mutex>lock(m);
		ready = true;
		end = true;
	}

	cv.notify_one();

	update_thread.join();
}

void Game::swap()
{
	octreeBody.clear();
	octreeBody.shrink_to_fit();
	mHoldingContainer->update();
	octree->UpdateTree();
	octree->GetRigidBodies(octreeBody);
	updateObjectRender();
	sceneGraph->swap();
}

void Game::update()
{
	while(true)
	{
		std::unique_lock<std::mutex> lock(m);
		cv.wait(lock, [this] {return ready; });

		if (end)
		{
			lock.unlock();
			return;
		}

		simulationLoop();

		processed = true;
		ready = false;
		lock.unlock();
		cv.notify_one();
	}
	//simulationLoop();
}

void Game::simulationLoop()
{
	// Calculate the physic calculations on all objects (e.g. new position, velocity, etc)
	calculateObjectPhysics();

	// Clear the manifold so that we can calculate all collisions for this simulation loop
	mManifold->clear();
	mPossibleCollisions.clear();
	mPossibleCollisions.shrink_to_fit();
	octree->GetPossibleCollisions(mPossibleCollisions);
	mPossibleRigidBodyCollisions.clear();

	for (int i = 0; i < mPossibleCollisions.size(); i++)
	{
		mPossibleRigidBodyCollisions.emplace(
			std::pair < RigidBody *, RigidBody *>(mPossibleCollisions[i].rigidBody1, mPossibleCollisions[i].rigidBody2)
		);

		mPossibleRigidBodyCollisions.emplace(
			std::pair < RigidBody *, RigidBody *>(mPossibleCollisions[i].rigidBody2, mPossibleCollisions[i].rigidBody1)
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
	sceneGraph->updateSceneGraph(getUpdateDt(), Matrix4F::createIdentity());

	for (int i = 0; i < octreeBody.size(); i++)
	{
		octreeBody[i]->calculatePhysics(getUpdateDt());
	}
}

void Game::dynamicCollisionDetection() const
{
	OutputDebugString((std::to_string(mPossibleCollisions.size()) + "\n").c_str());
	for (int i = 0; i < sceneBody.size(); i++)
	{
		for (int j = 0; j < octreeBody.size(); j++)
		{
			CollisionDetection::dynamicCollisionDetection(sceneBody[i], octreeBody[j], mManifold);
		}
	}

	for (int i = 0; i < mPossibleCollisions.size(); i++)
	{
		CollisionDetection::dynamicCollisionDetection(mPossibleCollisions[i].rigidBody1, mPossibleCollisions[i].rigidBody2, mManifold);
	}
}

void Game::dynamicCollisionResponse() const
{
	OutputDebugString((std::to_string(mManifold->getNumPoints()) + "\n").c_str());
	for (auto collision = 0; collision < mManifold->getNumPoints(); ++collision)
	{
		bool moved1 = false;
		bool moved2 = false;
		auto point = mManifold->getPoint(collision);
		CollisionResponse::dynamicCollisionResponse(point, moved1, moved2);
	}
}

void Game::updateObjectPhysics() const
{
	for (int i = 0; i < sceneBody.size(); i++)
	{
		sceneBody[i]->update();
	}

	for (int i = 0; i < octreeBody.size(); i++)
	{
		octreeBody[i]->update();
	}
}

void Game::updateObjectRender() const
{
	for (int i = 0; i < sceneBody.size(); i++)
	{
		sceneBody[i]->updateRender();
	}

	for (int i = 0; i < octreeBody.size(); i++)
	{
		octreeBody[i]->updateRender();
	}
}

void Game::render() const
{
	camera->Update();

	auto perspective = Matrix4F::createPerspective(45.0f, 600.0f / 600.0f, 0.1f, 1000.0f);
	auto view = camera->GetViewMatrix();

	mSphereShader->useShader();

	auto perspectiveLocation = glGetUniformLocation(mSphereShader->getShaderId(), "perspective");
	perspective.useMatrix(perspectiveLocation);

	auto viewLocation = glGetUniformLocation(mSphereShader->getShaderId(), "view");
	view.useMatrix(viewLocation);

	for (int i = 0; i < octreeBody.size(); i++)
	{
		octreeBody[i]->render(mSphereShader);
	}

	mPlaneShader->useShader();

	perspectiveLocation = glGetUniformLocation(mPlaneShader->getShaderId(), "perspective");
	perspective.useMatrix(perspectiveLocation);

	viewLocation = glGetUniformLocation(mPlaneShader->getShaderId(), "view");
	view.useMatrix(viewLocation);

	for (int i = 0; i < sceneBody.size(); i++)
	{
		sceneBody[i]->render(mPlaneShader);
	}

	octree->Render();
}

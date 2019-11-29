#include "GLFWWindow.h"

#include "Game.h"

GLFWWindow * GLFWWindow::mInstance = nullptr;

GLFWWindow::GLFWWindow()
{
	glfwInit();
	glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
	glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
	glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);

	mWindow = glfwCreateWindow(mWidth, mHeight, "Simulation ACW", nullptr, nullptr);

	if (mWindow == nullptr)
	{
		glfwTerminate();
	}

	glfwMakeContextCurrent(mWindow);
	glfwSwapInterval(1);

	glClearColor(0.0f, 0.0f, 0.0f, 1.0f);
	glClearDepth(1.0f);

	glEnable(GL_DEPTH_TEST);
	glCullFace(GL_NONE);
	glEnable(GL_BLEND);
	glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
}

GLFWWindow * GLFWWindow::instance()
{
	if (!mInstance)
	{
		mInstance = new GLFWWindow();
	}

	return mInstance;
}

bool GLFWWindow::windowEvents()
{
	glfwSwapBuffers(mWindow);
	glfwPollEvents();

	if (glfwGetKey(mWindow, GLFW_KEY_UP) == GLFW_PRESS)
	{
		Game::mCamera->panForward(true);
	}
	else
	{
		Game::mCamera->panForward(false);
	}

	if (glfwGetKey(mWindow, GLFW_KEY_DOWN) == GLFW_PRESS)
	{
		Game::mCamera->panBackward(true);
	}
	else
	{
		Game::mCamera->panBackward(false);
	}

	if (glfwGetKey(mWindow, GLFW_KEY_W) == GLFW_PRESS)
	{
		Game::mCamera->rotateUp(true);
	}
	else
	{
		Game::mCamera->rotateUp(false);
	}

	if (glfwGetKey(mWindow, GLFW_KEY_S) == GLFW_PRESS)
	{
		Game::mCamera->rotateDown(true);
	}
	else
	{
		Game::mCamera->rotateDown(false);
	}

	if (glfwGetKey(mWindow, GLFW_KEY_A) == GLFW_PRESS)
	{
		Game::mCamera->rotateLeft(true);
	}
	else
	{
		Game::mCamera->rotateLeft(false);
	}

	if (glfwGetKey(mWindow, GLFW_KEY_D) == GLFW_PRESS)
	{
		Game::mCamera->rotateRight(true);
	}
	else
	{
		Game::mCamera->rotateRight(false);
	}

	{
		static auto pause = true;
		static auto pauseCounter = 0;
		if (pause && glfwGetKey(mWindow, GLFW_KEY_P) == GLFW_PRESS)
		{
			Game::setPause();
			pause = false;
			pauseCounter = 0;
		}
		else if (glfwGetKey(mWindow, GLFW_KEY_P) == GLFW_RELEASE)
		{
			pause = true;
		}

		if (!pause && pauseCounter > 30)
		{
			pause = true;
		}
		else if (!pause)
		{
			pauseCounter++;
		}
	}

	{
		static auto reset = true;
		static auto resetCounter = 0;
		
		if (reset && glfwGetKey(mWindow, GLFW_KEY_E) == GLFW_PRESS)
		{
			Game::setReset();
			reset = false;
			resetCounter = 0;
		}
		else if (glfwGetKey(mWindow, GLFW_KEY_E) == GLFW_RELEASE)
		{
			reset = true;
		}

		if (!reset && resetCounter > 30)
		{
			reset = true;
		}
		else if (!reset)
		{
			resetCounter++;
		}
	}

	{
		static auto timeScale = true;
		static auto timeScaleCounter = 0;
		
		if (timeScale && glfwGetKey(mWindow, GLFW_KEY_U) == GLFW_PRESS)
		{
			Game::changeTimeScale(true);
			timeScale = false;
			timeScaleCounter = 0;
		}
		else if (glfwGetKey(mWindow, GLFW_KEY_U) == GLFW_RELEASE)
		{
			timeScale = true;
		}

		if (!timeScale && timeScaleCounter > 30)
		{
			timeScale = true;
		}
		else if (!timeScale)
		{
			timeScaleCounter++;
		}
	}

	{
		static auto timeScale = true;
		static auto timeScaleCounter = 0;
		if (timeScale && glfwGetKey(mWindow, GLFW_KEY_J) == GLFW_PRESS)
		{
			Game::changeTimeScale(false);
			timeScale = false;
			timeScaleCounter = 0;
		}
		else if (glfwGetKey(mWindow, GLFW_KEY_J) == GLFW_RELEASE)
		{
			timeScale = true;
		}

		if (!timeScale && timeScaleCounter > 30)
		{
			timeScale = true;
		}
		else if (!timeScale)
		{
			timeScaleCounter++;
		}
	}

	{
		static auto angularDisable = true;
		static auto angularDisableCounter = 0;

		if (angularDisable && glfwGetKey(mWindow, GLFW_KEY_7) == GLFW_PRESS)
		{
			Game::setAngularDisable();
			angularDisable = false;
			angularDisableCounter = 0;
		}
		else if (glfwGetKey(mWindow, GLFW_KEY_7) == GLFW_RELEASE)
		{
			angularDisable = true;
		}

		if (!angularDisable && angularDisableCounter > 30)
		{
			angularDisable = true;
		}
		else if (!angularDisable)
		{
			angularDisableCounter++;
		}
	}

	{
		static auto octreeDisable = true;
		static auto octreeDisableCounter = 0;

		if (octreeDisable && glfwGetKey(mWindow, GLFW_KEY_Q) == GLFW_PRESS)
		{
			Game::setOctreeDisable();
			octreeDisable = false;
			octreeDisableCounter = 0;
		}
		else if (glfwGetKey(mWindow, GLFW_KEY_Q) == GLFW_RELEASE)
		{
			octreeDisable = true;
		}

		if (!octreeDisable && octreeDisableCounter > 30)
		{
			octreeDisable = true;
		}
		else if (!octreeDisable)
		{
			octreeDisableCounter++;
		}

	}
	
	{
		static auto friction = true;
		static auto frictionCounter = 0;
		if (friction && glfwGetKey(mWindow, GLFW_KEY_I) == GLFW_PRESS)
		{
			Game::changeFriction(true);
			friction = false;
			frictionCounter = 0;
		}
		else if (glfwGetKey(mWindow, GLFW_KEY_I) == GLFW_RELEASE)
		{
			friction = true;
		}

		if (!friction && frictionCounter > 30)
		{
			friction = true;
		}
		else if (!friction)
		{
			frictionCounter++;
		}
	}

	{
		static auto friction = true;
		static auto frictionCounter = 0;
		if (friction && glfwGetKey(mWindow, GLFW_KEY_K) == GLFW_PRESS)
		{
			Game::changeFriction(false);
			friction = false;
			frictionCounter = 0;
		}
		else if (glfwGetKey(mWindow, GLFW_KEY_K) == GLFW_RELEASE)
		{
			friction = true;
		}

		if (!friction && frictionCounter > 30)
		{
			friction = true;
		}
		else if (!friction)
		{
			frictionCounter++;
		}
	}

	{
		static auto elasticity = true;
		static auto elasticityCounter = 0;
		
		if (elasticity && glfwGetKey(mWindow, GLFW_KEY_O) == GLFW_PRESS)
		{
			Game::changeSphereElasticity(true);
			elasticity = false;
			elasticityCounter = 0;
		}
		else if (glfwGetKey(mWindow, GLFW_KEY_O) == GLFW_RELEASE)
		{
			elasticity = true;
		}

		if (!elasticity && elasticityCounter > 30)
		{
			elasticity = true;
		}
		else if (!elasticity)
		{
			elasticityCounter++;
		}
	}

	{
		static auto elasticity = true;
		static auto elasticityCounter = 0;
		
		if (elasticity && glfwGetKey(mWindow, GLFW_KEY_L) == GLFW_PRESS)
		{
			Game::changeSphereElasticity(false);
			elasticity = false;
			elasticityCounter = 0;
		}
		else if (glfwGetKey(mWindow, GLFW_KEY_L) == GLFW_RELEASE)
		{
			elasticity = true;
		}

		if (!elasticity && elasticityCounter > 30)
		{
			elasticity = true;
		}
		else if (!elasticity)
		{
			elasticityCounter++;
		}
	}

	{
		static auto size = true;
		static auto sizeCounter = 0;
		
		if (size && glfwGetKey(mWindow, GLFW_KEY_Y) == GLFW_PRESS)
		{
			Game::changeSphereSize(true);
			size = false;
			sizeCounter = 0;
		}
		else if (glfwGetKey(mWindow, GLFW_KEY_Y) == GLFW_RELEASE)
		{
			size = true;
		}

		if (!size && sizeCounter > 30)
		{
			size = true;
		}
		else if (!size)
		{
			sizeCounter++;
		}
	}

	{
		static auto size = true;
		static auto sizeCounter = 0;
		
		if (size && glfwGetKey(mWindow, GLFW_KEY_H) == GLFW_PRESS)
		{
			Game::changeSphereSize(false);
			size = false;
			sizeCounter = 0;
		}
		else if (glfwGetKey(mWindow, GLFW_KEY_H) == GLFW_RELEASE)
		{
			size = true;
		}

		if (!size && sizeCounter > 30)
		{
			size = true;
		}
		else if (!size)
		{
			sizeCounter++;
		}
	}

	{
		static auto size = true;
		static auto sizeCounter = 0;

		if (size && glfwGetKey(mWindow, GLFW_KEY_T) == GLFW_PRESS)
		{
			Game::changeCuboidSize(true);
			size = false;
			sizeCounter = 0;
		}
		else if (glfwGetKey(mWindow, GLFW_KEY_T) == GLFW_RELEASE)
		{
			size = true;
		}

		if (!size && sizeCounter > 30)
		{
			size = true;
		}
		else if (!size)
		{
			sizeCounter++;
		}
	}

	{
		static auto size = true;
		static auto sizeCounter = 0;

		if (size && glfwGetKey(mWindow, GLFW_KEY_G) == GLFW_PRESS)
		{
			Game::changeCuboidSize(false);
			size = false;
			sizeCounter = 0;
		}
		else if (glfwGetKey(mWindow, GLFW_KEY_G) == GLFW_RELEASE)
		{
			size = true;
		}

		if (!size && sizeCounter > 30)
		{
			size = true;
		}
		else if (!size)
		{
			sizeCounter++;
		}
	}

	if (glfwGetKey(mWindow, GLFW_KEY_1) == GLFW_PRESS)
	{
		Game::addSphere();
	}

	if (glfwGetKey(mWindow, GLFW_KEY_2) == GLFW_PRESS)
	{
		Game::addCube();
	}

	for (auto it = mKeyListener.begin(); it != mKeyListener.end(); ++it)
	{
		if (glfwGetKey(mWindow, it->first) == GLFW_PRESS)
		{
			it->second->keyPressed(it->first);
		}
	}
	
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

	return !glfwWindowShouldClose(mWindow);
}

GLFWWindow::~GLFWWindow()
{
	glfwTerminate();
	mInstance = nullptr;
}

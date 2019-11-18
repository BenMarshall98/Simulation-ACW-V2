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

	if (glfwGetKey(mWindow, GLFW_KEY_P) == GLFW_PRESS)
	{
		Game::setPause();
	}

	if (glfwGetKey(mWindow, GLFW_KEY_R) == GLFW_PRESS)
	{
		Game::setReset();
	}

	if (glfwGetKey(mWindow, GLFW_KEY_U) == GLFW_PRESS)
	{
		Game::changeTimeScale(true);
	}

	if (glfwGetKey(mWindow, GLFW_KEY_J) == GLFW_PRESS)
	{
		Game::changeTimeScale(false);
	}

	if (glfwGetKey(mWindow, GLFW_KEY_I) == GLFW_PRESS)
	{
		Game::changeFriction(true);
	}

	if (glfwGetKey(mWindow, GLFW_KEY_K) == GLFW_PRESS)
	{
		Game::changeFriction(false);
	}

	if (glfwGetKey(mWindow, GLFW_KEY_O) == GLFW_PRESS)
	{
		Game::changeSphereElasticity(true);
	}

	if (glfwGetKey(mWindow, GLFW_KEY_L) == GLFW_PRESS)
	{
		Game::changeSphereElasticity(false);
	}

	if (glfwGetKey(mWindow, GLFW_KEY_Y) == GLFW_PRESS)
	{
		Game::changeSphereSize(true);
	}

	if (glfwGetKey(mWindow, GLFW_KEY_H) == GLFW_PRESS)
	{
		Game::changeSphereSize(false);
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

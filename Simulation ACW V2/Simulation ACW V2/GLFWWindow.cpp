#include "GLFWWindow.h"

#include "Game.h"

GLFWWindow * GLFWWindow::mInstance = nullptr;

GLFWWindow::GLFWWindow()
{
	glfwInit();
	glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
	glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
	glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);

	window = glfwCreateWindow(mWidth, mHeight, "Simulation ACW", nullptr, nullptr);

	if (window == nullptr)
	{
		glfwTerminate();
	}

	glfwMakeContextCurrent(window);
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
	glfwSwapBuffers(window);
	glfwPollEvents();

	if (glfwGetKey(window, GLFW_KEY_UP) == GLFW_PRESS)
	{
		Game::camera->panForward(true);
	}
	else
	{
		Game::camera->panForward(false);
	}

	if (glfwGetKey(window, GLFW_KEY_DOWN) == GLFW_PRESS)
	{
		Game::camera->panBackward(true);
	}
	else
	{
		Game::camera->panBackward(false);
	}

	if (glfwGetKey(window, GLFW_KEY_W) == GLFW_PRESS)
	{
		Game::camera->rotateUp(true);
	}
	else
	{
		Game::camera->rotateUp(false);
	}

	if (glfwGetKey(window, GLFW_KEY_S) == GLFW_PRESS)
	{
		Game::camera->rotateDown(true);
	}
	else
	{
		Game::camera->rotateDown(false);
	}

	if (glfwGetKey(window, GLFW_KEY_A) == GLFW_PRESS)
	{
		Game::camera->rotateLeft(true);
	}
	else
	{
		Game::camera->rotateLeft(false);
	}

	if (glfwGetKey(window, GLFW_KEY_D) == GLFW_PRESS)
	{
		Game::camera->rotateRight(true);
	}
	else
	{
		Game::camera->rotateRight(false);
	}

	if (glfwGetKey(window, GLFW_KEY_P) == GLFW_PRESS)
	{
		Game::setPause();
	}

	if (glfwGetKey(window, GLFW_KEY_R) == GLFW_PRESS)
	{
		Game::setReset();
	}

	if (glfwGetKey(window, GLFW_KEY_U) == GLFW_PRESS)
	{
		Game::changeTimeScale(true);
	}

	if (glfwGetKey(window, GLFW_KEY_J) == GLFW_PRESS)
	{
		Game::changeTimeScale(false);
	}

	if (glfwGetKey(window, GLFW_KEY_I) == GLFW_PRESS)
	{
		Game::changeFriction(true);
	}

	if (glfwGetKey(window, GLFW_KEY_K) == GLFW_PRESS)
	{
		Game::changeFriction(false);
	}

	if (glfwGetKey(window, GLFW_KEY_O) == GLFW_PRESS)
	{
		Game::changeSphereElasticty(true);
	}

	if (glfwGetKey(window, GLFW_KEY_L) == GLFW_PRESS)
	{
		Game::changeSphereElasticty(false);
	}

	if (glfwGetKey(window, GLFW_KEY_Y) == GLFW_PRESS)
	{
		Game::changeSphereSize(true);
	}

	if (glfwGetKey(window, GLFW_KEY_H) == GLFW_PRESS)
	{
		Game::changeSphereSize(false);
	}

	if (glfwGetKey(window, GLFW_KEY_1) == GLFW_PRESS)
	{
		Game::addSphere();
	}

	for (auto it = keyListener.begin(); it != keyListener.end(); ++it)
	{
		if (glfwGetKey(window, it->first) == GLFW_PRESS)
		{
			it->second->keyPressed(it->first);
		}
	}
	
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

	return !glfwWindowShouldClose(window);
}

GLFWWindow::~GLFWWindow()
{
	glfwTerminate();
	mInstance = nullptr;
}

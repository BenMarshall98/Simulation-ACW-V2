#include "GLFWWindow.h"

GLFWWindow * GLFWWindow::mInstance = nullptr;

std::map<char, SceneGraphAnimation *> GLFWWindow::keyListener = std::map<char, SceneGraphAnimation *>();

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

	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

	return !glfwWindowShouldClose(window);
}

GLFWWindow::~GLFWWindow()
{
}

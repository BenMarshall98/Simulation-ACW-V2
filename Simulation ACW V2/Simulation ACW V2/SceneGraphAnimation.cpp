#include "SceneGraphAnimation.h"
#include "GLFWWindow.h"

SceneGraphAnimation::SceneGraphAnimation(char pKey1, char pKey2) : mKey1(pKey1), mKey2(pKey2)
{
	GLFWWindow::addKeyListener(pKey1, this);
	GLFWWindow::addKeyListener(pKey2, this);
}

SceneGraphAnimation::~SceneGraphAnimation()
{
	GLFWWindow::removeKeyListener(mKey1);
	GLFWWindow::removeKeyListener(mKey2);
}
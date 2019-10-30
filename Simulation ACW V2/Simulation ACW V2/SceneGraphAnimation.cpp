#include "SceneGraphAnimation.h"
#include "GLFWWindow.h"

SceneGraphAnimation::SceneGraphAnimation(char pKey1, char pKey2) : mKey1(pKey1), mKey2(pKey2)
{
	GLFWWindow::instance()->addKeyListener(pKey1, this);
	GLFWWindow::instance()->addKeyListener(pKey2, this);
}

SceneGraphAnimation::~SceneGraphAnimation()
{
	GLFWWindow::instance()->removeKeyListener(mKey1);
	GLFWWindow::instance()->removeKeyListener(mKey2);
}
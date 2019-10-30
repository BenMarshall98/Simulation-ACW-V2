#pragma once
#include "gl.h"
#include "GLFW/glfw3.h"
#include <map>
#include "SceneGraphAnimation.h"

class GLFWWindow
{
	static GLFWWindow * mInstance;

	std::map<char, SceneGraphAnimation *> keyListener;
	GLFWwindow * window;

	int mWidth = 800;
	int mHeight = 600;

	GLFWWindow();
public:

	~GLFWWindow();

	static GLFWWindow * instance();

	void addKeyListener(char pKey, SceneGraphAnimation * pAnimation)
	{
		keyListener[pKey] = pAnimation;
	}

	void removeKeyListener(char pKey)
	{
		auto it = keyListener.find(pKey);

		if (it != keyListener.end())
		{
			keyListener.erase(it);
		}
	}

	bool windowEvents();

	int getWidth() const
	{
		return mWidth;
	}

	int getHeight() const
	{
		return mHeight;
	}
};


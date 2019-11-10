#pragma once
#include "gl.h"
#include "GLFW/glfw3.h"
#include <map>
#include "SceneGraphAnimation.h"

class GLFWWindow
{
	static GLFWWindow * mInstance;

	std::map<char, SceneGraphAnimation *> mKeyListener;
	GLFWwindow * mWindow;

	int mWidth = 800;
	int mHeight = 600;

	GLFWWindow();
public:

	~GLFWWindow();

	static GLFWWindow * instance();

	void addKeyListener(const char pKey, SceneGraphAnimation * pAnimation)
	{
		mKeyListener[pKey] = pAnimation;
	}

	void removeKeyListener(const char pKey)
	{
		auto it = mKeyListener.find(pKey);

		if (it != mKeyListener.end())
		{
			mKeyListener.erase(it);
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


#pragma once

#include "glm/glm.hpp"

class Camera
{
	glm::mat4 mViewMatrix;
	glm::vec3 mUpDirection;
	glm::vec3 mEyePosition;
	glm::vec3 mTargetPosition;
	float mAngleSpeed = 15.0f;
	float mMovementSpeed = 5.0f;
	bool mRotateLeft = false;
	bool mRotateRight = false;
	bool mRotateUp = false;
	bool mRotateDown = false;
	bool mPanForward = false;
	bool mPanBackward = false;

	void rotateLeftRight(bool pLeft = true);
	void rotateUpDown(bool pUp = true);
	void panForwardBackward(bool pForward = true);

public:
	Camera(glm::vec3 pEyePosition, glm::vec3 pUpDirection, glm::vec3 pTargetPosition);
	~Camera() = default;

	Camera(const Camera &) = delete;
	Camera(Camera &&) = delete;
	Camera & operator= (const Camera &) = delete;
	Camera & operator= (Camera &&) = delete;

	void rotateLeft(const bool pKeyPressed)
	{
		mRotateLeft = pKeyPressed;
	}

	void rotateRight(const bool pKeyPressed)
	{
		mRotateRight = pKeyPressed;
	}

	void rotateUp(const bool pKeyPressed)
	{
		mRotateUp = pKeyPressed;
	}

	void rotateDown(const bool pKeyPressed)
	{
		mRotateDown = pKeyPressed;
	}

	void panForward(const bool pKeyPressed)
	{
		mPanForward = pKeyPressed;
	}

	void panBackward(const bool pKeyPressed)
	{
		mPanBackward = pKeyPressed;
	}

	void update();

	glm::mat4 getViewMatrix() const
	{
		return mViewMatrix;
	}
};
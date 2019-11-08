#pragma once

#include "Matrix4F.h"
#include "Vector3F.h"

class Camera
{
	Matrix4F mViewMatrix;
	Vector3F mUpDirection;
	Vector3F mEyePosition;
	Vector3F mTargetPosition;
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
	Camera(Vector3F pEyePosition, Vector3F pUpDirection, Vector3F pTargetPosition);
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

	[[nodiscard]] Matrix4F getViewMatrix() const
	{
		return mViewMatrix;
	}
};
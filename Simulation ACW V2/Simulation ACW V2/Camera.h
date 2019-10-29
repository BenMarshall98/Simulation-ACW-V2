#pragma once

#include "Matrix4f.h"
#include "Vector3f.h"

class Camera
{
	Matrix4F mViewMatrix;
	Vector3F mUpDirection;
	Vector3F mEyePosition;
	Vector3F mTargetPosition;
	double mAngleSpeed = 15.0f;
	float mMovementSpeed = 5.0f;
	bool mRotateLeft = false;
	bool mRotateRight = false;
	bool mRotateUp = false;
	bool mRotateDown = false;
	bool mPanForward = false;
	bool mPanBackward = false;

	void RotateLeftRight(bool left = true);
	void RotateUpDown(bool up = true);
	void PanForwardBackward(bool forward = true);

public:
	Camera(Vector3F pEyePosition, Vector3F pUpDirection, Vector3F pTargetPosition);
	~Camera();

	void RotateLeft(bool pKeyPressed)
	{
		mRotateLeft = pKeyPressed;
	}

	void RotateRight(bool pKeyPressed)
	{
		mRotateRight = pKeyPressed;
	}

	void RotateUp(bool pKeyPressed)
	{
		mRotateUp = pKeyPressed;
	}

	void RotateDown(bool pKeyPressed)
	{
		mRotateDown = pKeyPressed;
	}

	void PanForward(bool pKeyPressed)
	{
		mPanForward = pKeyPressed;
	}

	void PanBackward(bool pKeyPressed)
	{
		mPanBackward = pKeyPressed;
	}

	void Update();

	Matrix4F GetViewMatrix() const
	{
		return mViewMatrix;
	}
};
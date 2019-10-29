#include "Camera.h"
#include "Game.h"

Camera::Camera(Vector3F pEyePosition, Vector3F pUpDirection, Vector3F pTargetPosition) :
	mEyePosition(pEyePosition), mUpDirection(pUpDirection), mTargetPosition(pTargetPosition)
{
	Update();
}

Camera::~Camera()
{
}

void Camera::RotateLeftRight(bool left)
{
	double angleChange = mAngleSpeed * Game::getRenderDt();

	if (left)
	{
		angleChange = -angleChange;
	}

	Vector3F zAxis = (mTargetPosition - mEyePosition).normalise();
	Vector3F xAxis = zAxis.cross(mUpDirection).normalise();
	Vector3F yAxis = zAxis.cross(xAxis);

	Matrix4F leftRightMat = Matrix4F::createRotation(yAxis, angleChange);

	mTargetPosition = mEyePosition + leftRightMat * zAxis;
}

void Camera::RotateUpDown(bool up)
{
	float angleChange = mAngleSpeed * Game::getRenderDt();

	if (!up)
	{
		angleChange = -angleChange;
	}

	Vector3F zAxis = (mTargetPosition - mEyePosition).normalise();
	Vector3F xAxis = zAxis.cross(mUpDirection).normalise();

	Matrix4F leftRightMat = Matrix4F::createRotation(xAxis, angleChange);

	mTargetPosition = mEyePosition + leftRightMat * zAxis;
	mUpDirection = mUpDirection * leftRightMat;
}

void Camera::PanForwardBackward(bool forward)
{
	float movementChange = mMovementSpeed * Game::getRenderDt();

	if (!forward)
	{
		movementChange = -movementChange;
	}

	Vector3F zAxis = (mTargetPosition - mEyePosition).normalise();
	Vector3F movement = zAxis * movementChange;

	mEyePosition = mEyePosition + movement;
	mTargetPosition = mTargetPosition + movement;
}

void Camera::Update()
{
	if (mRotateLeft && !mRotateRight)
	{
		RotateLeftRight(true);
	}
	else if (!mRotateLeft && mRotateRight)
	{
		RotateLeftRight(false);
	}

	if (mRotateUp && !mRotateDown)
	{
		RotateUpDown(true);
	}
	else if (!mRotateUp && mRotateDown)
	{
		RotateUpDown(false);
	}

	if (mPanForward && !mPanBackward)
	{
		PanForwardBackward(true);
	}
	else if (!mPanForward && mPanBackward)
	{
		PanForwardBackward(false);
	}

	mViewMatrix = Matrix4F::createLookAt(mEyePosition, mTargetPosition, mUpDirection);
}
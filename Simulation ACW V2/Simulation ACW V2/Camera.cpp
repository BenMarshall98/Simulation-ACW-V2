#include "Camera.h"
#include "Game.h"

Camera::Camera(const Vector3F pEyePosition, const Vector3F pUpDirection, const Vector3F pTargetPosition) :
	 mUpDirection(pUpDirection), mEyePosition(pEyePosition), mTargetPosition(pTargetPosition)
{
	update();
}

void Camera::rotateLeftRight(const bool pLeft)
{
	auto angleChange = mAngleSpeed * Game::getRenderDt();

	if (pLeft)
	{
		angleChange = -angleChange;
	}

	const auto zAxis = (mTargetPosition - mEyePosition).normalize();
	const auto xAxis = zAxis.cross(mUpDirection).normalize();
	const auto yAxis = zAxis.cross(xAxis);

	const auto leftRightMat = Matrix4F::createRotation(yAxis, angleChange);

	mTargetPosition = mEyePosition + leftRightMat * zAxis;
}

void Camera::rotateUpDown(const bool pUp)
{
	auto angleChange = mAngleSpeed * Game::getRenderDt();

	if (!pUp)
	{
		angleChange = -angleChange;
	}

	const auto zAxis = (mTargetPosition - mEyePosition).normalize();
	const auto xAxis = zAxis.cross(mUpDirection).normalize();

	const auto leftRightMat = Matrix4F::createRotation(xAxis, angleChange);

	mTargetPosition = mEyePosition + leftRightMat * zAxis;
	mUpDirection = mUpDirection * leftRightMat;
}

void Camera::panForwardBackward(const bool pForward)
{
	auto movementChange = mMovementSpeed * Game::getRenderDt();

	if (!pForward)
	{
		movementChange = -movementChange;
	}

	const auto zAxis = (mTargetPosition - mEyePosition).normalize();
	const auto movement = zAxis * movementChange;

	mEyePosition = mEyePosition + movement;
	mTargetPosition = mTargetPosition + movement;
}

void Camera::update()
{
	if (mRotateLeft && !mRotateRight)
	{
		rotateLeftRight(true);
	}
	else if (!mRotateLeft && mRotateRight)
	{
		rotateLeftRight(false);
	}

	if (mRotateUp && !mRotateDown)
	{
		rotateUpDown(true);
	}
	else if (!mRotateUp && mRotateDown)
	{
		rotateUpDown(false);
	}

	if (mPanForward && !mPanBackward)
	{
		panForwardBackward(true);
	}
	else if (!mPanForward && mPanBackward)
	{
		panForwardBackward(false);
	}

	mViewMatrix = Matrix4F::createLookAt(mEyePosition, mTargetPosition, mUpDirection);
}
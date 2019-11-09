#include "Camera.h"
#include "Game.h"

Camera::Camera(const glm::vec3 pEyePosition, const glm::vec3 pUpDirection, const glm::vec3 pTargetPosition) :
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

	const auto zAxis = normalize(mTargetPosition - mEyePosition);
	const auto xAxis = normalize(cross(zAxis, mUpDirection));
	const auto yAxis = cross(zAxis, xAxis);

	const auto leftRightMat = rotate(glm::mat4(1.0f), angleChange, yAxis);

	mTargetPosition = mEyePosition + glm::vec3(leftRightMat * glm::vec4(zAxis, 0.0f));
}

void Camera::rotateUpDown(const bool pUp)
{
	auto angleChange = mAngleSpeed * Game::getRenderDt();

	if (!pUp)
	{
		angleChange = -angleChange;
	}

	const auto zAxis = normalize(mTargetPosition - mEyePosition);
	const auto xAxis = normalize(cross(zAxis, mUpDirection));

	const auto leftRightMat = rotate(glm::mat4(1.0f), angleChange, xAxis);

	mTargetPosition = mEyePosition + glm::vec3(leftRightMat * glm::vec4(zAxis, 0.0f));
	mUpDirection = glm::vec3(glm::vec4(mUpDirection, 0.0f) * leftRightMat);
}

void Camera::panForwardBackward(const bool pForward)
{
	auto movementChange = mMovementSpeed * Game::getRenderDt();

	if (!pForward)
	{
		movementChange = -movementChange;
	}

	const auto zAxis = normalize(mTargetPosition - mEyePosition);
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

	mViewMatrix = lookAt(mEyePosition, mTargetPosition, mUpDirection);
}
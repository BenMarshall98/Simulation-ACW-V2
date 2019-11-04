#include "Quaternion.h"
#include <cmath>
#include <algorithm>

Quaternion::Quaternion()
{
}

Quaternion::Quaternion(float pX, float pY, float pZ, float pW) :
	mX(pX), mY(pY), mZ(pY), mW(pW)
{
}

Quaternion::~Quaternion()
{
}

void Quaternion::set(float pX, float pY, float pZ, float pW)
{
	mX = pX;
	mY = pY;
	mZ = pZ;
	mW = pW;
}


float Quaternion::getX() const
{
	return mX;
}

float Quaternion::getY() const
{
	return mY;
}

float Quaternion::getW() const
{
	return mW;
}

float Quaternion::getZ() const
{
	return mZ;
}

//http://number-none.com/product/Understanding%20Slerp,%20Then%20Not%20Using%20It/

Quaternion Quaternion::slerp(const Quaternion& pQuaternion, const float pN) const
{
	Quaternion quat1 = normalise();
	Quaternion quat2 = pQuaternion.normalise();

	float dot = quat1.dot(quat2);

	if (dot > 0.9995)
	{
		Quaternion result = quat1 + pN * (quat2 - quat1);
		result = result.normalise();
		return result;
	}

	dot = std::clamp(dot, -1.0f, 1.0f);

	float theta = acos(dot);
	theta *= pN;

	Quaternion quat3 = quat2 - quat1 * dot;
	quat3 = quat3.normalise();

	return quat1 * cos(theta) + quat3 * sin(theta);
}

Quaternion Quaternion::normalise() const
{
	const auto mag = sqrt(mX * mX + mY * mY + mZ * mZ + mW * mW);
	return Quaternion(mX / mag, mY / mag, mZ / mag, mW / mag);
}

float Quaternion::dot(const Quaternion & pQuaternion) const
{
	return mW * pQuaternion.mW + mX * pQuaternion.mX + mY * pQuaternion.mY + mZ * pQuaternion.mZ;
}

Quaternion Quaternion::add(const Quaternion& pQuaternion) const
{
	return {
		mX + pQuaternion.mX,
		mY + pQuaternion.mY,
		mZ + pQuaternion.mZ,
		mW + pQuaternion.mW
	};
}

Quaternion Quaternion::subtract(const Quaternion& pQuaternion) const
{
	return {
		mX - pQuaternion.mX,
		mY - pQuaternion.mY,
		mZ - pQuaternion.mZ,
		mW - pQuaternion.mW
	};
}

Quaternion Quaternion::mult(float pN) const
{
	return {
		mX * pN,
		mY * pN,
		mZ * pN,
		mW * pN
	};
}

Quaternion operator*(const Quaternion& pLhs, float pN)
{
	return pLhs.mult(pN);
}

class Quaternion operator*(float pN, const class Quaternion& pLhs)
{
	return pLhs.mult(pN);
}


Quaternion operator+(const Quaternion& pLhs, const Quaternion& pRhs)
{
	return pLhs.add(pRhs);
}

Quaternion operator-(const Quaternion& pLhs, const Quaternion& pRhs)
{
	return pLhs.subtract(pRhs);
}

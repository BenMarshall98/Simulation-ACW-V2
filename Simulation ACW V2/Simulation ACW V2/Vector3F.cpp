#include "Vector3f.h"
#define _USE_MATH_DEFINES
#include <cmath>

Vector3F::Vector3F() : mX(0), mY(0), mZ(0)
{
}

Vector3F::Vector3F(const float pX, const float pY, const float pZ) : mX(pX), mY(pY), mZ(pZ)
{
}

void Vector3F::set(const float pX, const float pY, const float pZ)
{
	mX = pX;
	mY = pY;
	mZ = pZ;
}

float Vector3F::getX() const
{
	
	return mX;
}

float Vector3F::getY() const
{
	return mY;
}

float Vector3F::getZ() const
{
	return mZ;
}

Vector3F Vector3F::add(const Vector3F &pVec) const
{
	return Vector3F(mX + pVec.mX, mY + pVec.mY, mZ + pVec.mZ);
}

Vector3F Vector3F::subtract(const Vector3F &pVec) const
{
	return Vector3F(mX - pVec.mX, mY - pVec.mY, mZ - pVec.mZ);
}

Vector3F Vector3F::multiply(const float pN) const
{
	return Vector3F(mX * pN, mY * pN, mZ * pN);
}

Vector3F Vector3F::divide(const float pN) const
{
	return Vector3F(mX / pN, mY / pN, mZ / pN);
}

Vector3F Vector3F::cross(const Vector3F& pVec) const
{
	return Vector3F(
		mY * pVec.mZ - mZ * pVec.mY,
		mZ * pVec.mX - mX * pVec.mZ,
		mX * pVec.mY - mY * pVec.mX
	);
}

float Vector3F::dot(const Vector3F &pVec) const
{
	return mX * pVec.mX + mY * pVec.mY + mZ * pVec.mZ;
}

float Vector3F::length() const
{
	return sqrt(mX * mX + mY * mY + mZ * mZ);
}

float Vector3F::distance(const Vector3F &pVec) const
{
	return subtract(pVec).length();
}

Vector3F Vector3F::normalize() const
{
	const auto len = length();
	return Vector3F(mX / len, mY / len, mZ / len);
}

Vector3F Vector3F::interpolate(const Vector3F& pVec, const float pN) const
{
	return *this + pN * (pVec - *this);
}

Vector3F operator+ (const Vector3F &pLhs, const Vector3F &pRhs)
{
	return pLhs.add(pRhs);
}

Vector3F operator- (const Vector3F &pLhs, const Vector3F &pRhs)
{
	return pLhs.subtract(pRhs);
}

Vector3F operator* (const Vector3F & pLhs, const float pN)
{
	return pLhs.multiply(pN);
}

Vector3F operator* (const float pN, const Vector3F & pLhs)
{
	return pLhs.multiply(pN);
}

Vector3F operator/ (const Vector3F & pLhs, const float pN)
{
	return pLhs.divide(pN);
}

bool operator==(const Vector3F& pLhs, const Vector3F& pRhs)
{
	return pLhs.getX() == pRhs.getX() &&
		pLhs.getY() == pRhs.getY() &&
		pLhs.getZ() == pRhs.getZ();
}
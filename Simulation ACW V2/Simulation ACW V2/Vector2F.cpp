#include "Vector2f.h"
#define _USE_MATH_DEFINES
#include <cmath>

Vector2F::Vector2F() : mX(0), mY(0)
{
}

Vector2F::Vector2F(const float pX, const float pY) : mX(pX), mY(pY)
{
}

void Vector2F::set(const float pX, const float pY)
{
	mX = pX;
	mY = pY;
}

float Vector2F::getX() const
{
	return mX;
}

float Vector2F::getY() const
{
	return mY;
}

Vector2F Vector2F::add(const Vector2F &pVec) const
{
	return Vector2F(mX + pVec.getX(), mY + pVec.getY());
}

Vector2F Vector2F::subtract(const Vector2F &pVec) const
{
	return Vector2F(mX - pVec.getX(), mY - pVec.getY());
}

Vector2F Vector2F::mult(const float pN) const
{
	return Vector2F(mX * pN, mY * pN);
}

Vector2F Vector2F::divide(const float pN) const
{
	return Vector2F(mX / pN, mY / pN);
}

float Vector2F::dot(const Vector2F &pVec) const
{
	return mX * pVec.getX() + mY * pVec.getY();
}

float Vector2F::length() const
{
	return sqrt(mX * mX + mY * mY);
}

float Vector2F::distance(const Vector2F &pVec) const
{
	return subtract(pVec).length();
}

Vector2F Vector2F::normalise()
{
	const auto len = length();
	set(mX / len, mY / len);
	return *this;
}

Vector2F operator+ (const Vector2F &pLhs, const Vector2F &pRhs)
{
	return pLhs.add(pRhs);
}

Vector2F operator- (const Vector2F &pLhs, const Vector2F &pRhs)
{
	return pLhs.subtract(pRhs);
}

Vector2F operator* (const Vector2F &pLhs, const float pN)
{
	return pLhs.mult(pN);
}

Vector2F operator* (const float pN, const Vector2F &pLhs)
{
	return pLhs.mult(pN);
}

Vector2F operator/ (const Vector2F &pLhs, const float pN)
{
	return pLhs.divide(pN);
}

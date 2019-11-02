#include "Matrix3f.h"
#include "gl.h"
#define _USE_MATH_DEFINES
#include <cmath>


Matrix3F::Matrix3F() : mMatrix{ {1, 0, 0}, {0, 1, 0}, {0, 0, 1} }
{
}

Matrix3F::Matrix3F(const float p11, const float p12, const float p13,
	const float p21, const float p22, const float p23,
	const float p31, const float p32, const float p33) :
	mMatrix{ {p11, p12, p13}, {p21, p22, p23}, {p31, p32, p33} }
{
}


Matrix3F Matrix3F::createIdentity()
{
	return {};
}

Matrix3F Matrix3F::createRotation(const Vector3F& pVec, const float pN)
{
	const auto x = pVec.getX();
	const auto y = pVec.getY();
	const auto z = pVec.getZ();

	const auto x2 = x * x;
	const auto y2 = y * y;
	const auto z2 = z * z;

	const auto c = cos(pN * static_cast<float>(M_PI) / 180.0f);
	const auto s = sin(pN * static_cast<float>(M_PI) / 180.0f);

	return {
		c + x2 * (1 - c), x * y * (1 - c) - s * z, x * z * (1 - c) + s * y,
		x * y * (1 - c) + s * z, c + y2 * (1 - c), y * z * (1 - c) - s * x,
		x * z * (1 - c) - s * y, y * z * (1 - c) + s * x, c + z2 * (1 - c)
	};
}

Matrix3F Matrix3F::createSkew(const Vector3F& pVec)
{
	const auto x = pVec.getX();
	const auto y = pVec.getY();
	const auto z = pVec.getZ();

	return {
		0, -z, y,
		z, 0, -x,
		-y, 0, x
	};
}


float Matrix3F::get(const int pX, const int pY) const
{
	return mMatrix[pX][pY];
}

Matrix3F Matrix3F::add(const Matrix3F& pMat) const
{
	return {
		mMatrix[0][0] + pMat.mMatrix[0][0], mMatrix[0][1] + pMat.mMatrix[0][1], mMatrix[0][2] + pMat.mMatrix[0][2], 
		mMatrix[1][0] + pMat.mMatrix[1][0], mMatrix[1][1] + pMat.mMatrix[1][1], mMatrix[1][2] + pMat.mMatrix[1][2], 
		mMatrix[2][0] + pMat.mMatrix[2][0], mMatrix[2][1] + pMat.mMatrix[2][1], mMatrix[2][2] + pMat.mMatrix[2][2]
	};
}

Matrix3F Matrix3F::subtract(const Matrix3F& pMat) const
{
	return {
		mMatrix[0][0] - pMat.mMatrix[0][0], mMatrix[0][1] - pMat.mMatrix[0][1], mMatrix[0][2] - pMat.mMatrix[0][2],
		mMatrix[1][0] - pMat.mMatrix[1][0], mMatrix[1][1] - pMat.mMatrix[1][1], mMatrix[1][2] - pMat.mMatrix[1][2],
		mMatrix[2][0] - pMat.mMatrix[2][0], mMatrix[2][1] - pMat.mMatrix[2][1], mMatrix[2][2] - pMat.mMatrix[2][2]
	};
}

Matrix3F Matrix3F::mult(const float pN) const
{
	return {
		mMatrix[0][0] * pN, mMatrix[0][1] * pN, mMatrix[0][2] * pN,
		mMatrix[1][0] * pN, mMatrix[1][1] * pN, mMatrix[1][2] * pN,
		mMatrix[2][0] * pN, mMatrix[2][1] * pN, mMatrix[2][2] * pN
	};
}

Vector3F Matrix3F::mult(const Vector3F& pVec) const
{
	return {
		mMatrix[0][0] * pVec.getX() + mMatrix[0][1] * pVec.getY() + mMatrix[0][2] * pVec.getZ(),
		mMatrix[1][0] * pVec.getX() + mMatrix[1][1] * pVec.getY() + mMatrix[1][2] * pVec.getZ(),
		mMatrix[2][0] * pVec.getX() + mMatrix[2][1] * pVec.getY() + mMatrix[2][2] * pVec.getZ()
	};
}

Matrix3F Matrix3F::mult(const Matrix3F& pMat) const
{
	return {
		mMatrix[0][0] * pMat.mMatrix[0][0] + mMatrix[0][1] * pMat.mMatrix[1][0] + mMatrix[0][2] * pMat.mMatrix[2][0],
		mMatrix[0][0] * pMat.mMatrix[0][1] + mMatrix[0][1] * pMat.mMatrix[1][1] + mMatrix[0][2] * pMat.mMatrix[2][1],
		mMatrix[0][0] * pMat.mMatrix[0][2] + mMatrix[0][1] * pMat.mMatrix[1][2] + mMatrix[0][2] * pMat.mMatrix[2][2],

		mMatrix[1][0] * pMat.mMatrix[0][0] + mMatrix[1][1] * pMat.mMatrix[1][0] + mMatrix[1][2] * pMat.mMatrix[2][0],
		mMatrix[1][0] * pMat.mMatrix[0][1] + mMatrix[1][1] * pMat.mMatrix[1][1] + mMatrix[1][2] * pMat.mMatrix[2][1],
		mMatrix[1][0] * pMat.mMatrix[0][2] + mMatrix[1][1] * pMat.mMatrix[1][2] + mMatrix[1][2] * pMat.mMatrix[2][2],

		mMatrix[2][0] * pMat.mMatrix[0][0] + mMatrix[2][1] * pMat.mMatrix[1][0] + mMatrix[2][2] * pMat.mMatrix[2][0],
		mMatrix[2][0] * pMat.mMatrix[0][1] + mMatrix[2][1] * pMat.mMatrix[1][1] + mMatrix[2][2] * pMat.mMatrix[2][1],
		mMatrix[2][0] * pMat.mMatrix[0][2] + mMatrix[2][1] * pMat.mMatrix[1][2] + mMatrix[2][2] * pMat.mMatrix[2][2]
	};
}

Matrix3F Matrix3F::divide(const float pN) const
{
	return {
		mMatrix[0][0] / pN, mMatrix[0][1] / pN, mMatrix[0][2] / pN,
		mMatrix[1][0] / pN, mMatrix[1][1] / pN, mMatrix[1][2] / pN,
		mMatrix[2][0] / pN, mMatrix[2][1] / pN, mMatrix[2][2] / pN
	};
}

Matrix3F Matrix3F::transpose() const
{
	return {
		mMatrix[0][0], mMatrix[1][0], mMatrix[2][0],
		mMatrix[0][1], mMatrix[1][1], mMatrix[2][1],
		mMatrix[0][2], mMatrix[1][2], mMatrix[2][2]
	};
}

Matrix3F Matrix3F::inverse() const
{
	const auto a = mMatrix[1][1] * mMatrix[2][2] - mMatrix[1][2] * mMatrix[2][1];
	const auto b = mMatrix[1][0] * mMatrix[2][2] - mMatrix[1][2] * mMatrix[2][0];
	const auto c = mMatrix[1][0] * mMatrix[2][1] - mMatrix[1][1] * mMatrix[2][0];
	const auto d = mMatrix[0][1] * mMatrix[2][2] - mMatrix[0][2] * mMatrix[2][1];
	const auto e = mMatrix[0][0] * mMatrix[2][2] - mMatrix[0][2] * mMatrix[2][0];
	const auto f = mMatrix[0][0] * mMatrix[2][1] - mMatrix[1][1] * mMatrix[2][0];
	const auto g = mMatrix[0][1] * mMatrix[1][2] - mMatrix[0][2] * mMatrix[1][1];
	const auto h = mMatrix[0][0] * mMatrix[1][2] - mMatrix[0][2] * mMatrix[1][0];
	const auto i = mMatrix[0][0] * mMatrix[1][1] - mMatrix[0][1] * mMatrix[1][0];

	Matrix3F adjugate(a, -d, g, -b, e, -h, c, -f, i);

	const auto determ = mMatrix[0][0] * a + mMatrix[0][1] * b + mMatrix[0][2] * c;

	return adjugate * (1.0f / determ);
}

Matrix3F Matrix3F::normaliseColumns() const
{
	const auto c1 = sqrt(mMatrix[0][0] * mMatrix[0][0] + mMatrix[1][0] * mMatrix[1][0] + mMatrix[2][0] * mMatrix[2][0]);
	const auto c2 = sqrt(mMatrix[0][1] * mMatrix[0][1] + mMatrix[1][1] * mMatrix[1][1] + mMatrix[2][1] * mMatrix[2][1]);
	const auto c3 = sqrt(mMatrix[0][2] * mMatrix[0][2] + mMatrix[1][2] * mMatrix[1][2] + mMatrix[2][2] * mMatrix[2][2]);

	return {
		mMatrix[0][0] / c1, mMatrix[0][1] / c2, mMatrix[0][2] / c3,
		mMatrix[1][0] / c1, mMatrix[1][1] / c2, mMatrix[1][2] / c3,
		mMatrix[2][0] / c1, mMatrix[2][1] / c2, mMatrix[2][2] / c3
	};
}

float* Matrix3F::operator[](const int pI)
{
	return mMatrix[pI];
}

Matrix3F operator+(const Matrix3F& pLhs, const Matrix3F& pRhs)
{
	return pLhs.add(pRhs);
}

Matrix3F operator-(const Matrix3F& pLhs, const Matrix3F& pRhs)
{
	return pLhs.subtract(pRhs);
}

Matrix3F operator*(const Matrix3F& pLhs, const float pN)
{
	return pLhs.mult(pN);
}

Matrix3F operator*(const float pN, const Matrix3F& pLhs)
{
	return pLhs.mult(pN);
}

Vector3F operator*(const Matrix3F& pLhs, const Vector3F& pRhs)
{
	return pLhs.mult(pRhs);
}

Vector3F operator*(const Vector3F& pLhs, const Matrix3F & pRhs)
{
	return pRhs.mult(pLhs);
}

Matrix3F operator*(const Matrix3F& pLhs, const Matrix3F& pRhs)
{
	return pLhs.mult(pRhs);
}

Matrix3F operator/(const Matrix3F& pLhs, const float pN)
{
	return pLhs.divide(pN);
}
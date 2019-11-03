#include "Matrix4f.h"
#include "gl.h"
#define _USE_MATH_DEFINES
#include <cmath>


Matrix4F::Matrix4F() : mMatrix{ {1, 0, 0, 0}, {0, 1, 0, 0}, {0, 0, 1, 0}, {0, 0, 0, 1} }
{
}

Matrix4F::Matrix4F(const float p11, const float p12, const float p13, const float p14,
	const float p21, const float p22, const float p23, const float p24,
	const float p31, const float p32, const float p33, const float p34,
	const float p41, const float p42, const float p43, const float p44) :
	mMatrix{ {p11, p12, p13, p14}, {p21, p22, p23, p24}, {p31, p32, p33, p34}, {p41, p42, p43, p44} }
{
}

Matrix4F::Matrix4F(const Matrix3F& pMatrix) :
	mMatrix{ {pMatrix.get(0, 0), pMatrix.get(0, 1), pMatrix.get(0, 2), 0},
			{pMatrix.get(1, 0), pMatrix.get(1, 1), pMatrix.get(1, 2), 0},
			{pMatrix.get(2, 0), pMatrix.get(2, 1), pMatrix.get(2, 2), 0},
			{0, 0, 0, 1} }
{
}


Matrix4F Matrix4F::createIdentity()
{
	return {};
}

Matrix4F Matrix4F::createTranslation(const Vector3F& pVec)
{
	return {
		1, 0, 0, pVec.getX(),
		0, 1, 0, pVec.getY(),
		0, 0, 1, pVec.getZ(),
		0, 0, 0, 1
	};
}

Matrix4F Matrix4F::createScale(const Vector3F& pVec)
{
	return {
		pVec.getX(), 0, 0, 0,
		0, pVec.getY(), 0, 0,
		0, 0, pVec.getZ(), 0,
		0, 0, 0, 1
	};
}

Matrix4F Matrix4F::createRotation(const Vector3F& pVec, const float pN)
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
		c + x2 * (1 - c), x * y * (1 - c) - s * z, x * z * (1 - c) + s * y, 0,
		x * y * (1 - c) + s * z, c + y2 * (1 - c), y * z * (1 - c) - s * x, 0,
		x * z * (1 - c) - s * y, y * z * (1 - c) + s * x, c + z2 * (1 - c), 0,
		0, 0, 0, 1
	};
}

//https://www.scratchapixel.com/lessons/mathematics-physics-for-computer-graphics/lookat-function

Matrix4F Matrix4F::createLookAt(const Vector3F& pEye, const Vector3F& pFocus, const Vector3F& pUp)
{
	auto zaxis = pEye - pFocus;
	zaxis = zaxis.normalise();
	auto yaxis = pUp;
	auto xaxis = yaxis.cross(zaxis);
	yaxis = zaxis.cross(xaxis);
	xaxis = xaxis.normalise();
	yaxis = yaxis.normalise();

	/*return {
		xaxis.getX(), yaxis.getX(), zaxis.getX(), 0,
		xaxis.getY(), yaxis.getY(), zaxis.getY(), 0,
		xaxis.getZ(), yaxis.getZ(), zaxis.getZ(), 0,
		-xaxis.dot(pEye), -yaxis.dot(pEye), -zaxis.dot(pEye), 1
	};*/

	return {
		xaxis.getX(), xaxis.getY(), xaxis.getZ(), -xaxis.dot(pEye),
		yaxis.getX(), yaxis.getY(), yaxis.getZ(), -yaxis.dot(pEye),
		zaxis.getX(), zaxis.getY(), zaxis.getZ(), -zaxis.dot(pEye),
		0, 0, 0, 1
	};
}

Matrix4F Matrix4F::createPerspective(const float pFov, const float pAspect, const float pNear, const float pFar)
{
	const auto f = 1.0f / tan(pFov * static_cast<float>(M_PI) / 180.0f / 2.0f);

	return {
		f / pAspect, 0, 0, 0,
		0, f, 0, 0,
		0, 0, pFar / (pNear - pFar), pNear * pFar / (pNear - pFar),
		0, 0, -1, 0
	};
}

float Matrix4F::get(const int pX, const int pY)
{
	return mMatrix[pX][pY];
}

Matrix4F Matrix4F::add(const Matrix4F& pMat) const
{
	return {
		mMatrix[0][0] + pMat.mMatrix[0][0], mMatrix[0][1] + pMat.mMatrix[0][1], mMatrix[0][2] + pMat.mMatrix[0][2], mMatrix[0][3] + pMat.mMatrix[0][3],
		mMatrix[1][0] + pMat.mMatrix[1][0], mMatrix[1][1] + pMat.mMatrix[1][1], mMatrix[1][2] + pMat.mMatrix[1][2], mMatrix[1][3] + pMat.mMatrix[1][3],
		mMatrix[2][0] + pMat.mMatrix[2][0], mMatrix[2][1] + pMat.mMatrix[2][1], mMatrix[2][2] + pMat.mMatrix[2][2], mMatrix[2][3] + pMat.mMatrix[2][3],
		mMatrix[3][0] + pMat.mMatrix[3][0], mMatrix[3][1] + pMat.mMatrix[3][1], mMatrix[3][2] + pMat.mMatrix[3][2], mMatrix[3][3] + pMat.mMatrix[3][3]
	};
}

Matrix4F Matrix4F::subtract(const Matrix4F& pMat) const
{
	return {
		mMatrix[0][0] - pMat.mMatrix[0][0], mMatrix[0][1] - pMat.mMatrix[0][1], mMatrix[0][2] - pMat.mMatrix[0][2], mMatrix[0][3] - pMat.mMatrix[0][3],
		mMatrix[1][0] - pMat.mMatrix[1][0], mMatrix[1][1] - pMat.mMatrix[1][1], mMatrix[1][2] - pMat.mMatrix[1][2], mMatrix[1][3] - pMat.mMatrix[1][3],
		mMatrix[2][0] - pMat.mMatrix[2][0], mMatrix[2][1] - pMat.mMatrix[2][1], mMatrix[2][2] - pMat.mMatrix[2][2], mMatrix[2][3] - pMat.mMatrix[2][3],
		mMatrix[3][0] - pMat.mMatrix[3][0], mMatrix[3][1] - pMat.mMatrix[3][1], mMatrix[3][2] - pMat.mMatrix[3][2], mMatrix[3][3] - pMat.mMatrix[3][3]
	};
}

Matrix4F Matrix4F::mult(const float pN) const
{
	return {
		mMatrix[0][0] * pN, mMatrix[0][1] * pN, mMatrix[0][2] * pN, mMatrix[0][3] * pN,
		mMatrix[1][0] * pN, mMatrix[1][1] * pN, mMatrix[1][2] * pN, mMatrix[1][3] * pN,
		mMatrix[2][0] * pN, mMatrix[2][1] * pN, mMatrix[2][2] * pN, mMatrix[2][3] * pN,
		mMatrix[3][0] * pN, mMatrix[3][1] * pN, mMatrix[3][2] * pN, mMatrix[3][3] * pN
	};
}

Vector3F Matrix4F::mult(const Vector3F& pVec) const
{
	return {
		mMatrix[0][0] * pVec.getX() + mMatrix[0][1] * pVec.getY() + mMatrix[0][2] * pVec.getZ() + mMatrix[0][3],
		mMatrix[1][0] * pVec.getX() + mMatrix[1][1] * pVec.getY() + mMatrix[1][2] * pVec.getZ() + mMatrix[1][3],
		mMatrix[2][0] * pVec.getX() + mMatrix[2][1] * pVec.getY() + mMatrix[2][2] * pVec.getZ() + mMatrix[2][3]
	};
}

Matrix4F Matrix4F::mult(const Matrix4F& pMat) const
{
	return {
		mMatrix[0][0] * pMat.mMatrix[0][0] + mMatrix[0][1] * pMat.mMatrix[1][0] + mMatrix[0][2] * pMat.mMatrix[2][0] + mMatrix[0][3] * pMat.mMatrix[3][0],
		mMatrix[0][0] * pMat.mMatrix[0][1] + mMatrix[0][1] * pMat.mMatrix[1][1] + mMatrix[0][2] * pMat.mMatrix[2][1] + mMatrix[0][3] * pMat.mMatrix[3][1],
		mMatrix[0][0] * pMat.mMatrix[0][2] + mMatrix[0][1] * pMat.mMatrix[1][2] + mMatrix[0][2] * pMat.mMatrix[2][2] + mMatrix[0][3] * pMat.mMatrix[3][2],
		mMatrix[0][0] * pMat.mMatrix[0][3] + mMatrix[0][1] * pMat.mMatrix[1][3] + mMatrix[0][2] * pMat.mMatrix[2][3] + mMatrix[0][3] * pMat.mMatrix[3][3],

		mMatrix[1][0] * pMat.mMatrix[0][0] + mMatrix[1][1] * pMat.mMatrix[1][0] + mMatrix[1][2] * pMat.mMatrix[2][0] + mMatrix[1][3] * pMat.mMatrix[3][0],
		mMatrix[1][0] * pMat.mMatrix[0][1] + mMatrix[1][1] * pMat.mMatrix[1][1] + mMatrix[1][2] * pMat.mMatrix[2][1] + mMatrix[1][3] * pMat.mMatrix[3][1],
		mMatrix[1][0] * pMat.mMatrix[0][2] + mMatrix[1][1] * pMat.mMatrix[1][2] + mMatrix[1][2] * pMat.mMatrix[2][2] + mMatrix[1][3] * pMat.mMatrix[3][2],
		mMatrix[1][0] * pMat.mMatrix[0][3] + mMatrix[1][1] * pMat.mMatrix[1][3] + mMatrix[1][2] * pMat.mMatrix[2][3] + mMatrix[1][3] * pMat.mMatrix[3][3],

		mMatrix[2][0] * pMat.mMatrix[0][0] + mMatrix[2][1] * pMat.mMatrix[1][0] + mMatrix[2][2] * pMat.mMatrix[2][0] + mMatrix[2][3] * pMat.mMatrix[3][0],
		mMatrix[2][0] * pMat.mMatrix[0][1] + mMatrix[2][1] * pMat.mMatrix[1][1] + mMatrix[2][2] * pMat.mMatrix[2][1] + mMatrix[2][3] * pMat.mMatrix[3][1],
		mMatrix[2][0] * pMat.mMatrix[0][2] + mMatrix[2][1] * pMat.mMatrix[1][2] + mMatrix[2][2] * pMat.mMatrix[2][2] + mMatrix[2][3] * pMat.mMatrix[3][2],
		mMatrix[2][0] * pMat.mMatrix[0][3] + mMatrix[2][1] * pMat.mMatrix[1][3] + mMatrix[2][2] * pMat.mMatrix[2][3] + mMatrix[2][3] * pMat.mMatrix[3][3],

		mMatrix[3][0] * pMat.mMatrix[0][0] + mMatrix[3][1] * pMat.mMatrix[1][0] + mMatrix[3][2] * pMat.mMatrix[2][0] + mMatrix[3][3] * pMat.mMatrix[3][0],
		mMatrix[3][0] * pMat.mMatrix[0][1] + mMatrix[3][1] * pMat.mMatrix[1][1] + mMatrix[3][2] * pMat.mMatrix[2][1] + mMatrix[3][3] * pMat.mMatrix[3][1],
		mMatrix[3][0] * pMat.mMatrix[0][2] + mMatrix[3][1] * pMat.mMatrix[1][2] + mMatrix[3][2] * pMat.mMatrix[2][2] + mMatrix[3][3] * pMat.mMatrix[3][2],
		mMatrix[3][0] * pMat.mMatrix[0][3] + mMatrix[3][1] * pMat.mMatrix[1][3] + mMatrix[3][2] * pMat.mMatrix[2][3] + mMatrix[3][3] * pMat.mMatrix[3][3]
	};
}

Matrix4F Matrix4F::divide(const float pN) const
{
	return {
		mMatrix[0][0] / pN, mMatrix[0][1] / pN, mMatrix[0][2] / pN, mMatrix[0][3] / pN,
		mMatrix[1][0] / pN, mMatrix[1][1] / pN, mMatrix[1][2] / pN, mMatrix[1][3] / pN,
		mMatrix[2][0] / pN, mMatrix[2][1] / pN, mMatrix[2][2] / pN, mMatrix[2][3] / pN,
		mMatrix[3][0] / pN, mMatrix[3][1] / pN, mMatrix[3][2] / pN, mMatrix[3][3] / pN
	};
}

Matrix4F Matrix4F::transpose() const
{
	return {
		mMatrix[0][0], mMatrix[1][0], mMatrix[2][0], mMatrix[3][0],
		mMatrix[0][1], mMatrix[1][1], mMatrix[2][1], mMatrix[3][1],
		mMatrix[0][2], mMatrix[1][2], mMatrix[2][2], mMatrix[3][2],
		mMatrix[0][3], mMatrix[1][3], mMatrix[2][3], mMatrix[3][3]
	};
}

float* Matrix4F::operator[](const int pI)
{
	return mMatrix[pI];
}

void Matrix4F::useMatrix(const int pLocation)
{
	float result[16];

	auto count = 0;
	for (auto& i : mMatrix)
	{
		for (auto j : i)
		{
			result[count] = j;
			count++;
		}
	}

	glUniformMatrix4fv(pLocation, 1, GL_TRUE, &result[0]);
}


Matrix4F operator+(const Matrix4F& pLhs, const Matrix4F& pRhs)
{
	return pLhs.add(pRhs);
}

Matrix4F operator-(const Matrix4F& pLhs, const Matrix4F& pRhs)
{
	return pLhs.subtract(pRhs);
}

Matrix4F operator*(const Matrix4F& pLhs, const float pN)
{
	return pLhs.mult(pN);
}

Matrix4F operator*(const float pN, const Matrix4F& pLhs)
{
	return pLhs.mult(pN);
}

Vector3F operator*(const Matrix4F& pLhs, const Vector3F& pRhs)
{
	return pLhs.mult(pRhs);
}

Vector3F operator*(const Vector3F& pLhs, const Matrix4F & pRhs)
{
	return pRhs.mult(pLhs);
}

Matrix4F operator*(const Matrix4F& pLhs, const Matrix4F& pRhs)
{
	return pLhs.mult(pRhs);
}

Matrix4F operator/(const Matrix4F& pLhs, const float pN)
{
	return pLhs.divide(pN);
}
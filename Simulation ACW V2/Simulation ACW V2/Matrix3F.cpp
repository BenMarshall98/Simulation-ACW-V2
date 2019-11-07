#include "Matrix3f.h"
#include "gl.h"
#define _USE_MATH_DEFINES
#include <cmath>
#include "glm/glm.hpp"
#include "glm/gtx/matrix_interpolation.hpp"

Matrix3F::Matrix3F() : mMatrix{ {1, 0, 0}, {0, 1, 0}, {0, 0, 1} }
{
}

Matrix3F::Matrix3F(const float p11, const float p12, const float p13,
	const float p21, const float p22, const float p23,
	const float p31, const float p32, const float p33) :
	mMatrix{ {p11, p12, p13}, {p21, p22, p23}, {p31, p32, p33} }
{
	if (isnan(p11))
	{
		int i = 0;
	}
}

Matrix3F::Matrix3F(const Quaternion& pQuaternion)
{
	const auto x = pQuaternion.getX();
	const auto y = pQuaternion.getY();
	const auto z = pQuaternion.getZ();
	const auto w = pQuaternion.getW();
	const auto x2 = x * x;
	const auto y2 = y * y;
	const auto z2 = z * z;
	const auto w2 = w * w;

	mMatrix[0][0] = w2 + x2 - y2 - z2;
	mMatrix[0][1] = 2 * x * y - 2 * w * z;
	mMatrix[0][2] = 2 * x * z + 2 * w * y;
	mMatrix[1][0] = 2 * x * y + 2 * w * z;
	mMatrix[1][1] = w2 - x2 + y2 + z2;
	mMatrix[1][2] = 2 * y * z + 2 * w * x;
	mMatrix[2][0] = 2 * x * z - 2 * w * y;
	mMatrix[2][1] = 2 * y * z - 2 * w * x;
	mMatrix[2][2] = w2 - x2 - y2 + z2;
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

Matrix3F Matrix3F::interpolate(const Matrix3F& pMat, const float pN) const
{
	glm::mat3 mat1 = glm::mat3(
		mMatrix[0][0], mMatrix[0][1], mMatrix[0][2],
		mMatrix[1][0], mMatrix[1][1], mMatrix[1][2],
		mMatrix[2][0], mMatrix[2][1], mMatrix[2][2]
	);

	glm::mat3 mat2 = glm::mat3(
		pMat.mMatrix[0][0], pMat.mMatrix[0][1], pMat.mMatrix[0][2],
		pMat.mMatrix[1][0], pMat.mMatrix[1][1], pMat.mMatrix[1][2],
		pMat.mMatrix[2][0], pMat.mMatrix[2][1], pMat.mMatrix[2][2]
	);

	glm::mat3 mat3 = glm::interpolate(glm::mat4(mat1), glm::mat4(mat2), pN);

	return Matrix3F(mat3[0][0], mat3[0][1], mat3[0][2],
		mat3[1][0], mat3[1][2], mat3[1][2],
		mat3[2][0], mat3[2][2], mat3[2][2]);
	/*const auto quat1 = toQuaternion();
	const auto quat2 = pMat.toQuaternion();
	const auto result = quat1.slerp(quat2, pN);
	return Matrix3F(result);*/
}

//https://www.euclideanspace.com/maths/geometry/rotations/conversions/matrixToQuaternion/

Quaternion Matrix3F::toQuaternion() const
{
	const auto trace = mMatrix[0][0] + mMatrix[1][1] + mMatrix[2][2];

	if (trace > 0.0f)
	{
		const auto s = sqrt(trace + 1.0f) * 2;
		return {
			(mMatrix[2][1] - mMatrix[1][2]) / s,
			(mMatrix[0][2] - mMatrix[2][0]) / s,
			(mMatrix[1][0] - mMatrix[0][1]) / s,
			0.25f * s
		};
	}
	if (mMatrix[0][0] > mMatrix[1][1] && mMatrix[0][0] > mMatrix[2][2])
	{
		const auto s = sqrt(1.0f + mMatrix[0][0] - mMatrix[1][1] - mMatrix[2][2]) * 2;
		return {
			0.25f * s,
			(mMatrix[0][1] + mMatrix[1][0]) / s,
			(mMatrix[0][2] + mMatrix[2][0]) / s,
			(mMatrix[2][1] - mMatrix[1][2]) / s
		};
	}
	if (mMatrix[1][1] > mMatrix[2][2])
	{
		const auto s = sqrt(1.0f + mMatrix[1][1] - mMatrix[0][0] - mMatrix[2][2]) * 2;
		return {
			(mMatrix[0][1] + mMatrix[1][0]) / s,
			0.25f * s,
			(mMatrix[1][2] + mMatrix[2][1]) / s,
			(mMatrix[1][0] - mMatrix[0][1]) / s
		};
	}
	const auto s = sqrt(1.0f + mMatrix[2][2] - mMatrix[0][0] - mMatrix[1][1]) * 2;
	return {
		(mMatrix[0][2] + mMatrix[2][0]) / s,
		(mMatrix[1][2] + mMatrix[2][1]) / s,
		0.25f * s,
		(mMatrix[1][0] + mMatrix[0][1]) / s
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
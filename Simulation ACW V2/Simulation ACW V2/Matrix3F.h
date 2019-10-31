#pragma once

#include "Vector3F.h"

class Matrix3F
{
public:
	Matrix3F();
	Matrix3F(float p11, float p12, float p13,
		float p21, float p22, float p23,
		float p31, float p32, float p33);

	static Matrix3F createIdentity();
	static Matrix3F createRotation(const Vector3F &pVec, float pN);
	static Matrix3F createSkew(const Vector3F &pVec);

	float get(int pX, int pY);

	Matrix3F add(const Matrix3F &pMat) const;
	Matrix3F subtract(const Matrix3F &pMat) const;
	Matrix3F mult(float pN) const;
	Vector3F mult(const Vector3F &pVec) const;
	Matrix3F mult(const Matrix3F &pMat) const;
	Matrix3F divide(float pN) const;
	Matrix3F transpose() const;
	Matrix3F inverse() const;

	float* operator[] (int pI);

private:
	float mMatrix[3][3];
};

Matrix3F operator+ (const Matrix3F &pLhs, const Matrix3F &pRhs);
Matrix3F operator- (const Matrix3F &pLhs, const Matrix3F &pRhs);
Matrix3F operator* (const Matrix3F &pLhs, float pN);
Matrix3F operator* (float pN, const Matrix3F &pLhs);
Vector3F operator* (const Matrix3F &pLhs, const Vector3F & pRhs);
Vector3F operator* (const Vector3F &pLhs, const Matrix3F & pRhs);
Matrix3F operator* (const Matrix3F &pLhs, const Matrix3F &pRhs);
Matrix3F operator/ (const Matrix3F &pLhs, float pN);


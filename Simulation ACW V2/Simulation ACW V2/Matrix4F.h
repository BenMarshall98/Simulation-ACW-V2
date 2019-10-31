#pragma once

#include "Vector3F.h"
#include "Matrix3F.h"

class Matrix4F
{
public:
	Matrix4F();
	Matrix4F(float p11, float p12, float p13, float p14,
		float p21, float p22, float p23, float p24,
		float p31, float p32, float p33, float p34,
		float p41, float p42, float p43, float p44);
	Matrix4F(const Matrix3F & pMatrix);

	static Matrix4F createIdentity();
	static Matrix4F createTranslation(const Vector3F &pVec);
	static Matrix4F createScale(const Vector3F &pVec);
	static Matrix4F createRotation(const Vector3F &pVec, float pN);
	static Matrix4F createLookAt(const Vector3F &pEye, const Vector3F &pFocus, const Vector3F &pUp);
	static Matrix4F createPerspective(float pFov, float pAspect, float pNear, float pFar);

	float get(int pX, int pY);

	Matrix4F add(const Matrix4F &pMat) const;
	Matrix4F subtract(const Matrix4F &pMat) const;
	Matrix4F mult(float pN) const;
	Vector3F mult(const Vector3F &pVec) const;
	Matrix4F mult(const Matrix4F &pMat) const;
	Matrix4F divide(float pN) const;
	Matrix4F transpose() const;

	float* operator[] (int pI);
	void useMatrix(int pLocation);

private:
	float mMatrix[4][4];
};

Matrix4F operator+ (const Matrix4F &pLhs, const Matrix4F &pRhs);
Matrix4F operator- (const Matrix4F &pLhs, const Matrix4F &pRhs);
Matrix4F operator* (const Matrix4F &pLhs, float pN);
Matrix4F operator* (float pN, const Matrix4F &pLhs);
Vector3F operator* (const Matrix4F &pLhs, const Vector3F & pRhs);
Vector3F operator* (const Vector3F &pLhs, const Matrix4F & pRhs);
Matrix4F operator* (const Matrix4F &pLhs, const Matrix4F &pRhs);
Matrix4F operator/ (const Matrix4F &pLhs, float pN);


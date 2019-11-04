#pragma once
class Vector3F
{
public:
	Vector3F();
	Vector3F(float pX, float pY, float pZ);

	void set(float pX, float pY, float pZ);
	float getX() const;
	float getY() const;
	float getZ() const;

	Vector3F add(const Vector3F &pVec) const;
	Vector3F subtract(const Vector3F &pVec) const;
	Vector3F mult(float pN) const;
	Vector3F divide(float pN) const;
	Vector3F cross(const Vector3F &pVec) const;
	float dot(const Vector3F &pVec) const;
	float length() const;
	float distance(const Vector3F &pVec) const;
	Vector3F normalize() const;
	Vector3F interpolate(const Vector3F &pVec, float pN) const;

private:
	float mX, mY, mZ;
};

Vector3F operator+ (const Vector3F &pLhs, const Vector3F &pRhs);
Vector3F operator- (const Vector3F &pLhs, const Vector3F &pRhs);
Vector3F operator* (const Vector3F &pLhs, float pN);
Vector3F operator* (float pN, const Vector3F &pLhs);
Vector3F operator/ (const Vector3F &pLhs, float pN);
bool operator== (const Vector3F &pLhs, const Vector3F &pRhs);


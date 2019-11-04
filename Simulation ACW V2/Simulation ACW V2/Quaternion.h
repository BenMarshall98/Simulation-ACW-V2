#pragma once
class Quaternion
{
public:
	Quaternion();
	Quaternion(float pX, float pY, float pZ, float pW);
	~Quaternion();

	void set(float pX, float pY, float pZ, float pW);

	float getX() const;
	float getY() const;
	float getZ() const;
	float getW() const;

	Quaternion slerp(const Quaternion & pQuaternion, const float pN) const;
	Quaternion normalise() const;
	float dot(const Quaternion & pQuaternion) const;
	Quaternion add(const Quaternion & pQuaternion) const;
	Quaternion subtract(const Quaternion & pQuaternion) const;
	Quaternion mult(float pN) const;

private:
	float mX, mY, mZ, mW;
};

Quaternion operator+ (const Quaternion & pLhs, const Quaternion & pRhs);
Quaternion operator- (const Quaternion & pLhs, const Quaternion & pRhs);
Quaternion operator* (const Quaternion & pLhs, float pN);
Quaternion operator* (float pN, const Quaternion & pLhs);
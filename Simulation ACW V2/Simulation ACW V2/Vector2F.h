#pragma once
class Vector2F
{
public:
	Vector2F();
	Vector2F(float pX, float pY);

	void set(float pX, float pY);
	float getX() const;
	float getY() const;

	Vector2F add(const Vector2F &pVec) const;
	Vector2F subtract(const Vector2F &pVec) const;
	Vector2F multiply(float pN) const;
	Vector2F divide(float pN) const;
	float dot(const Vector2F &pVec) const;
	float length() const;
	float distance(const Vector2F &pVec) const;
	Vector2F normalize();

private:
	float mX, mY;
};

Vector2F operator+ (const Vector2F &pLhs, const Vector2F &pRhs);
Vector2F operator- (const Vector2F &pLhs, const Vector2F &pRhs);
Vector2F operator* (const Vector2F &pLhs, float pN);
Vector2F operator* (float pN, const Vector2F &pLhs);
Vector2F operator/ (const Vector2F &pLhs, float pN);


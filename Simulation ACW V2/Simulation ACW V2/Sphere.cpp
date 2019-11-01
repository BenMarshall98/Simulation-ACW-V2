#include "Sphere.h"
#include <Windows.h>
#include "gl.h"
#define _USE_MATH_DEFINES
#include "TextureLoader.h"
#include "Model.h"
#include "Matrix4f.h"

Sphere::Sphere(const float pRadius, const float pMass, const Vector3F pPos, const Vector3F pAngularVelocity, const Vector3F pVelocity) :
	RigidBody(Vector3F(pRadius, pRadius, pRadius), pMass, pPos, pAngularVelocity, pVelocity, ObjectType::SPHERE)
{
	mTexture = TextureLoader::loadBmp("checker.bmp");
}

Sphere::~Sphere()
{
	glDeleteTextures(1, &mTexture);
}

void Sphere::render(Shader * pShader) const
{
	const auto translation = Matrix4F::createTranslation(mRenderPos);
	const auto scale = Matrix4F::createScale(mSize);
	const auto rotation = Matrix4F(mRenderRotation);

	auto modelMat = translation * scale * rotation;

	const auto modelLocation = glGetUniformLocation(pShader->getShaderId(), "model");
	modelMat.useMatrix(modelLocation);

	glActiveTexture(GL_TEXTURE0);
	glBindTexture(GL_TEXTURE_2D, mTexture);

	const auto textureLocation = glGetUniformLocation(pShader->getShaderId(), "texture");
	glUniform1i(textureLocation, 0);

	auto model = Model::createSphere();
	model->render();
}
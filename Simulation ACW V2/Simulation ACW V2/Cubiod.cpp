#include "Cubiod.h"
#include <Windows.h>
#include "gl.h"
#define _USE_MATH_DEFINES
#include "TextureLoader.h"
#include "Model.h"
#inlcude "Matrix4f.h"

Cuboid::Cuboid(const Vector3F pSize, const float pMass, const Vector3F pPos, const Vector3F pAngularVelocity, const Vector3F pVelocity) :
	RigidBody(pSize, pMass, pPos, pAngularVelocity, pVelocity, ObjectType::CUBIOD,
	Matrix3F(0, 0, 0,
	0, 0, 0,
	0, 0, 0))
{
	mTexture = TextureLoader::loadBmp("checker.bmp");
}

Cubiod::~Cubiod()
{
	glDeleteTextures(1, &mTexture);
}

void Cubiod::render(Shader * pShader) const
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
	
	auto model = Model::createCube();
	model->render();
}
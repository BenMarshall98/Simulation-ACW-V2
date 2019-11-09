#include "Cuboid.h"
#include <Windows.h>
#include "gl.h"
#define _USE_MATH_DEFINES
#include "TextureLoader.h"
#include "Model.h"
#include "Matrix4F.h"

Cuboid::Cuboid(const Vector3F pSize, const float pMass, const Vector3F pPos, const Vector3F pAngularVelocity, const Vector3F pVelocity) :
	RigidBody(pSize, pMass, pPos, pAngularVelocity, pVelocity, ObjectType::CUBOID,
	Matrix3F(0, 0, 0,
	0, 0, 0,
	0, 0, 0))
{
	mTexture = TextureLoader::loadBmp("checker.bmp");
}

Cuboid::~Cuboid()
{
	glDeleteTextures(1, &mTexture);
}

void Cuboid::render(Shader * pShader) const
{
	const auto translation = Matrix4F::createTranslation(mRenderPos);
	const auto scale = Matrix4F::createScale(mSize);
	const auto rot = glm::toMat4(mRenderRotation);
	const auto rotation = Matrix4F(rot[0][0], rot[0][1], rot[0][2], rot[0][3],
		rot[1][0], rot[1][1], rot[1][2], rot[1][3],
		rot[2][0], rot[2][1], rot[2][2], rot[2][3],
		rot[3][0], rot[3][1], rot[3][2], rot[3][3]);
	
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
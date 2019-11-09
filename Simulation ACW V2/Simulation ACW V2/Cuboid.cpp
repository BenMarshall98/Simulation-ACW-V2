#include "Cuboid.h"
#include <Windows.h>
#include "gl.h"
#define _USE_MATH_DEFINES
#include "TextureLoader.h"
#include "Model.h"

Cuboid::Cuboid(const glm::vec3 pSize, const float pMass, const glm::vec3 pPos, const glm::vec3 pAngularVelocity, const glm::vec3 pVelocity) :
	RigidBody(pSize, pMass, pPos, pAngularVelocity, pVelocity, ObjectType::CUBOID,
	glm::mat3(0, 0, 0,
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
	const auto translation = translate(glm::mat4(1.0f), mRenderPos);
	const auto scale = glm::scale(glm::mat4(1.0f), mSize);
	const auto rotation = toMat4(mRenderRotation);
	
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
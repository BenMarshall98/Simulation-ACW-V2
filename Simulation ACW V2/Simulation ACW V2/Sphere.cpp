#include "Sphere.h"
#include <Windows.h>
#include "gl.h"
#define _USE_MATH_DEFINES
#include "TextureLoader.h"
#include "Model.h"

Sphere::Sphere(const float pRadius, const float pMass, const glm::vec3 pPos, const glm::vec3 pAngularVelocity, const glm::vec3 pVelocity) :
	RigidBody(glm::vec3(pRadius, pRadius, pRadius), pMass, pPos,
		pAngularVelocity, pVelocity, ObjectType::SPHERE,
		glm::mat3(2.0f/5.0f * pMass * pRadius * pRadius, 0.0f, 0.0f,
		0.0f, 2.0f/5.0f * pMass * pRadius * pRadius, 0.0f,
		0.0f, 0.0f, 2.0f / 5.0f * pMass * pRadius * pRadius))
{
	mTexture = TextureLoader::loadBmp("checker.bmp");
}

Sphere::~Sphere()
{
	glDeleteTextures(1, &mTexture);
}

void Sphere::render(Shader * pShader) const
{
	const auto translation = glm::mat4::createTranslation(mRenderPos);
	const auto scale = glm::mat4::createScale(mSize);
	const auto rotation = glm::toMat4(mRenderRotation);

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
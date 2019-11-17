#include "Sphere.h"
#include <Windows.h>
#include "gl.h"
#define _USE_MATH_DEFINES
#include "TextureLoader.h"
#include "Model.h"

Sphere::Sphere(const float pRadius, const float pMass, const glm::vec3 pPos, const glm::vec3 pAngularVelocity, const glm::vec3 pVelocity, const glm::vec3 pColour) :
	RigidBody(glm::vec3(pRadius, pRadius, pRadius), pMass, pPos,
		pAngularVelocity, pVelocity, ObjectType::SPHERE,
		glm::mat3(2.0f/5.0f * pMass * pRadius * pRadius, 0.0f, 0.0f,
		0.0f, 2.0f/5.0f * pMass * pRadius * pRadius, 0.0f,
		0.0f, 0.0f, 2.0f / 5.0f * pMass * pRadius * pRadius)), mColour(pColour)
{
}

Sphere::~Sphere()
{
}

void Sphere::render(Shader * pShader) const
{
	static auto mTexture = TextureLoader::loadBmp("checker.bmp");
	const auto translation = translate(glm::mat4(1.0f), mRenderPos);
	const auto scale = glm::scale(glm::mat4(1.0f), mSize);
	const auto rotation = toMat4(mRenderRotation);

	auto modelMat = translation * scale * rotation;

	const auto modelLocation = glGetUniformLocation(pShader->getShaderId(), "model");
	glUniformMatrix4fv(modelLocation, 1, GL_FALSE, &modelMat[0][0]);

	const auto colorLocation = glGetUniformLocation(pShader->getShaderId(), "color");
	glUniform3f(colorLocation, mColour.x, mColour.y, mColour.z);

	glActiveTexture(GL_TEXTURE0);
	glBindTexture(GL_TEXTURE_2D, mTexture);

	const auto textureLocation = glGetUniformLocation(pShader->getShaderId(), "texture");
	glUniform1i(textureLocation, 0);

	auto model = Model::createSphere();
	model->render();
}
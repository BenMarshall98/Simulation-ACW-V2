#include "Bowl.h"
#include "SceneGraphNode.h"

Bowl::Bowl(const glm::vec3 pSize, const float pMass, const glm::vec3 pPos, const glm::vec3 pAngularVelocity, const glm::vec3 pVelocity) :
	RigidBody(pSize, pMass, pPos, pAngularVelocity, pVelocity, ObjectType::BOWL, glm::mat3())
{

}

void Bowl::render(Shader* pShader) const
{
	auto modelMat = glm::mat4();

	if (mParent)
	{
		modelMat = modelMat * mParent->getRenderMatrix();
	}

	const auto translation = translate(glm::mat4(1.0f) ,mRenderPos);
	const auto scale = glm::scale(glm::mat4(1.0f), mSize);
	const auto rotation = toMat4(mRenderRotation);

	modelMat = modelMat * translation * rotation * scale;

	const auto modelLocation = glGetUniformLocation(pShader->getShaderId(), "model");
	glUniformMatrix4fv(modelLocation, 1, GL_FALSE, &modelMat[0][0]);

	auto * model = Model::createBowl();
	model->render();
}


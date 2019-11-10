#include "PlaneHoles.h"
#include "gl.h"
#include "SceneGraphNode.h"

PlaneHoles::PlaneHoles(glm::vec3 pSize, float pMass, glm::vec3 pPos, glm::vec3 pAngularVelocity, glm::vec3 pVelocity) :
	RigidBody(pSize, pMass, pPos, pAngularVelocity, pVelocity, ObjectType::PLANEHOLES, glm::mat3())
{

}

PlaneHoles::~PlaneHoles()
{

}

void PlaneHoles::render(Shader* pShader) const
{
	auto modelMat = glm::mat4(1.0f);

	if (mParent)
	{
		modelMat = modelMat * mParent->getRenderMatrix();
	}

	const auto translation = translate(glm::mat4(1.0f), mRenderPos);
	const auto scale = glm::scale(glm::mat4(1.0f), mSize);
	const auto rotation = toMat4(mRenderRotation);

	modelMat = modelMat * translation * rotation * scale;

	const auto modelLocation = glGetUniformLocation(pShader->getShaderId(), "model");
	glUniformMatrix4fv(modelLocation, 1, GL_FALSE, &modelMat[0][0]);

	auto * model = Model::createPlaneWithHoles();
	model->render();
}

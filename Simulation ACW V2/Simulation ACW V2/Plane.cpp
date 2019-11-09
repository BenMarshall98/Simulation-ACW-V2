#include "Plane.h"
#include "gl.h"
#include "Model.h"
#include "SceneGraphNode.h"

Plane::Plane(const float pMass, const glm::vec3 pSize, const glm::vec3 pPos, const glm::vec3 pAngularVelocity, const glm::vec3 pVelocity) :
	RigidBody(pSize, pMass, pPos, pAngularVelocity, pVelocity, ObjectType::PLANE, glm::mat3())
{
}

void Plane::render(Shader* pShader) const
{
	auto modelMat = glm::mat4();

	if (mParent)
	{
		modelMat = modelMat * mParent->getRenderMatrix();
	}

	const auto translation = glm::mat4::createTranslation(mRenderPos);
	const auto scale = glm::mat4::createScale(mSize);
	const auto rot = glm::toMat4(mRenderRotation);
	const auto rotation = glm::mat4(rot[0][0], rot[0][1], rot[0][2], rot[0][3],
		rot[1][0], rot[1][1], rot[1][2], rot[1][3],
		rot[2][0], rot[2][1], rot[2][2], rot[2][3],
		rot[3][0], rot[3][1], rot[3][2], rot[3][3]);

	modelMat = modelMat * translation * rotation * scale;

	const auto modelLocation = glGetUniformLocation(pShader->getShaderId(), "model");
	modelMat.useMatrix(modelLocation);

	auto * model = Model::createPlane();
	model->render();
}
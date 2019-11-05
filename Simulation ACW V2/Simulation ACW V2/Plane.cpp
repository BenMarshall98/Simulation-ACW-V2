#include "Plane.h"
#include "Matrix4f.h"
#include "gl.h"
#include "Model.h"
#include "SceneGraphNode.h"

Plane::Plane(const float pMass, const Vector3F pSize, const Vector3F pPos, const Vector3F pAngularVelocity, const Vector3F pVelocity) :
	RigidBody(pSize, pMass, pPos, pAngularVelocity, pVelocity, ObjectType::PLANE, Matrix3F())
{
}

void Plane::render(Shader* pShader) const
{
	auto modelMat = Matrix4F();

	if (mParent)
	{
		modelMat = modelMat * mParent->getRenderMatrix();
	}

	const auto translation = Matrix4F::createTranslation(mRenderPos);
	const auto scale = Matrix4F::createScale(mSize);
	const auto rotation = Matrix4F(mRenderRotation);

	modelMat = modelMat * translation * rotation * scale;

	const auto modelLocation = glGetUniformLocation(pShader->getShaderId(), "model");
	modelMat.useMatrix(modelLocation);

	auto * model = Model::createPlane();
	model->render();
}
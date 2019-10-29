#include "PlaneHoles.h"
#include "Matrix4f.h"
#include "gl.h"
#include "SceneGraphNode.h"

PlaneHoles::PlaneHoles(Vector3F pSize, float pMass, Vector3F pPos, Vector3F pRotationAxis, float pRotationAngle, Vector3F pVelocity) :
	RigidBody(pSize, pMass, pPos, pRotationAxis, pRotationAngle, pVelocity, ObjectType::PLANEHOLES)
{

}

PlaneHoles::~PlaneHoles()
{

}

void PlaneHoles::render(Shader* pShader) const
{
	auto modelMat = Matrix4F();

	if (mParent)
	{
		modelMat = modelMat * mParent->getRenderMatrix();
	}

	const auto translation = Matrix4F::createTranslation(mRenderPos);
	const auto scale = Matrix4F::createScale(mSize);
	const auto rotation = Matrix4F::createRotation(mRotationAxis, mRotationAngle);

	modelMat = modelMat * translation * rotation * scale;

	const auto modelLocation = glGetUniformLocation(pShader->getShaderId(), "model");
	modelMat.useMatrix(modelLocation);

	auto * model = Model::CreatePlaneWithHoles();
	model->render();
}

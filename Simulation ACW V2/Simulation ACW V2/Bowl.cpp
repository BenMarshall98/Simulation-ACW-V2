#include "Bowl.h"
#include "Matrix4f.h"
#include "SceneGraphNode.h"

Bowl::Bowl(Vector3F pSize, float pMass, Vector3F pPos, Vector3F pAngulerVelocity ,Vector3F pVelocity) :
	RigidBody(pSize, pMass, pPos, pAngulerVelocity, pVelocity, ObjectType::BOWL, Matrix3F())
{

}

void Bowl::render(Shader* pShader) const
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

	auto * model = Model::createBowl();
	model->render();
}


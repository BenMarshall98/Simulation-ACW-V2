#include "Bowl.h"
#include "Matrix4F.h"
#include "SceneGraphNode.h"

Bowl::Bowl(const Vector3F pSize, const float pMass, const Vector3F pPos, const Vector3F pAngularVelocity, const Vector3F pVelocity) :
	RigidBody(pSize, pMass, pPos, pAngularVelocity, pVelocity, ObjectType::BOWL, Matrix3F())
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
	const auto rot = toMat4(mRenderRotation);
	const auto rotation = Matrix4F(rot[0][0], rot[0][1], rot[0][2], rot[0][3],
		rot[1][0], rot[1][1], rot[1][2], rot[1][3],
		rot[2][0], rot[2][1], rot[2][2], rot[2][3],
		rot[3][0], rot[3][1], rot[3][2], rot[3][3]);

	modelMat = modelMat * translation * rotation * scale;

	const auto modelLocation = glGetUniformLocation(pShader->getShaderId(), "model");
	modelMat.useMatrix(modelLocation);

	auto * model = Model::createBowl();
	model->render();
}


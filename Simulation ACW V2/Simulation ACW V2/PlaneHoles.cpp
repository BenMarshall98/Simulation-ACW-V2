#include "PlaneHoles.h"
#include "Matrix4f.h"
#include "gl.h"
#include "SceneGraphNode.h"

PlaneHoles::PlaneHoles(Vector3F pSize, float pMass, Vector3F pPos, Vector3F pAngularVelocity, Vector3F pVelocity) :
	RigidBody(pSize, pMass, pPos, pAngularVelocity, pVelocity, ObjectType::PLANEHOLES, Matrix3F())
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
	const auto rot = glm::toMat4(mRenderRotation);
	const auto rotation = Matrix4F(rot[0][0], rot[0][1], rot[0][2], rot[0][3],
		rot[1][0], rot[1][1], rot[1][2], rot[1][3],
		rot[2][0], rot[2][1], rot[2][2], rot[2][3],
		rot[3][0], rot[3][1], rot[3][2], rot[3][3]);

	modelMat = modelMat * translation * rotation * scale;

	const auto modelLocation = glGetUniformLocation(pShader->getShaderId(), "model");
	modelMat.useMatrix(modelLocation);

	auto * model = Model::createPlaneWithHoles();
	model->render();
}

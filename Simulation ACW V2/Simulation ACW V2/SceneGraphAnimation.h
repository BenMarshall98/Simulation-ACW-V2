#pragma once
class SceneGraphAnimation
{
protected:
	char mKey1;
	char mKey2;

public:
	SceneGraphAnimation(char pKey1, char pKey2);
	virtual ~SceneGraphAnimation();

	virtual void keyPressed(char pKey) = 0;
};


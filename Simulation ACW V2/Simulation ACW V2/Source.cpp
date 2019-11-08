#pragma comment(lib, "opengl32.lib")
#include "gl.h"
#include "GLFWWindow.h"
#include "Game.h"
#include "glm/glm.hpp"
#include "glm/gtc/quaternion.hpp"

int main()
{
	glm::quat startAngle = glm::angleAxis(glm::radians(60.0f), glm::vec3(1.0f, 0.0f, 0.0f));
	glm::vec3 changeAngle = glm::vec3(glm::radians(1.0f), 0.0f, 0.0f);
	glm::quat changeQuat = glm::quat(0.0f, changeAngle);

	glm::quat finishAngle = glm::normalize(startAngle + 0.5f * changeQuat * startAngle);
	
	
	auto window = GLFWWindow::instance();

	auto * game = new Game();

	game->run();

	delete game;

	delete window;
}
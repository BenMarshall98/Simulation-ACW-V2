#pragma comment(lib, "opengl32.lib")
#include "gl.h"
#include "GLFWWindow.h"
#include "Game.h"
#include "glm/glm.hpp"
#include "glm/gtc/quaternion.hpp"

int main()
{
	const auto window = GLFWWindow::instance();

	auto * game = new Game();

	game->run();

	delete game;

	delete window;
}
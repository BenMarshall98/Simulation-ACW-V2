#pragma comment(lib, "opengl32.lib")
#include "gl.h"
#include "GLFWWindow.h"
#include "Game.h"

int main()
{
	GLFWWindow::instance();

	auto * game = new Game();

	game->run();

	delete game;
}
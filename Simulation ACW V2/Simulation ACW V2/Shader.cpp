#include "Shader.h"
#include <fstream>
#include <sstream>

Shader::Shader(const std::string& pVertexProgram, const std::string& pFragmentProgram)
{
	int success;

	const unsigned int vertexShader = compileShader(pVertexProgram, GL_VERTEX_SHADER);
	const unsigned int fragmentShader = compileShader(pFragmentProgram, GL_FRAGMENT_SHADER);

	mShaderId = glCreateProgram();

	glAttachShader(mShaderId, vertexShader);
	glAttachShader(mShaderId, fragmentShader);

	glLinkProgram(mShaderId);
	glGetProgramiv(mShaderId, GL_LINK_STATUS, &success);

	if (!success)
	{
		//TODO: Error
	}

	glDetachShader(mShaderId, vertexShader);
	glDetachShader(mShaderId, fragmentShader);

	glDeleteShader(vertexShader);
	glDeleteShader(fragmentShader);
}

unsigned Shader::getShaderId() const
{
	return mShaderId;
}

void Shader::useShader()
{
	static Shader * lastShader = nullptr;

	if (this != lastShader)
	{
		glUseProgram(mShaderId);
		lastShader = this;
	}
}


int Shader::compileShader(const std::string& pFileName, const GLenum pShaderType)
{
	std::string shaderProgram;
	int success;

	readShader(pFileName, shaderProgram);
	const int shader = glCreateShader(pShaderType);
	auto program = shaderProgram.c_str();
	glShaderSource(shader, 1, &program, nullptr);
	glCompileShader(shader);

	glGetShaderiv(shader, GL_COMPILE_STATUS, &success);

	if (!success)
	{
		//TODO: Error
	}

	return shader;
}

auto Shader::readShader(const std::string& pFileName, std::string& pShaderProgram) -> void
{
	std::ifstream reader(pFileName.c_str());

	if (reader.fail())
	{
		//TODO: Error
		return;
	}

	std::ostringstream buffer;

	buffer << reader.rdbuf();
	pShaderProgram = buffer.str();
	reader.close();
}

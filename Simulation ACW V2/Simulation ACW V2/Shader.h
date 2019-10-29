#pragma once

#include <string>
#include "gl.h"

class Shader
{
public:
	Shader(const std::string& pVertexProgram, const std::string& pFragmentProgram);
	~Shader() = default;

	Shader(const Shader &) = delete;
	Shader(Shader &&) = delete;
	Shader & operator= (const Shader &) = delete;
	Shader & operator= (Shader &&) = delete;

	unsigned int getShaderId() const;
	void useShader();

	static int compileShader(const std::string& pFileName, GLenum pShaderType);
	static void readShader(const std::string& pFileName, std::string & pShaderProgram);

private:
	unsigned int mShaderId;
};


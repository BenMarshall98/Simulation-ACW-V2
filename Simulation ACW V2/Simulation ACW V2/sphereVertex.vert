#version 330

layout (location = 0) in vec3 position;
layout (location = 1) in vec2 texCoord;

uniform mat4 perspective;
uniform mat4 view;

uniform mat4 model;

out vec2 fTexCoord;

void main()
{
	fTexCoord = texCoord;
	gl_Position = perspective * view * model * vec4(position.xyz, 1.0);
	
}
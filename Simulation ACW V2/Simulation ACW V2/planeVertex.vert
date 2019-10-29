#version 330

layout (location = 0) in vec3 position;

uniform mat4 perspective;
uniform mat4 view;
uniform mat4 model;

void main()
{
	gl_Position = perspective * view * model * vec4(position.xyz, 1.0);
}
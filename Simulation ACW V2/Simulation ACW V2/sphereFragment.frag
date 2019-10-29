#version 330
uniform sampler2D text;

in vec2 fTexCoord;

out vec4 fragColor;

void main()
{
	fragColor = vec4(vec3(1.0, 0.0, 0.0) * texture(text, fTexCoord).rgb, 1.0);
}
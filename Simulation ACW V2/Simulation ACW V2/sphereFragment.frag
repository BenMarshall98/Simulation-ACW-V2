#version 330
uniform sampler2D text;
uniform vec3 color;

in vec2 fTexCoord;

out vec4 fragColor;

void main()
{
	fragColor = vec4(color * texture(text, fTexCoord).rgb, 1.0);
}
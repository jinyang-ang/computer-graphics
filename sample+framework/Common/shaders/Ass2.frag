#version 410 core
uniform vec3 fragmentColor;

// Output data
out vec3 color;

void main()
{
	color = fragmentColor;
}

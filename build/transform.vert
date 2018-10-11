#version 450

uniform mat4 Matrix;

layout(location = 0) in vec2 InPosition;

void main()
{
	gl_Position = Matrix * vec4(InPosition, 0, 1);
}
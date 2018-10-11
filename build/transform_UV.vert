#version 450

uniform mat4 Matrix;

layout(location = 0) in vec2 InPosition;
layout(location = 1) in vec2 InUV;

out VertexOut {
   vec2 UV;
} Out;

void main()
{
	gl_Position = Matrix * vec4(InPosition, 0, 1);
	Out.UV = InUV;
}
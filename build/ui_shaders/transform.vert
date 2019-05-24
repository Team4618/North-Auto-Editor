uniform mat4 Matrix;

layout(location = POSITION_SLOT) in vec2 InPosition;

void main()
{
	gl_Position = Matrix * vec4(InPosition, 0, 1);
}
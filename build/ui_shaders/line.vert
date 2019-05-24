uniform mat4 Matrix;
uniform float Width;

layout(location = POSITION_SLOT) in vec2 InPosition;
layout(location = NORMAL_SLOT) in vec2 InNormal;

out VertexOut {
   vec2 Offset;
} Out;

void main() {
   vec2 offset = InNormal * Width;
   vec2 pos = InPosition + offset;

   Out.Offset = offset;
   gl_Position = Matrix * vec4(pos, 0, 1);
}

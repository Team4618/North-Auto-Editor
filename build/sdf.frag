#version 450

uniform sampler2D Texture;
uniform vec4 Colour;

in VertexOut {
   vec2 UV;
} In;

layout(location = 0) out vec4 OutColor;

const float smoothing = 1.0/16.0;

void main() {
   float dist = texture(Texture, In.UV).a;
   float alpha = smoothstep(0.5 - smoothing, 0.5 + smoothing, dist);
	OutColor = vec4(Colour.rgb, Colour.a * alpha);
}
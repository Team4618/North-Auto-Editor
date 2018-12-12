#version 450

uniform vec4 Colour;
uniform float Feather;
uniform float Width;

in VertexOut {
   vec2 Offset;
} In;

layout(location = 0) out vec4 OutColor;

void main() {
   float solid_portion = Width - Feather;
   
   float perp_dist = length(In.Offset);
   float blend = clamp((perp_dist - solid_portion) / Feather, 0, 1);

   OutColor = mix(Colour, vec4(Colour.rgb, 0), blend);
}
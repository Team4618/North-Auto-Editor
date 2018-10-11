#version 450

uniform vec4 Colour;

layout(location = 0) out vec4 OutColor;

void main() {
	OutColor = Colour;
}
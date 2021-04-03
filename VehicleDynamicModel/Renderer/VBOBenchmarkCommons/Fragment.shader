#version 430
out vec4 color;
layout( std430, binding=10 ) buffer DebugBuffer
{
 vec4 debug; 
};
layout(std430, binding = 6) buffer ColorBuffer
{
	vec4 Color;
};
void main()
{
	color=Color;
}
#version 430
out vec4 color;
layout( std430, binding=10 ) buffer DebugBuffer
{
 vec4 debug; 
};
void main()
{
	color=vec4(0,1,0,1);
}
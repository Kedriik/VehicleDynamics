#version 430 
layout(location = 0) in vec4 vertexPosition_modelspace;
layout(std430, binding = 4) buffer PerFrame
{
	mat4 ViewMatrix;
};

layout(std430, binding = 5) buffer Constant
{
	mat4 ProjectionMatrix;
};
layout( std430, binding=10 ) buffer DebugBuffer
{
 vec4 debug; 
};
uniform mat4 ModelMatrix;
void main()
{
	gl_Position = ProjectionMatrix*ViewMatrix*ModelMatrix*vec4(vertexPosition_modelspace.xyz,1);
	
}
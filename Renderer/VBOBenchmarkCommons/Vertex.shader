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
void main()
{
	debug=vec4(11.1,11.1,11.1,11.1);
	gl_Position = ProjectionMatrix*ViewMatrix*vec4(vertexPosition_modelspace.xyz,1);
}
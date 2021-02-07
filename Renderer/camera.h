//#include <glm.hpp>
#pragma once
#include "glm\glm\glm.hpp"
using namespace glm;


class Camera 
{
GLFWwindow *window;

private:
	glm::vec3 forward, up, right, Position;
	quat camQuat;
	float sensRot, sensTrans;
	float sensRotInit, sensTransInit;
	mat4 viewMatrix;
	float windowHeight, windowWidth;
	vec3 LookAt;
public:
vec4 someStorage = vec4(0.0f);
Camera()
	{

	};
Camera(GLFWwindow *_window, float h, float w,  glm::vec3 p=vec3(0,0,0), vec3 f=vec3(1,0,0), vec3 u=vec3(0,1,0), float sr=0.001, float st=0.01, glm::vec3 look=vec3(0,0,0))
{
	window =_window;
	forward=f;
	up=u;
	camQuat=quat();
	Position=p;
	sensRot=sr;
	sensTrans=st;
	windowHeight=h;
	windowWidth=w;
	LookAt=look;
	sensRotInit=sr;
	sensTransInit=st;
}
~Camera()
{
	
}
void setPosition(vec3 pos)
{
	Position=pos;
}
vec3 getPosition()
{
	return Position;
}
void setForward(vec3 f)
{
	forward=f;
}
vec3 getForward()
{
	return forward;
}
float getCameraTransSens()
{
	return sensTrans;
};
float getCameraRotSens()
{
	return sensRot;
};
void setUp(vec3 u)
{
	up=u;
}
vec3 getUp()
{
	return up;
}
void setSens(float rot=0, float trans=0)
{
	if (rot != 0)
	{
		sensRotInit = rot;
		sensRot = rot;
	}
	if (trans != 0)
	{
		sensTransInit = trans;
		sensTrans = trans;
	}
}
void increaseSensTrans(float in)
{
sensTrans+=in;
}
void decreaseSensTrans(float dec)
{
sensTrans-=dec;
}
void setViewMatrix(const mat4 vm=mat4(1.0f))
{
	viewMatrix=vm;
}
mat4 getViewMatrix()
{
	return viewMatrix;
}
void setWindowDimensions()
{

}
void updateCamera(float rotX, float rotY, float rotZ,float X, float Y, float Z, double deltaTime)
{
	
	float sensRotLoc=sensRot*deltaTime;
	
	quat tempQuat=quat();
	forward=normalize(forward);
	up=normalize(up);
	right=cross(forward, up);
	if(sensRotLoc>6.28319)
		sensRotLoc-=6.28319;

	if(rotY!=0)
	{		
			tempQuat=glm::angleAxis(degrees(rotY*sensRotLoc),up);   
			forward=normalize(forward*tempQuat);
			up=normalize(up*tempQuat);
	}

	if(rotZ!=0)
	{		
			tempQuat=glm::angleAxis(degrees(rotZ*sensRotLoc),forward);    
			forward=normalize(forward*tempQuat);
			up=normalize(up*tempQuat);
	}
   
   if(rotX!=0)
	{
			tempQuat=glm::angleAxis(degrees(rotX*sensRotLoc),right);   
			forward=normalize(forward*tempQuat);
			up=normalize(up*tempQuat);
    }
  
	if(X!=0)
		Position.x=Position.x+X*sensTrans*deltaTime;
	if(Y!=0)
		Position.y=Position.y+Y*sensTrans*deltaTime;
	if(Z!=0)
		Position.z=Position.z+Z*sensTrans*deltaTime;

	viewMatrix = glm::lookAt
	(
		Position,					// Camera is here
		Position + forward,		// and looks here : at the same position, plus "direction"
		up							// Head is up (set to 0,-1,0 to look upside-down)
	);

}
glm::mat4 cameraPositionKeyboard(double deltaTime) 
{
	float xrot=0,yrot=0,zrot=0,x=0,y=0,z=0;
	
	
	if (glfwGetKey( window, GLFW_KEY_Q ) == GLFW_PRESS) 
		zrot=1;
	
	if (glfwGetKey( window, GLFW_KEY_E ) == GLFW_PRESS) 
		zrot=-1;
	
	if (glfwGetKey( window, GLFW_KEY_W ) == GLFW_PRESS) 
		xrot=1;
	
	if (glfwGetKey( window, GLFW_KEY_S ) == GLFW_PRESS) 
		xrot=-1;
	
	if (glfwGetKey( window, GLFW_KEY_D ) == GLFW_PRESS) 
		yrot=1;
	
	if (glfwGetKey( window, GLFW_KEY_A ) == GLFW_PRESS) 
		yrot=-1;
	
	
	/////////////////////
	
	
	if (glfwGetKey( window, GLFW_KEY_I ) == GLFW_PRESS) 
	{
		x+=forward.x;
		y+=forward.y;
		z+=forward.z;
	}

	if (glfwGetKey( window, GLFW_KEY_K ) == GLFW_PRESS) 
	{
		x-=forward.x;
		y-=forward.y;
		z-=forward.z;
	}

	if (glfwGetKey( window, GLFW_KEY_J ) == GLFW_PRESS) 
	{
		x-=right.x;
		y-=right.y;
		z-=right.z;
	}


	if (glfwGetKey( window, GLFW_KEY_L ) == GLFW_PRESS) 
	{
		x+=right.x;
		y+=right.y;
		z+=right.z;
	}

	if (glfwGetKey( window, GLFW_KEY_U ) == GLFW_PRESS) 
	{
		x-=up.x;
		y-=up.y;
		z-=up.z;
	}

	if (glfwGetKey( window, GLFW_KEY_O ) == GLFW_PRESS) 
	{
		x+=up.x;
		y+=up.y;
		z+=up.z;
	}
	
	updateCamera(xrot,yrot,zrot,x,y,z,deltaTime);
			 
			
	return viewMatrix;
}
glm::mat4 cameraPositionKeyboard(double deltaTime, bool *updated)
{
	float xrot = 0, yrot = 0, zrot = 0, x = 0, y = 0, z = 0;


	if (glfwGetKey(window, GLFW_KEY_Q) == GLFW_PRESS)
	{
		zrot = 1;
		*updated = true;
	}
	if (glfwGetKey(window, GLFW_KEY_E) == GLFW_PRESS)
	{
		zrot = -1;
		*updated = true;
	}

	if (glfwGetKey(window, GLFW_KEY_W) == GLFW_PRESS)
	{
		xrot = 1;
		*updated = true;
	}

	if (glfwGetKey(window, GLFW_KEY_S) == GLFW_PRESS)
	{
		xrot = -1;
		*updated = true;
	}

	if (glfwGetKey(window, GLFW_KEY_D) == GLFW_PRESS)
	{
		yrot = 1;
		*updated = true;
	}

	if (glfwGetKey(window, GLFW_KEY_A) == GLFW_PRESS)
	{
		yrot = -1;
		*updated = true;
	}


	/////////////////////


	if (glfwGetKey(window, GLFW_KEY_I) == GLFW_PRESS)
	{
		x += forward.x;
		y += forward.y;
		z += forward.z;
		*updated = true;
	}

	if (glfwGetKey(window, GLFW_KEY_K) == GLFW_PRESS)
	{
		x -= forward.x;
		y -= forward.y;
		z -= forward.z;
		*updated = true;
	}

	if (glfwGetKey(window, GLFW_KEY_J) == GLFW_PRESS)
	{
		x -= right.x;
		y -= right.y;
		z -= right.z;
		*updated = true;
	}


	if (glfwGetKey(window, GLFW_KEY_L) == GLFW_PRESS)
	{
		x += right.x;
		y += right.y;
		z += right.z;
		*updated = true;
	}

	if (glfwGetKey(window, GLFW_KEY_U) == GLFW_PRESS)
	{
		x -= up.x;
		y -= up.y;
		z -= up.z;
		*updated = true;
	}

	if (glfwGetKey(window, GLFW_KEY_O) == GLFW_PRESS)
	{
		x += up.x;
		y += up.y;
		z += up.z;
		*updated = true;
	}


	updateCamera(xrot, yrot, zrot, x, y, z, deltaTime);


	return viewMatrix;
}
glm::mat4 cameraPositionMouse(double deltaTime)
{
	float xrot=0,yrot=0,zrot=0,x=0,y=0,z=0;
	double xpos=windowWidth/2, ypos=windowHeight/2;
	glfwGetCursorPos(window, &xpos, &ypos);
	
	//windowHeight, windowWidth;
	// Reset mouse position for next frame
	glfwSetCursorPos(window, windowWidth/2, windowHeight/2);
	
	// Compute new orientation
	zrot+= float(windowWidth/2 - xpos );
	xrot+= float( windowHeight/2 - ypos );
	
	
	
	
	if (glfwGetKey( window, GLFW_KEY_I ) == GLFW_PRESS) 
	{
		x+=forward.x;
		y+=forward.y;
		z+=forward.z;
	}

	if (glfwGetKey( window, GLFW_KEY_K ) == GLFW_PRESS) 
	{
		x-=forward.x;
		y-=forward.y;
		z-=forward.z;
	}

	if (glfwGetKey( window, GLFW_KEY_J ) == GLFW_PRESS) 
	{
		x-=right.x;
		y-=right.y;
		z-=right.z;
	}


	if (glfwGetKey( window, GLFW_KEY_L ) == GLFW_PRESS) 
	{
		x+=right.x;
		y+=right.y;
		z+=right.z;
	}

	if (glfwGetKey( window, GLFW_KEY_U ) == GLFW_PRESS) 
	{
		x-=up.x;
		y-=up.y;
		z-=up.z;
	}

	if (glfwGetKey( window, GLFW_KEY_O ) == GLFW_PRESS) 
	{
		x+=up.x;
		y+=up.y;
		z+=up.z;
	}
	
	updateCamera(xrot,yrot,zrot,x,y,z,deltaTime);
			 
			viewMatrix       = glm::lookAt
								(
								Position,					// Camera is here
								Position+forward,		// and looks here : at the same position, plus "direction"
								up							// Head is up (set to 0,-1,0 to look upside-down)
								);
	return viewMatrix;
}
glm::mat4 cameraPositionAuto(vec3 pos, vec3 look)
{
	Position=pos;
	viewMatrix       = glm::lookAt
								(
								pos,					// Camera is here
								look,		// and looks here : at the same position, plus "direction"
								up							// Head is up (set to 0,-1,0 to look upside-down)
								);
	return viewMatrix;
}
glm::mat4 cameraPositionAutoAdaptiveCamera(double deltaTime)
{
float xrot=0,yrot=0,zrot=0,x=0,y=0,z=0;
	
	float wspT=deltaTime*sensTransInit, wspR=deltaTime*sensRotInit;

	if (glfwGetKey( window, GLFW_KEY_Q ) == GLFW_PRESS)
	{
		sensRot+=wspR;
		zrot=1;
	}
	else if (glfwGetKey( window, GLFW_KEY_E ) == GLFW_PRESS) 
	{
		sensRot+=wspR;
		zrot=-1;
	}
	else if (glfwGetKey( window, GLFW_KEY_W ) == GLFW_PRESS) 
	{
		sensRot+=wspR;
		xrot=1;
	}
	else if (glfwGetKey( window, GLFW_KEY_S ) == GLFW_PRESS) 
	{
		sensRot+=wspR;
		xrot=-1;
	}
	else if (glfwGetKey( window, GLFW_KEY_D ) == GLFW_PRESS) 
	{
		sensRot+=wspR;
		yrot=1;
	}
	else if (glfwGetKey( window, GLFW_KEY_A ) == GLFW_PRESS) 
	{
		sensRot+=wspR;
		yrot=-1;
	}
	else
	{
		sensRot=sensRotInit;
	}
	/////////////////////
	
	
	if (glfwGetKey( window, GLFW_KEY_I ) == GLFW_PRESS) 
	{
		sensTrans+=wspT;
		
		x+=forward.x;
		y+=forward.y;
		z+=forward.z;
	}

	else if (glfwGetKey( window, GLFW_KEY_K ) == GLFW_PRESS) 
	{
		sensTrans+=wspT;
		x-=forward.x;
		y-=forward.y;
		z-=forward.z;
	}

	else if (glfwGetKey( window, GLFW_KEY_J ) == GLFW_PRESS) 
	{
		sensTrans+=wspT;
		x-=right.x;
		y-=right.y;
		z-=right.z;
	}


	else if (glfwGetKey( window, GLFW_KEY_L ) == GLFW_PRESS) 
	{
		sensTrans+=wspT;
		x+=right.x;
		y+=right.y;
		z+=right.z;
	}

	else if (glfwGetKey( window, GLFW_KEY_U ) == GLFW_PRESS) 
	{
		sensTrans+=wspT;
		x-=up.x;
		y-=up.y;
		z-=up.z;
	}

	else if (glfwGetKey( window, GLFW_KEY_O ) == GLFW_PRESS) 
	{
		sensTrans+=wspT;
		x+=up.x;
		y+=up.y;
		z+=up.z;
	}
	else
	{
		sensTrans=sensTransInit;
	}
	
	updateCamera(xrot,yrot,zrot,x,y,z,deltaTime);
			viewMatrix       = glm::lookAt
								(
								Position,					// Camera is here
								Position+forward,		// and looks here : at the same position, plus "direction"
								up							// Head is up (set to 0,-1,0 to look upside-down)
								);
	return viewMatrix;
}
};
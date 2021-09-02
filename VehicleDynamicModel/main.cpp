#pragma once
#include "Renderer/Simulation.h"
#include "OpenDriveDocument.h"
#include <exception>
int main(int argc, char** argv)
{

	std::string openDriveFilePath;
	if (argc > 1) {
		openDriveFilePath = std::string(argv[1]);
	}
	else {
		openDriveFilePath = "Ex_Line-Spiral-Arc.xodr";
	}
	int generate_gaps = false;
	if (argc > 3) {
		Road::S_STEP = std::stod(argv[2]);
		Road::T_STEP = std::stod(argv[3]);
	}
	else {
		Road::S_STEP = 10.1;
		Road::T_STEP = 1.1;
	}
	Road::debugRender = false;
	Simulation simulation(openDriveFilePath);
	simulation.doPhysics = true;
	simulation.init(1500, 1000);
	simulation.launchLoop();
}
#pragma once
#include "tinyxml2/tinyxml2.h"
#include <iostream>
#include <fstream>
#include <string.h>
#include <stdio.h>
#include <optional>
#include <iostream>
class XMLDocument;
class XMLElement;
enum XMLError;
class XMLText;
struct header {
	header(int _revMajor, int _revMinor, std::optional<std::string> _name = std::optional<std::string>(),
		//std::optional<t_header_Version> version;
		std::optional<std::string> _date = std::optional<std::string>(),
		std::optional<double> _north = std::optional<double>(),
		std::optional<double> _south = std::optional<double>(),
		std::optional<double> _east = std::optional<double>(),
		std::optional<double> _west = std::optional<double>(),
		std::optional<std::string> _vendor = std::optional<std::string>()) :revMajor(_revMajor), revMinor(_revMinor), name(_name),
		date(_date), north(_north), south(_south), east(_east), west(_west), vendor(_vendor) {

	}

	int revMajor;
	int revMinor;
	std::optional<std::string> name;
	//std::optional<t_header_Version> version;
	std::optional<std::string> date;
	std::optional<double> north;
	std::optional<double> south;
	std::optional<double> east;
	std::optional<double> west;
	std::optional<std::string> vendor;

};
struct Geometry {
	Geometry(double _s, double _x, double _y, double _hdg, double _length):
		x(_x), y(_y), hdg(_hdg) {
		if (_s < 0) throw "s value cannot be negative";
		if (_length < 0) throw "length cannot be negative";
		s = _s;
		length = _length;
	}
	double s;
	double x;
	double y;
	double hdg;
	double length;
};
struct Line:Geometry {
	Line(double _s, double _x, double _y, double _hdg, double _length):
		Geometry( _s,_x,_y,_hdg,_length) {

	}
};
struct Spiral :Geometry {
	Spiral(double _curvStart, double _curvEnd, double _s, double _x, double _y, double _hdg, double _length):
		curvStart(_curvStart), curvEnd(_curvEnd),Geometry( _s,_x,_y,_hdg,_length) {

	}
	double curvStart;
	double curvEnd;
};

struct Arc :Geometry {
	Arc(double _curvature, double _s, double _x, double _y, double _hdg, double _length):
		curvature(_curvature),Geometry(_s,_x,_y,_hdg,_length) {

	}
	double curvature;
};

struct Poly3 :Geometry {
	Poly3(double _a, double _b, double _c, double _d, double _s, double _x, double _y, double _hdg, double _length) :
		a(_a), b(_b), c(_c), d(_d), Geometry(_s, _x, _y, _hdg, _length) {

	}
	double a, b, c, d;
};

class OpenDriveDocument
{
public:
	OpenDriveDocument(){
		std::string filePath = "D:\\Miscellaneous Projects\\VehicleDynamicModel\\VehicleDynamicModel\\VehicleDynamicModel\\Ex_Line-Spiral-Arc.xodr";
		std::ifstream myfile;
		tinyxml2::XMLDocument doc;
		doc.LoadFile(filePath.c_str());
		tinyxml2::XMLNode *n = doc.FirstChild();
		//tinyxml2::XMLElement* e = 
		//n->ToText()->
		tinyxml2::XMLError error = doc.ErrorID();
		// Navigate to the title, using the convenience function,
		// with a dangerous lack of error checking.
		tinyxml2::XMLNode* node = doc.FirstChild()->NextSibling();
		tinyxml2::XMLElement* e1 = doc.FirstChildElement()->FirstChildElement("header");
		float f;
		error = e1->QueryFloatAttribute("revMinor", &f);
		printf("Name of play (1): %f\n", f);

		// Text is just another Node to TinyXML-2. The more
		// general way to get to the XMLText:
		tinyxml2::XMLText* textNode = doc.FirstChildElement("OpenDRIVE")->FirstChildElement("header")->FirstChild()->ToText();
		
		
	}
	int parsePlanView() {
		throw "Not implemented";
		return -1;
	}
	
};


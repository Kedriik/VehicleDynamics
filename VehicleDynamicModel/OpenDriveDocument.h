#pragma once
#include "tinyxml2/tinyxml2.h"
#include <iostream>
#include <fstream>
#include <string.h>
#include <stdio.h>
#include <optional>
#include <iostream>
//#include "glm\glm\glm.hpp"
#include "../Renderer/glm/glm/glm.hpp"
#include <vector>
//using namespace glm;
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
namespace PlanView{
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

		virtual std::vector<glm::vec4> generateReferenceLine() = 0;
	};
	struct Line:Geometry {
		Line(double _s, double _x, double _y, double _hdg, double _length):
			Geometry( _s,_x,_y,_hdg,_length) {
		}
		std::vector<glm::vec4> generateReferenceLine()override {
			std::vector<glm::vec4> vertices;
			return vertices;
		}
	};
	struct Spiral:Geometry {
		Spiral(double _curvStart, double _curvEnd, double _s, double _x, double _y, double _hdg, double _length):
			curvStart(_curvStart), curvEnd(_curvEnd),Geometry( _s,_x,_y,_hdg,_length) {

		}
		double curvStart;
		double curvEnd;
		std::vector<glm::vec4> generateReferenceLine()override {
			std::vector<glm::vec4> vertices;
			return vertices;
		}
	};

	struct Arc:Geometry {
		Arc(double _curvature, double _s, double _x, double _y, double _hdg, double _length):
			curvature(_curvature),Geometry(_s,_x,_y,_hdg,_length) {

		}
		double curvature;
		std::vector<glm::vec4> generateReferenceLine()override {
			std::vector<glm::vec4> vertices;
			return vertices;
		}
	};

	struct Poly3:Geometry {
		Poly3(double _a, double _b, double _c, double _d, double _s, double _x, double _y, double _hdg, double _length) :
			a(_a), b(_b), c(_c), d(_d), Geometry(_s, _x, _y, _hdg, _length) {

		}
		double a, b, c, d;
		std::vector<glm::vec4> generateReferenceLine()override {
			std::vector<glm::vec4> vertices;
			return vertices;
		}
	};
	struct ParamPoly3:Geometry {
		ParamPoly3(double _aU, double _bU, double _cU, double _dU, double _aV, double _bV, double _cV, double _dV, std::string _pRange
			, double _s, double _x, double _y, double _hdg, double _length) :aU(_aU), bU(_bU), cU(_cU), dU(_dU),
			aV(_aV), bV(_bV), cV(_cV), dV(_dV),Geometry(_s, _x, _y, _hdg, _length) {
			if (_pRange != "arcLength" || _pRange != "normalized") throw "Unknow value for pRange:" + _pRange;
			pRange = _pRange;

		}
		double aU, bU, cU, dU, aV, bV, cV, dV;
		std::string pRange;
		std::vector<glm::vec4> generateReferenceLine()override {
			std::vector<glm::vec4> vertices;
			return vertices;
		}
	};
}
class OpenDriveDocument
{
public:
	tinyxml2::XMLDocument doc;
	OpenDriveDocument(){
		std::string filePath = "D:\\Miscellaneous Projects\\VehicleDynamicModel\\VehicleDynamicModel\\VehicleDynamicModel\\Ex_Line-Spiral-Arc.xodr";
		std::ifstream myfile;
		doc.LoadFile(filePath.c_str());
		this->parsePlanView(this->doc);
	}

	int parsePlanView(tinyxml2::XMLDocument& doc) {
		std::vector<PlanView::Geometry*> geometries;
		tinyxml2::XMLElement* n = doc.FirstChildElement()->FirstChildElement("road")->FirstChildElement("planView")->FirstChildElement();
		while (n!=nullptr) {
			tinyxml2::XMLElement* g = n->FirstChildElement();
			const char* v = g->Value();
			const char* v1 = g->Value();
			double s,  x,  y,  hdg,  length;
			PlanView::Geometry* geometry = nullptr;
			tinyxml2::XMLError error = n->QueryDoubleAttribute("s", &s);
			error = n->QueryDoubleAttribute("x", &x);
			error = n->QueryDoubleAttribute("y", &y);
			error = n->QueryDoubleAttribute("hdg", &hdg);
			error = n->QueryDoubleAttribute("length", &length);
			if(std::string(v) == "line"){
				
				geometry = new PlanView::Line(s, x, y, hdg, length);
				//std::cout << v << std::endl;
			}
			else if (std::string(v) == "arc") {
				double curvature;
				error = n->QueryDoubleAttribute("curvature", &curvature);
				//geometry = new Arc(curvature, s, x, y, hdg, length);
				geometry = new PlanView::Arc(curvature, s, x, y, hdg, length);
				
			}
			else if (std::string(v) == "spiral") {
				double curvStart,curvEnd;
				error = n->QueryDoubleAttribute("curvStart", &curvStart);
				error = n->QueryDoubleAttribute("curvEnd", &curvEnd);
				geometry = new PlanView::Spiral(curvStart, curvEnd,s, x, y, hdg, length);
			}
			geometries.push_back(geometry);
			n = n->NextSiblingElement();
		}
		///tinyxml2::XMLNode* n2 = doc.NextSibling();
		
		throw "Not implemented";
		return -1;
	}
	
};


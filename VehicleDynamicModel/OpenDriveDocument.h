#pragma once
#include "tinyxml2/tinyxml2.h"
#include <iostream>
#include <fstream>
#include <string.h>
#include <stdio.h>
#include <optional>
#include "../Renderer/glm/glm/glm.hpp"
#include <vector>
#include <math.h>
//#include "..\Renderer\glm\gtx\transform.hpp"
#include "../Renderer/glm/glm/gtx/transform.hpp"
#include "../Renderer/glm/glm/gtc/matrix_transform.hpp"
#include "../Renderer/glm/glm/gtc/quaternion.hpp"
#include "../Renderer/glm/glm/gtx/quaternion.hpp"
#include <iostream>
#include <functional>
double fx(double t) {
	return cos(t * t);
}
double fy(double t) {
	return sin(t * t);
}
template<typename T>
T trapezoidalIntegral(std::function<double (double)> f, T a, T b, int n) {
	T h = (b - a) / n;
	T result = 0.5 * f(a) + 0.5 * f(b);
	for (int i = 0; i < n; i++) {
		result += f(a + i * h);
	}
	result *= h;
	return result;
}
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
		int n_vertices;
		std::vector<glm::vec4> vertices;
		virtual void generateReferenceLine() = 0;
	};
	struct Line:Geometry {
		Line(double _s, double _x, double _y, double _hdg, double _length):
			Geometry( _s,_x,_y,_hdg,_length) {
			n_vertices = 2;
		}
		void generateReferenceLine()override {
			double step = this->length / this->n_vertices;
			for (int i = 0; i <= this->n_vertices; i++) {
				glm::dvec4 position;
				position.x = this->x + cos(this->hdg) * i*step;
				position.y = this->y + sin(this->hdg) * i+step;
				position.z = 0;
				this->vertices.push_back(position);
			}
		}
	};
	struct Spiral:Geometry {
		Spiral(double _curvStart, double _curvEnd, double _s, double _x, double _y, double _hdg, double _length):
			curvStart(_curvStart), curvEnd(_curvEnd),Geometry( _s,_x,_y,_hdg,_length) {
			n_vertices = 100;

		}
	private:
		double curvStart;
		double curvEnd;
		double angleOffset = 0;
	public:
		void calcAngleOffset(double x_origin, double y_origin, double x_origin_d, double y_origin_d) {
			glm::dvec2 vecStart;
			vecStart.x = x_origin;
			vecStart.y = y_origin;
			glm::dvec2 vecEnd;
			vecEnd.x = x_origin_d;
			vecEnd.y = y_origin_d;
			glm::dvec2 temp ;
			glm::dvec2 normalized;
			glm::dvec2 OX ;
			OX.x = 1;
			OX.y = 0;
			normalized = glm::normalize(vecEnd-vecStart);
			//let dot = x1*x2 + y1*y2  # dot product between [x1, y1] and [x2, y2]
			//det = x1*y2 - y1*x2      # determinant
			//angle = atan2(det, dot)  # atan2(y, x) or atan2(sin, cos)	
			double dot = normalized.x * OX.x + normalized.y * OX.y;
			double det = normalized.x * OX.y - normalized.y * OX.x;
			this->angleOffset = atan2(det, dot);
		}
		void transformAndAddVertex(double x, double y, double x_origin, double y_origin) {
			glm::dvec4 position;
			glm::dvec3 origin;
			origin.x = 0;
			origin.y = 0;
			origin.z = 1;
			position.x = x;
			position.y = y;
			position.z = 0;
			position.x -= x_origin;
			position.y -= y_origin;
			glm::dquat rotate = glm::angleAxis(glm::degrees((this->hdg + this->angleOffset)), origin);
			position = position * rotate;
			position.x += this->x;
			position.y += this->y;
			this->vertices.push_back(position);
		}
		void generateReferenceLine()override {
			double  a = 0;
			int n_integral = 4000;
			double x;
			double y;
			double start_curvature, end_curvature;
			bool reversed = false;
			int direction = 1;
			if (this->curvEnd != 0 && this->curvStart == 0) {
				int dir = 1;
				if (this->curvEnd < 0) {
					dir = -1;
				}
				a = 1 / (sqrt(2 * (1 / abs(this->curvEnd)) * this->length));
				double step = this->length / this->n_vertices;
				for (int i = 0; i <= this->n_vertices; i++) {
					x = (1 / a) * trapezoidalIntegral(fx, 0.0, a * step * i, n_integral);
					y = dir * (1 / a) * trapezoidalIntegral(fy, 0.0, a * step * i, n_integral);
					this->transformAndAddVertex(x, y, 0, 0);
				}
			}
			else if (this->curvEnd == 0 && this->curvStart != 0) {

				a = 1 / (sqrt(2 * (1 / abs(this->curvStart)) * this->length));
				double dir = 1;
				if (this->curvStart < 0) {
					dir = -1;
				}
				double step = this->length / this->n_vertices;
				double ds = 0.001;
				double x_origin = (1 / a) * trapezoidalIntegral(fx, 0.0, -a * this->length, n_integral);
				double y_origin = dir * (1 / a) * trapezoidalIntegral(fy, 0.0, a * this->length, n_integral);
				double x_origin_d = (1 / a) * trapezoidalIntegral(fx, 0.0, -a * (this->length - ds), n_integral);
				double y_origin_d = dir * (1 / a) * trapezoidalIntegral(fy, 0.0, a * (this->length - ds), n_integral);
				this->calcAngleOffset(x_origin, y_origin, x_origin_d, y_origin_d);
				for (int i = 0; i <= this->n_vertices; i++) {
					x = (1 / a) * trapezoidalIntegral(fx, 0.0, -a * (this->length - step * i), n_integral);
					y = dir * (1 / a) * trapezoidalIntegral(fy, 0.0, a * (this->length - step * i), n_integral);
					this->transformAndAddVertex(x, y, x_origin, y_origin);
				}
			}
			else if (glm::sign(this->curvEnd) != glm::sign(this->curvStart)) {
				double adjusted_len = this->length / 2;

				double step = adjusted_len / this->n_vertices;
				a = 1 / (sqrt(2 * (1 / abs(this->curvStart)) * adjusted_len));
				int dir_ys = 1;
				int dir_ye = 1;
				if (this->curvStart < 0) {
					dir_ys = -1;
				}
				if (this->curvEnd < 0) {
					dir_ye = -1;
				}

				double ds = 0.0001;
				double x_origin = (1 / a) * trapezoidalIntegral(fx, 0.0, -a * adjusted_len, n_integral);
				double y_origin = dir_ys * (1 / a) * trapezoidalIntegral(fy, 0.0, a * adjusted_len, n_integral);
				double x_origin_d = (1 / a) * trapezoidalIntegral(fx, 0.0, -a * (adjusted_len - ds), n_integral);
				double y_origin_d = dir_ys * (1 / a) * trapezoidalIntegral(fy, 0.0, a * (adjusted_len - ds), n_integral);
				this->calcAngleOffset(x_origin, y_origin, x_origin_d, y_origin_d);
				for (int i = 0; i <= this->n_vertices; i++) {
					x = (1 / a) * trapezoidalIntegral(fx, 0.0, -a * (adjusted_len - step * i), n_integral);
					y = dir_ys * (1 / a) * trapezoidalIntegral(fy, 0.0, a * (adjusted_len - step * i), n_integral);
					this->transformAndAddVertex(x, y, x_origin, y_origin);
				}

				a = 1 / (sqrt(2 * (1 / abs(this->curvEnd)) * adjusted_len));
				for (int i = 0; i <= this->n_vertices; i++) {
					x = (1 / a) * trapezoidalIntegral(fx, 0.0, a * step * i, n_integral);
					y = dir_ye * (1 / a) * trapezoidalIntegral(fy, 0.0, a * step * i, n_integral);
					this->transformAndAddVertex(x, y, x_origin, y_origin);
				}
			}
			else {
				double direction = 1;
				if (abs(this->curvEnd) > abs(this->curvStart)) {
					start_curvature = abs(this->curvStart);
					end_curvature = abs(this->curvEnd);
					if (this->curvEnd < 0) {
						direction = -1;
					}
				}
				else {
					start_curvature = abs(this->curvEnd);
					end_curvature = abs(this->curvStart);
					if (this->curvStart > 0) {
						direction = -1;
					}
					reversed = true;
				}


				double L0 = 0;
				if (start_curvature != 0) {
					L0 = ((1 / end_curvature) * this->length) / abs(((1 / start_curvature) - (1 / end_curvature)));
				}
				double total_len = this->length + L0;
				a = 1 / (sqrt(2 * (1 / abs(end_curvature)) * (total_len)));
				double step = this->length / this->n_vertices;
				double x_origin = 0, y_origin = 0, x_origin_d = 0, y_origin_d = 0;
				double ds = 0.001;
				n_integral = 4000;
				if (reversed) {
					x_origin = (1 / a) * trapezoidalIntegral(fx, 0.0, a * (total_len), n_integral);
					y_origin = direction * (1 / a) * trapezoidalIntegral(fy, 0.0, a * (total_len), n_integral);
					x_origin_d = (1 / a) * trapezoidalIntegral(fx, 0.0, a * (total_len - ds), n_integral);
					y_origin_d = direction * (1 / a) * trapezoidalIntegral(fy, 0.0, a * (total_len - ds), n_integral);
				}
				else {
					x_origin = (1 / a) * trapezoidalIntegral(fx, 0.0, a * (L0), n_integral);
					y_origin = direction * (1 / a) * trapezoidalIntegral(fy, 0.0, a * (L0), n_integral);
					x_origin_d = (1 / a) * trapezoidalIntegral(fx, 0.0, a * (L0 + ds), n_integral);
					y_origin_d = direction * (1 / a) * trapezoidalIntegral(fy, 0.0, a * (L0 + ds), n_integral);
				}
				this->calcAngleOffset(x_origin, y_origin, x_origin_d, y_origin_d);
				for (double i = 0; i <= this->n_vertices; i++) {
					x = (1 / a) * trapezoidalIntegral(fx, 0.0, a * (step * i + L0), n_integral);
					y = direction * (1 / a) * trapezoidalIntegral(fy, 0.0, a * (step * i + L0), n_integral);

					this->transformAndAddVertex(x, y, x_origin, y_origin);
				}

			}
		}
	};

	struct Arc:Geometry {
		Arc(double _curvature, double _s, double _x, double _y, double _hdg, double _length):
			curvature(_curvature),Geometry(_s,_x,_y,_hdg,_length) {
			n_vertices = 100;
		}
		double curvature;
		void generateReferenceLine()override {
			double step = this->length / this->n_vertices;
			for (int i = 0; i <= this->n_vertices; i++) {
				glm::dvec4 position;
				double fi = i*step * this->curvature + this->hdg;
				position.x = (1.0 / this->curvature) * (sin(fi) - sin(this->hdg)) + this->x;
				position.y = (1.0 / this->curvature) * (cos(this->hdg) - cos(fi)) + this->y;
				position.z = 0;
				position.z = 0;
				this->vertices.push_back(position);
			}
		}
	};

	struct Poly3:Geometry {
		Poly3(double _a, double _b, double _c, double _d, double _s, double _x, double _y, double _hdg, double _length) :
			a(_a), b(_b), c(_c), d(_d), Geometry(_s, _x, _y, _hdg, _length) {
			n_vertices = 100;
		}
		double a, b, c, d;
		void generateReferenceLine()override {
		}
	};
	struct ParamPoly3:Geometry {
		ParamPoly3(double _aU, double _bU, double _cU, double _dU, double _aV, double _bV, double _cV, double _dV, std::string _pRange
			, double _s, double _x, double _y, double _hdg, double _length) :aU(_aU), bU(_bU), cU(_cU), dU(_dU),
			aV(_aV), bV(_bV), cV(_cV), dV(_dV),Geometry(_s, _x, _y, _hdg, _length) {
			if (_pRange != "arcLength" || _pRange != "normalized") throw "Unknow value for pRange:" + _pRange;
			pRange = _pRange;
			n_vertices = 100;
		}
		double aU, bU, cU, dU, aV, bV, cV, dV;
		std::string pRange;
		void generateReferenceLine()override {

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
		
		return 0;
	}
	
};


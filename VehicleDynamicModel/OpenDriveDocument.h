#pragma once
#ifndef OpenDriveDocument_h
#define OpenDriveDocument_h
#include "tinyxml2/tinyxml2.h"
#include <iostream>
#include <fstream>
#include <string.h>
#include <stdio.h>
#include <optional>
#include "Renderer/glm/glm/glm.hpp"
#include <vector>
#include <math.h>
#include <algorithm>
#include "Renderer/glm/glm/gtx/transform.hpp"
#include "Renderer/glm/glm/gtc/matrix_transform.hpp"
#include "Renderer/glm/glm/gtc/quaternion.hpp"
#include "Renderer/glm/glm/gtx/quaternion.hpp"
#include <iostream>
#include <functional>
#include <exception>
#include "btBulletDynamicsCommon.h"
#include <array>
#include "CDT/include/CDT.h"
//#include "Renderer/Simulation.h"
#include "Definitions.h"
#include <chrono>
#include <thread>
#include <mutex>
#include <random>

#define GLM_SWIZZLE
namespace OpenDriveMath {
	double static fx(double t) {
		return cos(t * t);
	}
	
	double static fy(double t) {
		return sin(t * t);
	}
};
template<typename F, typename T>
T trapezoidalIntegral(F f, T a, T b, int n) {
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
//enum XMLError;
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
enum TrafficRule {
	defaultTrafficRule = 0
};
enum class ElementType {
	road = 0,
	junction = 1
};
enum class ContactPoint {
	start = 0,
	end = 1
};
struct Predecessor {
	std::string elementId;
	ElementType elementType;
	std::optional <ContactPoint> contactPoint;
	std::optional<double> elementS;
	std::optional<int> elementDir;
	Predecessor(std::string _elementId, ElementType _elementType,
		std::optional <double> _elementS = std::optional<double>(),
		std::optional <int> _elementDir = std::optional <int>(),
		std::optional<ContactPoint> _contactPoint = std::optional<ContactPoint>()) :
		elementId(_elementId), contactPoint(_contactPoint), elementType(_elementType), elementS(_elementS), elementDir(_elementDir) {
	}
	Predecessor() {

	}
};
struct Successor {
	std::string elementId;
	ElementType elementType;
	std::optional <ContactPoint> contactPoint;
	std::optional<double> elementS;
	std::optional<int> elementDir;
	Successor(std::string _elementId, ElementType _elementType,
		std::optional <double> _elementS = std::optional<double>(),
		std::optional <int> _elementDir = std::optional <int>(),
		std::optional<ContactPoint> _contactPoint = std::optional<ContactPoint>()) :
		elementId(_elementId), contactPoint(_contactPoint), elementType(_elementType), elementS(_elementS), elementDir(_elementDir) {
	}
	Successor() {

	}
};
struct Link {
	std::optional<Successor> successor = std::optional<Successor>();
	std::optional<Predecessor> predecessor = std::optional<Predecessor>();
};
enum class CountryCode {
	PL = 0,
	DE = 1
};
enum class SpeedUnit {
	mps = 0,   //m/s
	mph = 1,
	kmph = 2  //km/h
};
enum class DistanceUnit {
	m, km, ft, mile
};
enum class MassUnit {
	kg, t
};
enum class Slope {
	percent
};
struct Speed {
	std::string maxSpeed;
	SpeedUnit speedUnit;
	Speed(std::string _maxSpeed = "undefined", SpeedUnit _speedUnit = SpeedUnit::kmph) :
		maxSpeed(_maxSpeed), speedUnit(_speedUnit) {}
};
struct Type {
	double s;
	std::string type;
	std::optional<CountryCode> countryCode;
	std::optional<Speed> speed;
	Type(double _s, std::string _type,
		std::optional<CountryCode> _countryCode = std::optional<CountryCode>(),
		std::optional<Speed> _speed = std::optional<Speed>())
		:s(_s), type(_type), countryCode(_countryCode), speed(_speed) {}
};
struct Lane {
	Lane() {

	}
	~Lane() {

	}
};
struct Object {
	Object() {

	}
	~Object() {

	}
};
struct Signal {
	Signal() {

	}
	~Signal() {

	}
};
struct Railroad {
	Railroad() {

	}
	~Railroad() {

	}
};
struct Surface {
	Surface() {

	}
	~Surface() {

	}
};
struct Shape {
	double s, t, a, b, c, d;
	Shape(double _s, double _t, double _a, double _b, double _c, double _d) :
		t(_t), a(_a), b(_b), c(_c), d(_d) {
		if (_s < 0) throw new std::exception("s value cannot be negative!");
		s = _s;
	}
};
struct Superelevation {
	double s, a, b, c, d;
	Superelevation(double _s, double _a, double _b, double _c, double _d) :
		a(_a), b(_b), c(_c), d(_d) {
		if (_s < 0) throw new std::exception("s value cannot be negative!");
		s = _s;
	}
};
struct SBasedProperty {

	template <class T>
	T getCurrentProp(double s, std::vector<T> sBasedProperty, T defaultProp) {
		int i = 0;
		int current = -1;
		for (i = 0; i < sBasedProperty.size(); i++) {
			if (s >= sBasedProperty.at(i).s) current = i;
			else break;
		}
		if (current >= 0)
			return sBasedProperty.at(current);
		return defaultProp;
	}
	template <class T>
	T getCurrentPropSOffset(double sOffset, std::vector<T> sBasedProperty, T defaultProp) {
		int i = 0;
		int current = -1;
		for (i = 0; i < sBasedProperty.size(); i++) {
			if (sOffset >= sBasedProperty.at(i).sOffset) current = i;
			else break;
		}
		if (current >= 0)
			return sBasedProperty.at(current);
		return defaultProp;
	}
};
struct LateralProfile :SBasedProperty {
	std::vector<Shape> shapes;
	std::vector<Superelevation> superelevations;
	LateralProfile(std::vector<Shape> _shapes, std::vector<Superelevation> _superelevations) :
		shapes(_shapes), superelevations(_superelevations) {}
	LateralProfile() {}
	Superelevation getCurrentSuperelevation(double s) {
		Superelevation defaultSuperelevation(0, 0, 0, 0, 0);
		return this->getCurrentProp(s, this->superelevations, defaultSuperelevation);
	}
};
struct Elevation {
	double s, a, b, c, d;
	Elevation(double _s, double _a, double _b, double _c, double _d) :
		a(_a), b(_b), c(_c), d(_d) {
		if (_s < 0) throw new std::exception("s value cannot be negative!");
		s = _s;
	}
};
struct ElevationProfile :SBasedProperty {
private:
	std::vector<Elevation> elevations;
public:
	ElevationProfile(std::vector<Elevation> _elevations) :elevations(_elevations) {}
	ElevationProfile() {}
	Elevation getCurrentElevation(double s) {
		Elevation defaultElevation(0, 0, 0, 0, 0);
		return this->getCurrentProp(s, this->elevations, defaultElevation);
	}
};
struct Geometry {
	Geometry(double _s, double _x, double _y, double _hdg, double _length) :
		x(_x), y(_y), hdg(_hdg) {
		if (_s < 0) throw new std::exception("s value cannot be negative");
		if (_length < 0) throw new std::exception("length cannot be negative");
		s = _s;
		length = _length;
		n_vertices = (int(length)/10 + 1);
	}
	double s;
	double x;
	double y;
	double hdg;
	double length;

	int n_vertices;
	std::string name;
	std::vector<glm::dvec4> vertices;
	virtual void generateReferenceLine() = 0;
	virtual glm::dvec4 generatePosition(double s) = 0;
};
struct line :Geometry {
	line(double _s, double _x, double _y, double _hdg, double _length) :
		Geometry(_s, _x, _y, _hdg, _length) {
	}
	void generateReferenceLine()override {
		double step = this->length / this->n_vertices;
		for (int i = 0; i <= this->n_vertices; i++) {
			glm::dvec4 position;
			position.x = this->x + cos(this->hdg) * i * step;
			position.y = this->y + sin(this->hdg) * i * step;
			position.z = 0;
			this->vertices.push_back(position);
		}
	}
	glm::dvec4 generatePosition(double s) {
		//if (s > this->length) s = this->length;
		glm::dvec4 position;
		position.x = this->x + cos(this->hdg) * s;
		position.y = this->y + sin(this->hdg) * s;
		position.z = 0;
		return position;
	}
};
using OpenDriveMath::fx;
using OpenDriveMath::fy;
struct spiral :Geometry {
	spiral(double _curvStart, double _curvEnd, double _s, double _x, double _y, double _hdg, double _length) :
		curvStart(_curvStart), curvEnd(_curvEnd), Geometry(_s, _x, _y, _hdg, _length) {

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
		glm::dvec2 temp;
		glm::dvec2 normalized;
		glm::dvec2 OX;
		OX.x = 1;
		OX.y = 0;
		normalized = glm::normalize(vecEnd - vecStart);
		//let dot = x1*x2 + y1*y2  # dot product between [x1, y1] and [x2, y2]
		//det = x1*y2 - y1*x2      # determinant
		//angle = atan2(det, dot)  # atan2(y, x) or atan2(sin, cos)	
		double dot = normalized.x * OX.x + normalized.y * OX.y;
		double det = normalized.x * OX.y - normalized.y * OX.x;
		this->angleOffset = atan2(det, dot);
	}
	glm::dvec4 transformAndReturnVertex(double x, double y, double x_origin, double y_origin) {
		glm::dvec4 position;
		glm::dvec3 origin;
		origin.x = 0;
		origin.y = 0;
		origin.z = 1;
		position.x = x;
		position.y = y;
		position.z = 0;
		position.w = 1;
		position.x -= x_origin;
		position.y -= y_origin;
		glm::dquat rotate = glm::angleAxis(this->hdg + this->angleOffset, origin);
		position = rotate * position;
		position.x += this->x;
		position.y += this->y;
		return position;
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
		position.w = 1;
		position.x -= x_origin;
		position.y -= y_origin;
		glm::dquat rotate = glm::angleAxis(this->hdg + this->angleOffset, origin);
		position = rotate * position;
		position.x += this->x;
		position.y += this->y;
		this->vertices.push_back(position);
	}
	void generateReferenceLine()override {
		double  a = 0;
		int n_integral = 1000;
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
			double x_origin = (1 / a) * trapezoidalIntegral(OpenDriveMath::fx, 0.0, -a * this->length, n_integral);
			double y_origin = dir * (1 / a) * trapezoidalIntegral(OpenDriveMath::fy, 0.0, a * this->length, n_integral);
			double x_origin_d = (1 / a) * trapezoidalIntegral(OpenDriveMath::fx, 0.0, -a * (this->length - ds), n_integral);
			double y_origin_d = dir * (1 / a) * trapezoidalIntegral(OpenDriveMath::fy, 0.0, a * (this->length - ds), n_integral);
			this->calcAngleOffset(x_origin, y_origin, x_origin_d, y_origin_d);
			for (int i = 0; i <= this->n_vertices; i++) {
				x = (1 / a) * trapezoidalIntegral(OpenDriveMath::fx, 0.0, -a * (this->length - step * i), n_integral);
				y = dir * (1 / a) * trapezoidalIntegral(OpenDriveMath::fy, 0.0, a * (this->length - step * i), n_integral);
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
			double x_origin = (1 / a) * trapezoidalIntegral(OpenDriveMath::fx, 0.0, -a * adjusted_len, n_integral);
			double y_origin = dir_ys * (1 / a) * trapezoidalIntegral(OpenDriveMath::fy, 0.0, a * adjusted_len, n_integral);
			double x_origin_d = (1 / a) * trapezoidalIntegral(OpenDriveMath::fx, 0.0, -a * (adjusted_len - ds), n_integral);
			double y_origin_d = dir_ys * (1 / a) * trapezoidalIntegral(OpenDriveMath::fy, 0.0, a * (adjusted_len - ds), n_integral);
			this->calcAngleOffset(x_origin, y_origin, x_origin_d, y_origin_d);
			for (int i = 0; i <= this->n_vertices; i++) {
				x = (1 / a) * trapezoidalIntegral(OpenDriveMath::fx, 0.0, -a * (adjusted_len - step * i), n_integral);
				y = dir_ys * (1 / a) * trapezoidalIntegral(OpenDriveMath::fy, 0.0, a * (adjusted_len - step * i), n_integral);
				this->transformAndAddVertex(x, y, x_origin, y_origin);
			}

			a = 1 / (sqrt(2 * (1 / abs(this->curvEnd)) * adjusted_len));
			for (int i = 0; i <= this->n_vertices; i++) {
				x = (1 / a) * trapezoidalIntegral(OpenDriveMath::fx, 0.0, a * step * i, n_integral);
				y = dir_ye * (1 / a) * trapezoidalIntegral(OpenDriveMath::fy, 0.0, a * step * i, n_integral);
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
			double ds = 0.1;

			if (reversed) {
				x_origin = (1 / a) * trapezoidalIntegral(OpenDriveMath::fx, 0.0, a * (total_len), n_integral);
				y_origin = direction * (1 / a) * trapezoidalIntegral(OpenDriveMath::fy, 0.0, a * (total_len), n_integral);
				x_origin_d = (1 / a) * trapezoidalIntegral(OpenDriveMath::fx, 0.0, a * (total_len - ds), n_integral);
				y_origin_d = direction * (1 / a) * trapezoidalIntegral(OpenDriveMath::fy, 0.0, a * (total_len - ds), n_integral);
			}
			else {
				x_origin = (1 / a) * trapezoidalIntegral(OpenDriveMath::fx, 0.0, a * (L0), n_integral);
				y_origin = direction * (1 / a) * trapezoidalIntegral(OpenDriveMath::fy, 0.0, a * (L0), n_integral);
				x_origin_d = (1 / a) * trapezoidalIntegral(OpenDriveMath::fx, 0.0, a * (L0 + ds), n_integral);
				y_origin_d = direction * (1 / a) * trapezoidalIntegral(OpenDriveMath::fy, 0.0, a * (L0 + ds), n_integral);
			}
			this->calcAngleOffset(x_origin, y_origin, x_origin_d, y_origin_d);
			for (double i = 0; i <= this->n_vertices; i++) {
				x = (1 / a) * trapezoidalIntegral(OpenDriveMath::fx, 0.0, a * (step * i + L0), n_integral);
				y = direction * (1 / a) * trapezoidalIntegral(OpenDriveMath::fy, 0.0, a * (step * i + L0), n_integral);

				this->transformAndAddVertex(x, y, x_origin, y_origin);
			}
			std::reverse(this->vertices.begin(), this->vertices.end());

		}
	}
	glm::dvec4 generatePosition(double s) {
		//if (s > this->length) s = this->length;
		double  a = 0;
		int n_integral = 1000;
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
				x = (1 / a) * trapezoidalIntegral(OpenDriveMath::fx, 0.0, a * s, n_integral);
				y = dir * (1 / a) * trapezoidalIntegral(OpenDriveMath::fy, 0.0, a * s, n_integral);
				return this->transformAndReturnVertex(x, y, 0, 0);
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
			double x_origin = (1 / a) * trapezoidalIntegral(OpenDriveMath::fx, 0.0, -a * this->length, n_integral);
			double y_origin = dir * (1 / a) * trapezoidalIntegral(OpenDriveMath::fy, 0.0, a * this->length, n_integral);
			double x_origin_d = (1 / a) * trapezoidalIntegral(OpenDriveMath::fx, 0.0, -a * (this->length - ds), n_integral);
			double y_origin_d = dir * (1 / a) * trapezoidalIntegral(OpenDriveMath::fy, 0.0, a * (this->length - ds), n_integral);
			this->calcAngleOffset(x_origin, y_origin, x_origin_d, y_origin_d);
			for (int i = 0; i <= this->n_vertices; i++) {
				x = (1 / a) * trapezoidalIntegral(OpenDriveMath::fx, 0.0, -a * (this->length - s), n_integral);
				y = dir * (1 / a) * trapezoidalIntegral(OpenDriveMath::fy, 0.0, a * (this->length - s), n_integral);
				return this->transformAndReturnVertex(x, y, x_origin, y_origin);
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
			double x_origin = (1 / a) * trapezoidalIntegral(OpenDriveMath::fx, 0.0, -a * adjusted_len, n_integral);
			double y_origin = dir_ys * (1 / a) * trapezoidalIntegral(OpenDriveMath::fy, 0.0, a * adjusted_len, n_integral);
			double x_origin_d = (1 / a) * trapezoidalIntegral(OpenDriveMath::fx, 0.0, -a * (adjusted_len - ds), n_integral);
			double y_origin_d = dir_ys * (1 / a) * trapezoidalIntegral(OpenDriveMath::fy, 0.0, a * (adjusted_len - ds), n_integral);
			this->calcAngleOffset(x_origin, y_origin, x_origin_d, y_origin_d);
			for (int i = 0; i <= this->n_vertices; i++) {
				x = (1 / a) * trapezoidalIntegral(OpenDriveMath::fx, 0.0, -a * (adjusted_len - s), n_integral);
				y = dir_ys * (1 / a) * trapezoidalIntegral(OpenDriveMath::fy, 0.0, a * (adjusted_len - s), n_integral);
				return this->transformAndReturnVertex(x, y, x_origin, y_origin);
			}

			a = 1 / (sqrt(2 * (1 / abs(this->curvEnd)) * adjusted_len));
			for (int i = 0; i <= this->n_vertices; i++) {
				x = (1 / a) * trapezoidalIntegral(OpenDriveMath::fx, 0.0, a * s, n_integral);
				y = dir_ye * (1 / a) * trapezoidalIntegral(OpenDriveMath::fy, 0.0, a * s, n_integral);
				return this->transformAndReturnVertex(x, y, x_origin, y_origin);
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
			double ds = 0.1;

			if (reversed) {
				x_origin = (1 / a) * trapezoidalIntegral(OpenDriveMath::fx, 0.0, a * (total_len), n_integral);
				y_origin = direction * (1 / a) * trapezoidalIntegral(OpenDriveMath::fy, 0.0, a * (total_len), n_integral);
				x_origin_d = (1 / a) * trapezoidalIntegral(OpenDriveMath::fx, 0.0, a * (total_len - ds), n_integral);
				y_origin_d = direction * (1 / a) * trapezoidalIntegral(OpenDriveMath::fy, 0.0, a * (total_len - ds), n_integral);
			}
			else {
				x_origin = (1 / a) * trapezoidalIntegral(OpenDriveMath::fx, 0.0, a * (L0), n_integral);
				y_origin = direction * (1 / a) * trapezoidalIntegral(OpenDriveMath::fy, 0.0, a * (L0), n_integral);
				x_origin_d = (1 / a) * trapezoidalIntegral(OpenDriveMath::fx, 0.0, a * (L0 + ds), n_integral);
				y_origin_d = direction * (1 / a) * trapezoidalIntegral(OpenDriveMath::fy, 0.0, a * (L0 + ds), n_integral);
			}
			this->calcAngleOffset(x_origin, y_origin, x_origin_d, y_origin_d);
			for (double i = 0; i <= this->n_vertices; i++) {
				x = (1 / a) * trapezoidalIntegral(OpenDriveMath::fx, 0.0, a * (s + L0), n_integral);
				y = direction * (1 / a) * trapezoidalIntegral(OpenDriveMath::fy, 0.0, a * (s + L0), n_integral);

				return this->transformAndReturnVertex(x, y, x_origin, y_origin);
			}

		}

	}
};
struct arc :Geometry {
	arc(double _curvature, double _s, double _x, double _y, double _hdg, double _length) :
		curvature(_curvature), Geometry(_s, _x, _y, _hdg, _length) {
		//n_vertices = (int(length) + 1);
	}
	double curvature;
	void generateReferenceLine()override {
		double step = this->length / this->n_vertices;
		for (int i = 0; i <= this->n_vertices; i++) {
			glm::dvec4 position;
			double fi = i * step * this->curvature + this->hdg;
			position.x = (1.0 / this->curvature) * (sin(fi) - sin(this->hdg)) + this->x;
			position.y = (1.0 / this->curvature) * (cos(this->hdg) - cos(fi)) + this->y;
			position.z = 0;
			this->vertices.push_back(position);
		}
	}
	glm::dvec4 generatePosition(double s) {
		//if (s > this->length) s = this->length;
		glm::dvec4 position;
		double fi = s * this->curvature + this->hdg;
		position.x = (1.0 / this->curvature) * (sin(fi) - sin(this->hdg)) + this->x;
		position.y = (1.0 / this->curvature) * (cos(this->hdg) - cos(fi)) + this->y;
		position.z = 0;
		return position;
	}
};
struct poly3 :Geometry {
	poly3(double _a, double _b, double _c, double _d, double _s, double _x, double _y, double _hdg, double _length) :
		a(_a), b(_b), c(_c), d(_d), Geometry(_s, _x, _y, _hdg, _length) {
		n_vertices = 100;
	}
	double a, b, c, d;
	void generateReferenceLine()override {
	}
};
struct paramPoly3 :Geometry {
	paramPoly3(double _aU, double _bU, double _cU, double _dU, double _aV, double _bV, double _cV, double _dV, std::string _pRange
		, double _s, double _x, double _y, double _hdg, double _length) :aU(_aU), bU(_bU), cU(_cU), dU(_dU),
		aV(_aV), bV(_bV), cV(_cV), dV(_dV), Geometry(_s, _x, _y, _hdg, _length) {
		if (_pRange != "arcLength" || _pRange != "normalized") throw new std::exception("Unknow value for pRange:");// +_pRange);
		pRange = _pRange;
		n_vertices = 100;
	}
	double aU, bU, cU, dU, aV, bV, cV, dV;
	std::string pRange;
	void generateReferenceLine()override {

	}
};
struct PlanView {
	std::vector<Geometry*> geometries;
	PlanView(std::vector<Geometry*> _geometries) :geometries(_geometries) {

	}
	~PlanView() {

	}
	/*Geometry* getCurrentGeometry(double s) {
		Geometry* defaultGeometry = nullptr;
		this->getCurrentProp(s, geometries,defaultGeometry);
	}*/
};
namespace Lanes {
	enum class Rule {
		NoPassing,
		Caution,
		none,
	};
	enum class Type {
		shoulder,
		border,
		driving,
		stop,
		none,
		restricted,
		parking,
		median,
		biking,
		sidewalk,
		curb,
		exit,
		entry,
		onramp,
		offRamp,
		connectingRamp
	};
	struct Height {
		double sOffset = 0, inner = 0, outer = 0;
		Height(double _sOffset, double _inner, double _outer) :sOffset(_sOffset), inner(_inner), outer(_outer) {
			if (this->sOffset < 0) throw new std::exception("sOffset cannot be negative.");
		}
		Height() {}
	};
	struct Width {
		double sOffset = 0, a = 0, b = 0, c = 0, d = 0;
		Width(double _sOffset, double _a, double _b, double _c, double _d) :
			sOffset(_sOffset), a(_a), b(_b), c(_c), d(_d) {
			if (this->sOffset < 0) throw new std::exception("sOffset cannot be negative.");
		}
		Width() {}
	};
	struct LaneOffset {
		double s = 0, a = 0, b = 0, c = 0, d = 0;
		LaneOffset(double _s, double _a, double _b, double _c, double _d):
		s(_s),a(_a),b(_b),c(_c),d(_d){
			if (s < 0) throw new std::exception("S cannot be negative");
		}
		LaneOffset() {}
	};
	struct Border {
		double sOffset = 0, a = 0, b = 0, c = 0, d = 0;
		Border(double _sOffset, double _a, double _b, double _c, double _d) :
			sOffset(_sOffset), a(_a), b(_b), c(_c), d(_d) {
			if (this->sOffset < 0) throw new std::exception("sOffset cannot be negative.");
		}
		Border() {}
	};
	struct LaneGeometry {
		std::vector<Border> border;
		std::vector<Width> width;
		LaneGeometry(std::vector<Border> _border, std::vector<Width> _width) :
			border(_border), width(_width) {}
		double getCurrentWidth(double s) {

		}
		LaneGeometry() {}
	};
	struct Predecessor {
		int id;
		Predecessor(int _id) :id(_id) {}
	};
	struct Successor {
		int id;
		Successor(int _id) :id(_id) {}
	};
	struct Link {
		std::vector<Predecessor> predecessor;
		std::vector<Successor> successor;
	};
	struct Lane {
		int id;
		std::string type;
		std::optional<bool> level = std::optional<bool>();
		std::vector<Height> height;
		LaneGeometry laneGeometry;
		std::optional<Link> link;
		bool operator< (const Lane& other) const {
			return id < other.id;
		}

	};
	struct LaneSide {
		std::vector<Lane> lane;
		LaneSide(std::vector<Lane> _lane) :lane(_lane) {

		}
		LaneSide(){}
		virtual ~LaneSide() {}
	};
	struct Left:LaneSide {
		Left(std::vector<Lane> _lane):LaneSide(_lane) {
			for (int i = 0; i < this->lane.size(); i++) {
				if (this->lane.at(i).id < 0) throw new std::exception("Lane ids for left lanes have to be positive !");
			}
		}
	};
	struct Center:LaneSide {
		Center(std::vector<Lane> _lane):LaneSide(_lane) {
			for (int i = 0; i < this->lane.size(); i++) {
				if (this->lane.at(i).id != 0) throw new std::exception("Lane ids for center lanes have to be equal zero !");
			}
		}
		Center() {}
	};
	struct Right:LaneSide {
		
		Right(std::vector<Lane> _lane) :LaneSide(_lane) {
			for (int i = 0; i < this->lane.size(); i++) {
				if (this->lane.at(i).id > 0) throw new std::exception("Lane ids for right lanes have to be negative !");
			}
		}
	};
	struct RoadMark {
		RoadMark() {}
	};
}
struct LaneSection :SBasedProperty {
	double s;
	std::optional<bool> singleSide = std::optional<bool>();
	std::optional<Lanes::Right> right = std::optional<Lanes::Right>();
	std::optional<Lanes::Left>  left = std::optional<Lanes::Left>();
	Lanes::Center center;
	LaneSection(double _s, std::optional<bool> _singleSide, Lanes::Center _center, std::optional<Lanes::Right> _right = std::optional<Lanes::Right>(),
		std::optional<Lanes::Left>  _left = std::optional<Lanes::Left>()) :s(_s), singleSide(_singleSide), center(_center), left(_left), right(_right) {}
	void getCurrentLeftWidth(double s, int index, double& innerDist, double& outerDist) {
		if (index <= 0) {
			throw new std::exception("Index for left lanes has to be positive");
		}
		if (left.has_value()) {
			Lanes::Width defaultWidth;
			int i;
			double accumulativeWidth = 0;
			for (i = 0; i < left.value().lane.size(); i++) {
				Lanes::Width currentWidth = this->getCurrentPropSOffset(s - this->s, left.value().lane.at(i).laneGeometry.width, defaultWidth);
				double ds = s - this->s - currentWidth.sOffset;
				if (index == left.value().lane.at(i).id) {
					//currentWidth = this->getCurrentPropSOffset(s - this->s, left.value().lane.at(i).laneGeometry.width, defaultWidth);
					//ds = s - this->s - currentWidth.sOffset;
					outerDist = accumulativeWidth + currentWidth.a + currentWidth.b * ds + currentWidth.c * std::pow(ds, 2) + currentWidth.d * std::pow(ds, 3);
					innerDist = accumulativeWidth;
					return;
				}
				else {
					accumulativeWidth += currentWidth.a + currentWidth.b * ds + currentWidth.c * std::pow(ds, 2) + currentWidth.d * std::pow(ds, 3);
				}
			}
			innerDist = 0;
			outerDist = 0;
		}
	}
	void getCurrentRightWidth(double s, int index, double& innerDist, double& outerDist) {
		if (index >= 0) {
			throw new std::exception("Index for left lanes has to be neagitve");
		}
		if (right.has_value()) {
			Lanes::Width defaultWidth;
			int i;
			double accumulativeWidth = 0;
			for (i = 0; i < right.value().lane.size(); i++) {
				Lanes::Width currentWidth = this->getCurrentPropSOffset(s - this->s, right.value().lane.at(i).laneGeometry.width, defaultWidth);
				double ds = s - this->s - currentWidth.sOffset;
				//s-ssection - offsetstart = ds
				if (index == right.value().lane.at(i).id) {
					//currentWidth = this->getCurrentPropSOffset(s - this->s, right.value().lane.at(i).laneGeometry.width, defaultWidth);
					//ds = s - this->s - currentWidth.sOffset;
					outerDist = accumulativeWidth + currentWidth.a + currentWidth.b * ds + currentWidth.c * std::pow(ds, 2) + currentWidth.d * std::pow(ds, 3);
					innerDist = accumulativeWidth;
					return;
				}
				else {
					accumulativeWidth += currentWidth.a + currentWidth.b * ds + currentWidth.c * std::pow(ds, 2) + currentWidth.d * std::pow(ds, 3);
				}
			}
			innerDist = 0;
			outerDist = 0;
		}
	}
	void getCurrentWidth(double s, int index, double& innerDist, double& outerDist, Lanes::LaneSide* laneSide) {
		if (Lanes::Right* d = dynamic_cast<Lanes::Right*>(laneSide))
		{
			return this->getCurrentRightWidth(s, index, innerDist, outerDist);
		}
		if (Lanes::Left* d = dynamic_cast<Lanes::Left*>(laneSide))
		{
			return this->getCurrentLeftWidth(s, index, innerDist, outerDist);
		}
	}
	LaneSection() {
		this->s = 0;
		Lanes::Center defaultCenter = Lanes::Center();
	}
};
struct lanes :SBasedProperty {
	std::vector<LaneSection> laneSections;
	lanes(std::vector<LaneSection> _laneSections) :laneSections(_laneSections) {}
	LaneSection getCurrentLaneSection(double s) {
		LaneSection defaultLaneSection = LaneSection();
		return this->getCurrentProp(s, this->laneSections, defaultLaneSection);
	}
};
struct RoadSegment {
	int t_section_n_points = 10; //both sides: left& right
	std::vector<glm::dvec4> vertices;
	std::vector<btVector3> btVertices;
};
struct LineSegment {
	int t_section_n_points = 10;
	std::vector<glm::dvec4> vertices;
	LineSegment(int _t_section_n_points = 10) :t_section_n_points(_t_section_n_points) {}
	LineSegment(std::vector<glm::dvec4> _vertices, int _t_section_n_points = 10) :t_section_n_points(_t_section_n_points),
		vertices(_vertices) {}
};
class Road {
	friend class OpenDriveDocument;
public:
	std::vector<unsigned int> debugIndexes;
	std::vector <glm::dvec4> debugVertices;
	std::vector <glm::dvec4> _vertices;
	std::vector<VerticesObject*> debugLanesRenderObject;
	std::vector<IndexedVerticesObject*> debugEdgeRenderObjects;
	std::vector<IndexedVerticesObject*> lanesRenderObjects;
	static double S_STEP;
	static double T_STEP;
	static bool debugRender;
private:
	double length;
	std::string id;
	std::string junction;
	PlanView planView;
	Link link;
	std::optional<std::string> name;
	std::optional<TrafficRule> rule;
	std::optional<LateralProfile> lateralProfile;
	std::optional<ElevationProfile> elevationProfile;
	std::vector<Type> types;
	lanes roadLanes;
	std::vector<glm::dvec4> referenceLinePoints;
	std::vector<RoadSegment> roadSegments;
	glm::dvec4 grey = glm::dvec4(128. / 255., 128. / 255., 128. / 255., 1.);
	glm::dvec4 red = glm::dvec4(1, 0, 0, 1);
	glm::dvec4 yellow = glm::dvec4(1, 1, 0, 1);
	glm::dvec4 green = glm::dvec4(0, 1, 0, 1);
	glm::dvec4 blue = glm::dvec4(0, 0, 1, 1);
	glm::dvec4 brown = glm::dvec4(135. / 255., 104. / 255., 12. / 255., 1);
	glm::dvec4 dark_brown = glm::dvec4(89. / 255., 64. / 255., 24. / 255., 1);
	glm::dvec4 purple = glm::dvec4(1, 0, 1, 1);


	static float get_random()
	{
		static std::default_random_engine e;
		static std::uniform_real_distribution<> dis(0, 1); // rage 0 - 1
		return dis(e);
	}
	bool isThisConnectionRoad(std::vector<std::string>& connection_road_ids){
		for (auto connection_id : connection_road_ids) {
			if (connection_id == this->id) {
				return true;
			}
		}
		return false;
	}
	void generateLanesVerticesForGeometry(Lanes::LaneSide* laneSide,
		LaneSection laneSection, LaneSection* nextLaneSection, 
		ElevationProfile ep, LateralProfile lp, std::vector<std::string>& connection_road_ids) {
		double g_step_ds = S_STEP;
		double g_step_ds_epsilon = 0.1;
		double g_t_dir_ds = T_STEP;
		for(int i=0;i<laneSide->lane.size();i++){
			Lanes::Lane lane = laneSide->lane.at(i);
			if ((lane.type == "border" || lane.type == "none") && i == laneSide->lane.size()-1 && !this->isThisConnectionRoad(connection_road_ids)) {
				continue;
			}
			glm::dvec3 s_direction;
			for (Geometry* geometry : this->getPlanView().geometries) {
				g_step_ds = S_STEP;
				g_t_dir_ds = T_STEP;
				if (geometry->length <= g_step_ds) {
					g_step_ds = geometry->length / 2;
				}
				std::vector<glm::dvec4> laneVertices;
				std::vector<glm::dvec4> edgesVertices;
				double road_s = geometry->s;
				double current_s = 0;
				double step_ds = g_step_ds;
				double step_ds_epsilon = g_step_ds_epsilon;
				double t_dir_ds = g_t_dir_ds;
				bool finished = false;
				while (!finished) {
					if (road_s < laneSection.s) {
						current_s += step_ds;
						road_s += step_ds;
						continue;
					}
					if (current_s >= geometry->length) {
						step_ds_epsilon = 0.001;
						current_s = geometry->length - step_ds_epsilon;
						road_s = geometry->s + geometry->length - step_ds_epsilon;
						finished = true;
					}
					if (nextLaneSection != nullptr && road_s > nextLaneSection->s) {
						finished = true;
					}
					Elevation elevation = ep.getCurrentElevation(road_s);
					Superelevation superelevation = lp.getCurrentSuperelevation(road_s);
					Elevation elevation_ds = ep.getCurrentElevation(road_s + step_ds_epsilon);
					Superelevation superelevation_ds = lp.getCurrentSuperelevation(road_s + step_ds_epsilon);
					glm::dvec4 position = geometry->generatePosition(current_s);
					glm::dvec4 position_ds = geometry->generatePosition((current_s + step_ds_epsilon));
					double ds_start = road_s - elevation.s;
					position.z = elevation.a + elevation.b * ds_start + elevation.c * std::pow(ds_start, 2) + elevation.d * std::pow(ds_start, 3);
					ds_start = road_s - superelevation.s;
					position.w = superelevation.a + superelevation.b * ds_start + superelevation.c * std::pow(ds_start, 2) + superelevation.d * std::pow(ds_start, 3);
					ds_start = (road_s + step_ds_epsilon) - elevation_ds.s;
					position_ds.z = elevation_ds.a + elevation_ds.b * ds_start + elevation_ds.c * std::pow(ds_start, 2) + elevation_ds.d * std::pow(ds_start, 3);
					ds_start = (road_s + step_ds_epsilon) - superelevation_ds.s;
					position_ds.w = superelevation_ds.a + superelevation_ds.b * ds_start + superelevation_ds.c * std::pow(ds_start, 2) + superelevation_ds.d * std::pow(ds_start, 3);
					glm::dvec3 up = glm::dvec3(0, 0, 1);
					s_direction = glm::dvec3(glm::normalize(position_ds - position));
					glm::dquat superelevation_rotation = glm::angleAxis(position.w, glm::dvec3(s_direction));
					glm::dvec3 t_direction = glm::dvec3(0.0);
					double innerW, outerW;
					if(Lanes::Right* d = dynamic_cast<Lanes::Right*>(laneSide)){
						t_direction = superelevation_rotation * glm::normalize(glm::cross(s_direction, up));
						laneSection.getCurrentRightWidth(road_s, lane.id, innerW, outerW);
					}
					else if (Lanes::Left* d = dynamic_cast<Lanes::Left*>(laneSide)) {
						t_direction = superelevation_rotation * glm::normalize(glm::cross(up,s_direction));
						laneSection.getCurrentLeftWidth(road_s, lane.id, innerW, outerW);
					}
					if (innerW == outerW) {
						current_s += step_ds;
						road_s += step_ds;
						continue;
					}
					double accumulated_width = innerW;
					double width = (outerW - innerW);
					glm::dvec3 middle = ((width / 2.0) + innerW) * glm::dvec3(t_direction) + glm::dvec3(position);
					if (width >= t_dir_ds) {
						t_dir_ds = width / 2;
					}
					while (accumulated_width < outerW) {
						glm::dvec3 pos = (std::min(accumulated_width, outerW)) * t_direction + glm::dvec3(position);
						glm::dvec3 middle_dir = glm::normalize(middle - pos);
						pos += middle_dir * g_step_ds_epsilon;
						laneVertices.push_back(glm::dvec4(pos, 1.0));
						accumulated_width += t_dir_ds *Road::get_random();;
					}

					glm::dvec3 posmax = outerW * t_direction + glm::dvec3(position);
					glm::dvec3 posmin = innerW * t_direction + glm::dvec3(position);
					edgesVertices.push_back(glm::dvec4(posmax, 1.0));
					edgesVertices.push_back(glm::dvec4(posmin, 1.0));
					current_s += step_ds;
					road_s += step_ds;
				}
				this->generateLanesRenderObject(edgesVertices, laneVertices, s_direction,lane);
			}
		}
	}
	void generateLanesRenderObject(std::vector<glm::dvec4>& edgesVertices,
		std::vector<glm::dvec4> laneVertices, glm::dvec3 s_direction, Lanes::Lane lane) {
		if (edgesVertices.size() == 0 || laneVertices.size() == 0){
			return;
		}
		std::vector<unsigned int> laneEdgeIndexes;
		std::vector<CDT::Edge> edges;
		edges.push_back(CDT::Edge(0, 1));
		laneEdgeIndexes.push_back(0);
		laneEdgeIndexes.push_back(1);
		laneEdgeIndexes.push_back(edgesVertices.size() - 1);
		laneEdgeIndexes.push_back(edgesVertices.size() - 2);
		edges.push_back(CDT::Edge(edgesVertices.size() - 1, edgesVertices.size() - 2));
		for (unsigned int i = 0; i < edgesVertices.size() - 2; i += 2) {
			laneEdgeIndexes.push_back(i);
			laneEdgeIndexes.push_back(i + 2);
			edges.push_back(CDT::Edge(i, i + 2));
		}
		for (unsigned int i = 1; i < edgesVertices.size() - 2; i += 2) {
			edges.push_back(CDT::Edge(i, i + 2));
			laneEdgeIndexes.push_back(i);
			laneEdgeIndexes.push_back(i + 2);
		}
		std::vector<CDT::V2d<double>> vertices;

		for (glm::dvec4 ev : edgesVertices) {
			CDT::V2d v = CDT::V2d<double>::make(ev.x, ev.y);
			v.z = ev.z;
			vertices.push_back(v);
		}

		for (glm::dvec4 ev : laneVertices) {
			CDT::V2d v = CDT::V2d<double>::make(ev.x, ev.y);
			v.z = ev.z;
			vertices.push_back(v);
		}
		
		CDT::RemoveDuplicates(vertices);
		//CDT::RemoveDuplicatesAndRemapEdges(vertices, edges);
		std::vector<glm::dvec4> debugVertices;
		for (int i = 0; i < vertices.size(); i++) {
			debugVertices.push_back(glm::dvec4(vertices.at(i).x, vertices.at(i).y, vertices.at(i).z, 1));
		}
		//if(debugRender)
			//debugLanesRenderObject.push_back(new VerticesObject(debugVertices, GL_POINTS, purple));
		std::vector<glm::dvec4> debugLanesVertices;
		for (int i = 0; i < laneEdgeIndexes.size(); i++) {
			debugLanesVertices.push_back(glm::dvec4(vertices.at(laneEdgeIndexes.at(i)).x, vertices.at(laneEdgeIndexes.at(i)).y,
				vertices.at(laneEdgeIndexes.at(i)).z, 1.0));
		}
		if (debugRender)
			debugLanesRenderObject.push_back(new VerticesObject(debugLanesVertices, GL_LINES, glm::dvec4(0, 1, 0, 1)));
		bool triangulate = true;
		if(triangulate){
			using Triangulation = CDT::Triangulation<double>;

			Triangulation cdt =
				Triangulation(CDT::FindingClosestPoint::ClosestRandom, 10);
			try {
				cdt.insertVertices(vertices);
				cdt.insertEdges(edges);
				//cdt.eraseSuperTriangle();
				cdt.eraseOuterTriangles();
			}
			catch (std::exception e) {
				std::cout << e.what() << std::endl;
				cdt.eraseSuperTriangle();
			}
		
			std::vector <glm::dvec4> road_vertices;
			std::vector<unsigned int> roadIndexes;
			for (unsigned int i = 0; i < cdt.vertices.size(); i++) {
				road_vertices.push_back(glm::dvec4(cdt.vertices.at(i).pos.x, cdt.vertices.at(i).pos.y, cdt.vertices.at(i).pos.z, 1));
			}

			for (unsigned int i = 0; i < cdt.triangles.size(); i++) {
				roadIndexes.push_back(cdt.triangles.at(i).vertices.at(0));
				roadIndexes.push_back(cdt.triangles.at(i).vertices.at(1));
				roadIndexes.push_back(cdt.triangles.at(i).vertices.at(2));
			}
			if(lane.type == "restricted"){
				IndexedVerticesObject* ivobj = new IndexedVerticesObject(road_vertices, roadIndexes, GL_TRIANGLES, this->red);
				this->lanesRenderObjects.push_back(ivobj);
			}
			else if (lane.type == "border") {
				IndexedVerticesObject* ivobj = new IndexedVerticesObject(road_vertices, roadIndexes, GL_TRIANGLES, this->brown);
				this->lanesRenderObjects.push_back(ivobj);
			}
			else if (lane.type == "stop") {
				IndexedVerticesObject* ivobj = new IndexedVerticesObject(road_vertices, roadIndexes, GL_TRIANGLES, this->dark_brown);
				this->lanesRenderObjects.push_back(ivobj);
			}
			else if (lane.type == "shoulder") {
				IndexedVerticesObject* ivobj = new IndexedVerticesObject(road_vertices, roadIndexes, GL_TRIANGLES, this->green);
				this->lanesRenderObjects.push_back(ivobj);
			}
			else if (lane.type == "none") {
				IndexedVerticesObject* ivobj = new IndexedVerticesObject(road_vertices, roadIndexes, GL_TRIANGLES, this->purple);
				this->lanesRenderObjects.push_back(ivobj);
			}
			else {
				IndexedVerticesObject* ivobj = new IndexedVerticesObject(road_vertices, roadIndexes, GL_TRIANGLES, glm::dvec4(128. / 255., 128. / 255., 128. / 255., 1.));
				this->lanesRenderObjects.push_back(ivobj);
			}
		}
		if (debugRender){
			IndexedVerticesObject* iobj = new IndexedVerticesObject(edgesVertices, laneEdgeIndexes, GL_LINES);
			this->debugEdgeRenderObjects.push_back(iobj);
		}
		
	}
	void generateRoadLanes(std::vector<glm::dvec4>& reference_line_vertices,
		std::vector<std::string>& connection_road_ids) {
		ElevationProfile ep = this->elevationProfile.value_or(ElevationProfile());
		LateralProfile lp = this->lateralProfile.value_or(LateralProfile());

		for(int k=0;k<this->roadLanes.laneSections.size();k++) {
			LaneSection laneSection = this->roadLanes.laneSections.at(k);
			LaneSection* nextLaneSection = nullptr;
			if (k != roadLanes.laneSections.size() - 1) {
				nextLaneSection = &this->roadLanes.laneSections.at(k + 1);
			}
			if (laneSection.right.has_value()) {
				this->generateLanesVerticesForGeometry(&laneSection.right.value(), laneSection, nextLaneSection, 
					ep, lp, connection_road_ids);
			}
			if (laneSection.left.has_value()) {
				this->generateLanesVerticesForGeometry(&laneSection.left.value(), laneSection, nextLaneSection, 
					ep, lp, connection_road_ids);
			}
		}
	}
	void generateRoad(std::vector<glm::dvec4>& vertices, std::vector<unsigned int>& roadIndexes, std::vector<glm::dvec4>& reference_line_vertices,
		std::vector<glm::dvec4>& road_left_edges, std::vector<glm::dvec4>& road_right_edges, std::vector<std::string>& connection_road_ids) {
		double edge_epsilon = 0.0;
		ElevationProfile ep = this->elevationProfile.value_or(ElevationProfile());
		LateralProfile lp = this->lateralProfile.value_or(LateralProfile());
		double triangulation_step = 1;
		//TODO: Duplicate functionality move to function !
		for (int i = 0; i < this->planView.geometries.size(); i++) {
			double ds = 0;
			Geometry* g = this->planView.geometries.at(i);
			g->generateReferenceLine();
			std::vector<glm::dvec4> referenceLineCoordinates = g->vertices;
			double step_ds = g->length / referenceLineCoordinates.size();
			double epsilon = 0.01;
			double a = -1;
			if (i == this->planView.geometries.size() - 1) {
				for (int j = 0; j < referenceLineCoordinates.size(); j++) {
					double s_start;
					double s_end;
					glm::dvec4 positionStart;
					glm::dvec4 positionEnd;
					if (j == referenceLineCoordinates.size() - 1) {
						s_start = g->s + g->length;
						s_end = g->s + g->length - epsilon;
						positionStart = referenceLineCoordinates.at(j);
						positionEnd = g->generatePosition(g->length - epsilon);
					}
					else if (j==0) {
						s_start = g->s;
						s_end = g->s+epsilon;
						positionStart = referenceLineCoordinates.at(0);
						positionEnd = g->generatePosition(epsilon);
					}
					else {
						s_start = g->s + j * step_ds;
						s_end = g->s + (j + 1) * step_ds;
						positionStart = referenceLineCoordinates.at(j);
						positionEnd = referenceLineCoordinates.at(j + 1);
					}

					Elevation elevation_start = ep.getCurrentElevation(s_start);
					Elevation elevation_end = ep.getCurrentElevation(s_end);
					Superelevation superelevation_start = lp.getCurrentSuperelevation(s_start);
					Superelevation superelevation_end = lp.getCurrentSuperelevation(s_end);
					//a + b*ds + c*ds² + d*ds³
					double ds_start = s_start - elevation_start.s;
					double ds_end = s_end - elevation_end.s;

					positionStart.z = elevation_start.a + elevation_start.b * ds_start + elevation_start.c * std::pow(ds_start, 2) + elevation_start.d * std::pow(ds_start, 3);
					ds_start = s_start - superelevation_start.s;
					positionStart.w = superelevation_start.a + superelevation_start.b * ds_start + superelevation_start.c * std::pow(ds_start, 2) + superelevation_start.d * std::pow(ds_start, 3);

					positionEnd.z = elevation_end.a + elevation_end.b * ds_end + elevation_end.c * std::pow(ds_end, 2) + elevation_end.d * std::pow(ds_end, 3);
					ds_end = s_end - superelevation_end.s;
					positionEnd.w = superelevation_end.a + superelevation_end.b * ds_end + superelevation_end.c * std::pow(ds_end, 2) + superelevation_end.d * std::pow(ds_end, 3);
					referenceLinePoints.push_back(positionStart);
					glm::dvec3 up = glm::dvec3(0, 0, 1);
					glm::dvec3 s_direction = glm::dvec3(glm::normalize(positionEnd - positionStart));
					if (j == referenceLineCoordinates.size() - 1) {
						s_direction *= -1;
					}
					glm::dquat superelevation_rotation = glm::angleAxis(positionStart.w, glm::dvec3(s_direction));
					glm::dvec3 left_direction = superelevation_rotation * glm::normalize(glm::cross(up, s_direction));
					glm::dvec3 right_direction = superelevation_rotation * glm::normalize(glm::cross(s_direction, up));
					LaneSection laneSection = this->roadLanes.getCurrentLaneSection(s_start);
					if (laneSection.right.has_value() && laneSection.right.value().lane.size() > 0) {
						Lanes::Right right = laneSection.right.value();
						for (int i = 0; i < right.lane.size(); i++) {
							double innerW, outerW;
							laneSection.getCurrentRightWidth(s_start, right.lane.at(i).id, innerW, outerW);
							glm::dvec3 innerPos = glm::dvec3(positionStart) + right_direction * innerW;
							glm::dvec3 outerPos = glm::dvec3(positionStart) + right_direction * outerW;
							LineSegment lineSegment = LineSegment();
							int n_points = int((outerW - innerW) / triangulation_step + 1);
							double t_ds = (outerW - innerW) / n_points;
							glm::dvec3 t_dir = right_direction;// glm::normalize(outerPos - innerPos);

							if (right.lane.size() - 1 == i)
								road_right_edges.push_back(glm::dvec4(innerPos + t_dir * t_ds * double(n_points + edge_epsilon), 0.0));
							for (int i = 1; i < n_points; i++) {
								vertices.push_back(glm::dvec4(innerPos + t_dir * t_ds * double(i), 0.0));
							}

						}
					}
					else {
						road_right_edges.push_back(positionStart);
					}
					if (laneSection.left.has_value() && laneSection.left.value().lane.size()>0) {
						Lanes::Left left = laneSection.left.value();
						for (int i = 0; i < left.lane.size(); i++) {
							double innerW, outerW;
							laneSection.getCurrentLeftWidth(s_start, left.lane.at(i).id, innerW, outerW);
							glm::dvec3 innerPos = glm::dvec3(positionStart) + left_direction * innerW;
							glm::dvec3 outerPos = glm::dvec3(positionStart) + left_direction * outerW;
							LineSegment lineSegment = LineSegment();
							int n_points = int((outerW - innerW) / triangulation_step + 1);
							double t_ds = (outerW - innerW) / n_points;
							glm::dvec3 t_dir = left_direction;// glm::normalize(outerPos - innerPos);

							if (i == left.lane.size() - 1)
								road_left_edges.push_back(glm::dvec4(innerPos + t_dir * t_ds * double(n_points + edge_epsilon), 0.0));
							for (int i = 1; i < n_points; i++) {
								vertices.push_back(glm::dvec4(innerPos + t_dir * t_ds * double(i), 0.0));
							}

						}
					}
					else {
						road_left_edges.push_back(positionStart);
					}
				}
			}
			else {
				for (int j = 0; j < referenceLineCoordinates.size() - 1; j++) {
					double s_start;
					double s_end;
					glm::dvec4 positionStart;
					glm::dvec4 positionEnd;
					if (j == 0) {
						s_start = g->s;
						s_end = g->s+epsilon;
						positionStart = referenceLineCoordinates.at(0);
						positionEnd = g->generatePosition(epsilon);
					}
					else {
						s_start = g->s + j * step_ds;
						s_end = g->s + (j + 1) * step_ds;
						positionStart = referenceLineCoordinates.at(j);
						positionEnd = referenceLineCoordinates.at(j + 1);
					}

					Elevation elevation_start = ep.getCurrentElevation(s_start);
					Elevation elevation_end = ep.getCurrentElevation(s_end);
					Superelevation superelevation_start = lp.getCurrentSuperelevation(s_start);
					Superelevation superelevation_end = lp.getCurrentSuperelevation(s_end);
					//a + b*ds + c*ds² + d*ds³
					double ds_start = s_start - elevation_start.s;
					double ds_end = s_end - elevation_end.s;

					positionStart.z = elevation_start.a + elevation_start.b * ds_start + elevation_start.c * std::pow(ds_start, 2) + elevation_start.d * std::pow(ds_start, 3);
					ds_start = s_start - superelevation_start.s;
					positionStart.w = superelevation_start.a + superelevation_start.b * ds_start + superelevation_start.c * std::pow(ds_start, 2) + superelevation_start.d * std::pow(ds_start, 3);

					positionEnd.z = elevation_end.a + elevation_end.b * ds_end + elevation_end.c * std::pow(ds_end, 2) + elevation_end.d * std::pow(ds_end, 3);
					ds_end = s_end - superelevation_end.s;
					positionEnd.w = superelevation_end.a + superelevation_end.b * ds_end + superelevation_end.c * std::pow(ds_end, 2) + superelevation_end.d * std::pow(ds_end, 3);
					referenceLinePoints.push_back(positionStart);
					glm::dvec3 up = glm::dvec3(0, 0, 1);
					glm::dvec3 s_direction = glm::dvec3(glm::normalize(positionEnd - positionStart));
					if (j == referenceLineCoordinates.size() - 1) {
						s_direction *= -1;
					}
					glm::dquat superelevation_rotation = glm::angleAxis(positionStart.w, glm::dvec3(s_direction));
					glm::dvec3 left_direction = superelevation_rotation * glm::normalize(glm::cross(up, s_direction));
					glm::dvec3 right_direction = superelevation_rotation * glm::normalize(glm::cross(s_direction, up));
					LaneSection laneSection = this->roadLanes.getCurrentLaneSection(s_start);
					if (laneSection.right.has_value() && laneSection.right.value().lane.size()>0) {
						Lanes::Right right = laneSection.right.value();
						for (int i = 0; i < right.lane.size(); i++) {
							double innerW, outerW;
							laneSection.getCurrentRightWidth(s_start, right.lane.at(i).id, innerW, outerW);
							if (innerW == outerW) {
								road_right_edges.push_back(positionStart);
								continue;
							}
							glm::dvec3 innerPos = glm::dvec3(positionStart) + right_direction * innerW;
							glm::dvec3 outerPos = glm::dvec3(positionStart) + right_direction * outerW;
							LineSegment lineSegment = LineSegment();
							int n_points = int((outerW - innerW) / triangulation_step + 1);
							double t_ds = (outerW - innerW) / n_points;
							glm::dvec3 t_dir = right_direction;// glm::normalize(outerPos - innerPos);

							if (right.lane.size() - 1 == i)
								road_right_edges.push_back(glm::dvec4(innerPos + t_dir * t_ds * double(n_points + edge_epsilon), 0.0));
							for (int i = 1; i < n_points; i++) {
								vertices.push_back(glm::dvec4(innerPos + t_dir * t_ds * double(i), 0.0));
							}

						}
					}
					else {
						road_right_edges.push_back(positionStart);
					}
					if (laneSection.left.has_value() && laneSection.left.value().lane.size()>0) {
						Lanes::Left left = laneSection.left.value();
						for (int i = 0; i < left.lane.size(); i++) {
							double innerW, outerW;
							laneSection.getCurrentLeftWidth(s_start, left.lane.at(i).id, innerW, outerW);
							if (innerW == outerW) {
								road_left_edges.push_back(positionStart);
								continue;
							}
							glm::dvec3 innerPos = glm::dvec3(positionStart) + left_direction * innerW;
							glm::dvec3 outerPos = glm::dvec3(positionStart) + left_direction * outerW;
							LineSegment lineSegment = LineSegment();
							int n_points = int((outerW - innerW) / triangulation_step + 1);
							double t_ds = (outerW - innerW) / n_points;
							glm::dvec3 t_dir = left_direction;// glm::normalize(outerPos - innerPos);

							if (i == left.lane.size() - 1)
								road_left_edges.push_back(glm::dvec4(innerPos + t_dir * t_ds * double(n_points + edge_epsilon), 0.0));
							for (int i = 1; i < n_points; i++) {
								vertices.push_back(glm::dvec4(innerPos + t_dir * t_ds * double(i), 0.0));
							}

						}
					}
					else {
						road_left_edges.push_back(positionStart);
					}
				}
			}
		}
		this->_vertices = vertices;
	}

	void generateReferenceLineCoordinates() {
		ElevationProfile ep = this->elevationProfile.value_or(ElevationProfile());
		LateralProfile lp = this->lateralProfile.value_or(LateralProfile());
		for (int i = 0; i < this->planView.geometries.size(); i++) {
			double ds = 0;
			Geometry* g = this->planView.geometries.at(i);
			g->generateReferenceLine();
			std::vector<glm::dvec4> referenceLineCoordinates = g->vertices;
			double step_ds = g->length / referenceLineCoordinates.size();
			for (int j = 0; j < referenceLineCoordinates.size(); j++) {
				double s = g->s + j * step_ds;
				glm::dvec4 pos = referenceLineCoordinates.at(j);
				Elevation elevation = ep.getCurrentElevation(s);
				Superelevation superelevation = lp.getCurrentSuperelevation(s);
				//a + b*ds + c*ds² + d*ds³
				double ds = s - elevation.s;
				pos.z = elevation.a + elevation.b * ds + elevation.c * std::pow(ds, 2) + elevation.d * std::pow(ds, 3);
				ds = s - superelevation.s;
				pos.w = superelevation.a + superelevation.b * ds + superelevation.c * std::pow(ds, 2) + superelevation.d * std::pow(ds, 3);
				//referenceLinePoints.push_back(pos);
			}
		}
	}

public:
	Road(double _length, std::string _id, std::string _junction,
		std::optional<std::string> _name, std::optional<TrafficRule> _rule, PlanView _planView,
		Link _link, std::vector<Type> _types, lanes _lanes, std::optional<LateralProfile> _lateralProfile = std::optional<LateralProfile>(),
		std::optional<ElevationProfile> _elevationProfile = std::optional<ElevationProfile>()) :
		length(_length), id(_id), junction(_junction), name(_name), rule(_rule), planView(_planView),
		link(_link), types(_types), roadLanes(_lanes), lateralProfile(_lateralProfile), elevationProfile(_elevationProfile) {}
	~Road() {}
	PlanView getPlanView() {
		return this->planView;
	}
	std::vector<glm::dvec4> getReferencePoints() {
		return this->referenceLinePoints;
	}
};
class OpenDriveDocument
{
private:
	
	tinyxml2::XMLDocument doc;
	static std::mutex g_cout_mutex;
	Link createLink(tinyxml2::XMLElement* xml_road);
	template <class T>
	void populateCessor(tinyxml2::XMLElement* xml_cessor, T& cessor);
	void populateGeometries(tinyxml2::XMLElement* xml_road, std::vector<Geometry*>& geometries);
	void populateTypes(tinyxml2::XMLElement* xml_road, std::vector<Type>& types);
	void populateSuperelevations(tinyxml2::XMLElement* xml_road, std::vector<Superelevation>& sueperelevations);
	void populateShapes(tinyxml2::XMLElement* xml_road, std::vector<Shape>& shapes);
	void populateElevations(tinyxml2::XMLElement* xml_road, std::vector<Elevation>& elevations);
	void populateLaneSections(tinyxml2::XMLElement* xml_road, std::vector<LaneSection>& laneSections);
	void populateLane(tinyxml2::XMLElement* xml_lanesection, Lanes::Lane& lane);
	void switchLaneType(std::string s_type, Lanes::Type& type);
	template <class T>
	void populateGenericPolynomialTag(tinyxml2::XMLElement* xml_tag, T& polynomialStruct);
	template <class T>
	void populateGenericPolynomialTags(tinyxml2::XMLElement* xml_tag, std::string childTagName, std::vector<T>& polynomialStructs);
	void populateConnectionRoads() //TODO: this is temporary function !!! Consider removing it and implement proper junction parsing
	{
		tinyxml2::XMLElement* xml_junction = this->doc.FirstChildElement()->FirstChildElement("junction");
		while (xml_junction != nullptr) {
			tinyxml2::XMLElement* xml_connection = xml_junction->FirstChildElement("connection");
			while (xml_connection != nullptr) {
				std::string connection_road_id = std::string(xml_connection->Attribute("connectingRoad"));
				this->connection_road_ids.push_back(connection_road_id);
				xml_connection = xml_connection->NextSiblingElement("connection");
			}
			xml_junction = xml_junction->NextSiblingElement("junction");
		}
	}
public:
	std::vector<Road> roads;
	std::vector <glm::dvec4> reference_line_vertices;
	std::vector<IndexedVerticesObject*> roadRenderObjects;
	std::vector<IndexedVerticesObject*> LanesRenderObjects;
	std::vector<IndexedVerticesObject*> indexedDebug;
	std::vector<VerticesObject*> debug;
	static std::vector<std::string> connection_road_ids;
	OpenDriveDocument(std::string filePath);
	void generateReferenceLines();
	static void GenerateRoad(int ii, Road* road, std::vector <glm::dvec4>* reference_line_vertices, std::vector<std::string>* connection_road_ids,
		std::vector<IndexedVerticesObject*>* roadRenderObjects, std::vector<IndexedVerticesObject*>* indexedDebug = nullptr, 
		std::vector<VerticesObject*>* debug = nullptr) {
		g_cout_mutex.lock();
		std::cout << "Starting thread for road id " << road->id << std::endl;
		g_cout_mutex.unlock();
		using clock = std::chrono::system_clock;
		using sec = std::chrono::duration<double>;
		auto before = clock::now();
		bool triangulate = true;
		double dist_epsilon = 0.1;
		std::vector <glm::dvec4> road_vertices;
		std::vector<unsigned int> roadIndexes;
		std::vector<unsigned int> debugIndexes;
		std::vector <glm::dvec4> debugVertices;
		std::vector<glm::dvec4> road_left_edge;
		std::vector<glm::dvec4> road_right_edge;
		
		road->generateRoad(road_vertices, roadIndexes, *reference_line_vertices, road_left_edge, road_right_edge,
			*connection_road_ids);
		sec duration = clock::now() - before;
		before = clock::now();
		std::vector<CDT::V2d<double>> vertices;
		std::vector<CDT::Edge> edges;
		for (int i = 0; i < road_vertices.size(); i++) {
			CDT::V2d v = CDT::V2d<double>::make(road_vertices.at(i).x, road_vertices.at(i).y);
			v.z = road_vertices.at(i).z;
			vertices.push_back(v);
		}

		//road_vertices.clear();
		int left_start, left_end, right_start, right_end;
		left_start = vertices.size();
		edges.push_back(CDT::Edge(left_start, vertices.size() + 1));
		for (int i = 0; i < road_left_edge.size(); i++) {
			CDT::V2d v = CDT::V2d<double>::make(road_left_edge.at(i).x, road_left_edge.at(i).y);
			v.z = road_left_edge.at(i).z;
			vertices.push_back(v);
			if (i > 1) {
				edges.push_back(CDT::Edge(vertices.size() - 2, vertices.size() - 1));
			}
		}
		left_end = vertices.size() - 1;
		right_start = vertices.size();
		edges.push_back(CDT::Edge(right_start, vertices.size() + 1));
		for (int i = 0; i < road_right_edge.size(); i++) {
			CDT::V2d v = CDT::V2d<double>::make(road_right_edge.at(i).x, road_right_edge.at(i).y);
			v.z = road_right_edge.at(i).z;
			vertices.push_back(v);
			if (i > 1) {
				edges.push_back(CDT::Edge(vertices.size() - 2, vertices.size() - 1));
			}
		}
		right_end = vertices.size() - 1;

		edges.push_back(CDT::Edge(left_start, right_start));
		edges.push_back(CDT::Edge(left_end, right_end));
		if(debug!=nullptr){
			VerticesObject* vobj = new VerticesObject(road_vertices, GL_POINTS);
			debug->push_back(vobj);
		}
		std::vector<glm::dvec4> edgesVertices;
		if (indexedDebug != nullptr) {
			
			for (auto v : vertices) {
				edgesVertices.push_back(glm::dvec4(v.x, v.y, v.z,1));
			}
			std::vector<unsigned int> edgesIndexes;
			for (auto edge : edges) {
				edgesIndexes.push_back(edge.v1());
				edgesIndexes.push_back(edge.v2());
			}
			IndexedVerticesObject* ivobj = new IndexedVerticesObject(edgesVertices, edgesIndexes, GL_LINES);
			indexedDebug->push_back(ivobj);
		}
		using Triangulation = CDT::Triangulation<double>;

		Triangulation cdt =
			Triangulation(CDT::FindingClosestPoint::ClosestRandom, 10);
		
		try {
			cdt.insertVertices(vertices);
			cdt.insertEdges(edges);
			cdt.eraseOuterTriangles();
		}
		catch (std::exception e) {
			std::cout << e.what() << std::endl;
			cdt.eraseSuperTriangle();
		}

		road_vertices.clear();
		for (int i = 0; i < cdt.vertices.size(); i++) {
			road_vertices.push_back(glm::dvec4(cdt.vertices.at(i).pos.x, cdt.vertices.at(i).pos.y, cdt.vertices.at(i).pos.z, 1));
		}

		for (int i = 0; i < cdt.triangles.size(); i++) {
			roadIndexes.push_back(cdt.triangles.at(i).vertices.at(0));
			roadIndexes.push_back(cdt.triangles.at(i).vertices.at(1));
			roadIndexes.push_back(cdt.triangles.at(i).vertices.at(2));
		}
		
		
		IndexedVerticesObject* iobj = new IndexedVerticesObject(road_vertices, roadIndexes, GL_TRIANGLES);
		roadRenderObjects->push_back(iobj);

		duration = clock::now() - before;
		g_cout_mutex.lock();
		std::cout << "Thread for road id " <<road->id<<" finished in " << duration.count() << "s" << std::endl;
		g_cout_mutex.unlock();
	}
	static void GenerateRoadLanes(int ii, Road* road, std::vector<glm::dvec4>* reference_line_vertices) {
		g_cout_mutex.lock();
		std::cout << "Starting thread for road id " << road->id << std::endl;
		g_cout_mutex.unlock();
		using clock = std::chrono::system_clock;
		using sec = std::chrono::duration<double>;
		auto before = clock::now();
		road->generateRoadLanes(*reference_line_vertices,OpenDriveDocument::connection_road_ids);
		sec duration = clock::now() - before;
		g_cout_mutex.lock();
		std::cout << "Thread for road id " << road->id << " finished in " << duration.count() << "s" << std::endl;
		g_cout_mutex.unlock();
	}
	void generateRoads() {
		std::vector<std::thread*> _threads;
		int i = 0;
		for (i; i < this->roads.size(); i++) {
			_threads.push_back(new std::thread(OpenDriveDocument::GenerateRoad, i, 
				&roads.at(i),&reference_line_vertices,&connection_road_ids,&roadRenderObjects,&this->indexedDebug,&this->debug));
		}
		for (auto t : _threads) {
			t->join();
		}
		for (auto t : _threads) {
			delete t;
		}
	}
	void GenerateRoadsLanes() {
		std::vector<std::thread*> _threads;
		int i = 0;
		for (i; i < this->roads.size(); i++) {
			_threads.push_back(new std::thread(OpenDriveDocument::GenerateRoadLanes, i,
				&roads.at(i), &reference_line_vertices));
		}
		for (auto t : _threads) {
			t->join();
		}
		for (auto t : _threads) {
			delete t;
		}
		
	}
	void printOpenDriveDocument();
	std::vector<Road> getRoads();
	int parseOpenDriveDocument();

};
#endif 


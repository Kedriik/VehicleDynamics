#include "OpenDriveDocument.h"

OpenDriveDocument::OpenDriveDocument(std::string filePath) {
	std::ifstream myfile;
	doc.LoadFile(filePath.c_str());
	this->parsePlanView(this->doc);
}
void OpenDriveDocument::generateReferenceLines() {
	for (int i = 0; i < this->roads.size(); i++) {
		Road r = roads.at(i);
		for (int j = 0; j < r.planView.geometries.size(); j++) {
			Geometry* g = r.planView.geometries.at(j);
			g->generateReferenceLine();
		}

	}
}
std::vector<Road> OpenDriveDocument::getRoads() {
	return this->roads;
}

int OpenDriveDocument::parsePlanView(tinyxml2::XMLDocument& doc) {
	tinyxml2::XMLElement* xml_road = doc.FirstChildElement()->FirstChildElement("road");
	double road_length;
	std::string road_id, road_junction;
	std::optional<std::string> road_name;
	std::optional<TrafficRule> road_rule;

	tinyxml2::XMLError error = xml_road->QueryDoubleAttribute("length", &road_length);
	road_id = std::string(xml_road->Attribute("id"));
	road_junction = std::string(xml_road->Attribute("junction"));
	if (xml_road->Attribute("name") != NULL) {
		road_name = std::string(xml_road->Attribute("name"));
	}

	if (xml_road->Attribute("rule") != NULL) {
		std::string sroad_rule = std::string(xml_road->Attribute("rule"));
	};
	//error = xml_road->QueryStringAttribute("junction", &road_junction);
	//Road road = Road();

	while (xml_road != nullptr) {
		tinyxml2::XMLElement* geometry = xml_road->FirstChildElement("planView")->FirstChildElement("geometry");
		std::vector<Geometry*> geometries;
		this->populateGeometries(geometries, geometry);
		PlanView planView = PlanView(geometries);
		Road road = Road(road_length, road_id, road_junction, road_name, road_rule, planView);
		this->roads.push_back(road);
		xml_road = xml_road->NextSiblingElement("road");
	}

	return 0;
}
void OpenDriveDocument::populateGeometries(std::vector<Geometry*>& geometries, tinyxml2::XMLElement* geometry)
{
	double s, x, y, hdg, length;
	while (geometry != nullptr) {

		Geometry* Geometry = nullptr;
		tinyxml2::XMLError error = geometry->QueryDoubleAttribute("s", &s);
		error = geometry->QueryDoubleAttribute("x", &x);
		error = geometry->QueryDoubleAttribute("y", &y);
		error = geometry->QueryDoubleAttribute("hdg", &hdg);
		error = geometry->QueryDoubleAttribute("length", &length);
		const char* v = geometry->FirstChildElement()->Value();
		if (std::string(v) == "line") {

			Geometry = new line(s, x, y, hdg, length);
			Geometry->name = "Line";
		}
		else if (std::string(v) == "arc") {
			double curvature;
			error = geometry->FirstChildElement()->QueryDoubleAttribute("curvature", &curvature);
			Geometry = new arc(curvature, s, x, y, hdg, length);
			Geometry->name = "Arc";
		}
		else if (std::string(v) == "spiral") {
			double curvStart, curvEnd;
			error = geometry->FirstChildElement()->QueryDoubleAttribute("curvStart", &curvStart);
			error = geometry->FirstChildElement()->QueryDoubleAttribute("curvEnd", &curvEnd);
			Geometry = new spiral(curvStart, curvEnd, s, x, y, hdg, length);
			Geometry->name = "Spiral";
		}
		geometries.push_back(Geometry);
		geometry = geometry->NextSiblingElement("geometry");
	}
}
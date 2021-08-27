#include "OpenDriveDocument.h"
std::mutex OpenDriveDocument::g_cout_mutex;
std::vector<std::string> OpenDriveDocument::connection_road_ids;
double Road::S_STEP;
double Road::T_STEP;
bool Road::debugRender;
OpenDriveDocument::OpenDriveDocument(std::string filePath) {
	std::ifstream myfile;
	doc.LoadFile(filePath.c_str());
}
void OpenDriveDocument::generateReferenceLines() {
	for (int i = 0; i < this->roads.size(); i++) {
		roads.at(i).generateReferenceLineCoordinates();
	}
}
std::vector<Road> OpenDriveDocument::getRoads() {
	return this->roads;
}
int OpenDriveDocument::parseOpenDriveDocument() {
	this->populateConnectionRoads();
	tinyxml2::XMLElement* xml_road = this->doc.FirstChildElement()->FirstChildElement("road");
	double road_length;
	std::string road_id, road_junction;
	std::optional<std::string> road_name;
	std::optional<TrafficRule> road_rule;
	tinyxml2::XMLError error;
	while (xml_road != nullptr) {
		road_id = std::string(xml_road->Attribute("id"));
		road_junction = std::string(xml_road->Attribute("junction"));
		if (xml_road->Attribute("name") != NULL) {
			road_name = std::string(xml_road->Attribute("name"));
		}

		if (xml_road->Attribute("rule") != NULL) {
			std::string sroad_rule = std::string(xml_road->Attribute("rule"));
		};
		error = xml_road->QueryDoubleAttribute("length", &road_length);
		
		std::vector<Geometry*> geometries;
		std::vector<Type> types;
		std::vector<Elevation> elevations;
		std::vector<Superelevation> superelevations;
		std::vector<Shape> shapes;
		std::vector<LaneSection> laneSections;
		
		this->populateGeometries(xml_road,geometries);
		PlanView planView = PlanView(geometries);
		Link link = this->createLink(xml_road);
		this->populateTypes(xml_road,types);
		this->populateElevations(xml_road, elevations);
		this->populateSuperelevations(xml_road, superelevations);
		this->populateShapes(xml_road, shapes);
		this->populateLaneSections(xml_road, laneSections);
		lanes roadLanes = lanes(laneSections);
		ElevationProfile elevationProfile(elevations);
		LateralProfile lateralProfile(shapes, superelevations);
		Road road(road_length, road_id, road_junction, road_name, road_rule, planView, link, types,
			roadLanes,lateralProfile, elevationProfile);
		this->roads.push_back(road);
		xml_road = xml_road->NextSiblingElement("road");
	}

	return 0;
}
void OpenDriveDocument::populateTypes(tinyxml2::XMLElement* xml_road, std::vector<Type>& types)
{
	tinyxml2::XMLElement* xml_type = xml_road->FirstChildElement("type");
	while (xml_type != nullptr) {
		double s;
		std::string type, sCountryCode;
		std::optional<CountryCode> countryCode;
		xml_type->QueryDoubleAttribute("s", &s);
		type = std::string(xml_type->Attribute("type"));
		if (xml_type->Attribute("country") != NULL) {
			sCountryCode = std::string(xml_type->Attribute("country"));
			if (sCountryCode == "PL") {
				countryCode = CountryCode::PL;
			}
			else if (sCountryCode == "DE") {
				countryCode = CountryCode::DE;
			}
			else {
				throw "Unsupported country code !";
			}
		}
		Type t = Type(s, type, countryCode);
		types.push_back(t);
		xml_type = xml_type->NextSiblingElement("type");
	}
}
void OpenDriveDocument::populateSuperelevations(tinyxml2::XMLElement* xml_road, std::vector<Superelevation>& superelevations)
{
	tinyxml2::XMLElement* xml_lateralProfile = xml_road->FirstChildElement("lateralProfile");
	if (xml_lateralProfile == nullptr) return;
	tinyxml2::XMLElement* xml_superElevation = xml_lateralProfile->FirstChildElement("superelevation");
	while (xml_superElevation != nullptr) {
		double s, a, b, c, d;
		if (xml_superElevation->QueryDoubleAttribute("s", &s) != tinyxml2::XMLError::XML_SUCCESS) throw "Something wrong with superelevation";
		if (xml_superElevation->QueryDoubleAttribute("a", &a) != tinyxml2::XMLError::XML_SUCCESS) throw "Something wrong with superelevation";
		if (xml_superElevation->QueryDoubleAttribute("b", &b) != tinyxml2::XMLError::XML_SUCCESS) throw "Something wrong with superelevation";
		if (xml_superElevation->QueryDoubleAttribute("c", &c) != tinyxml2::XMLError::XML_SUCCESS) throw "Something wrong with superelevation";
		if (xml_superElevation->QueryDoubleAttribute("d", &d) != tinyxml2::XMLError::XML_SUCCESS) throw "Something wrong with superelevation";
		Superelevation superelevation = Superelevation(s, a, b, c, d);
		superelevations.push_back(superelevation);
		xml_superElevation = xml_superElevation->NextSiblingElement("superelevation");
	}
}
void OpenDriveDocument::populateShapes(tinyxml2::XMLElement* xml_road, std::vector<Shape>& shapes)
{
	tinyxml2::XMLElement* xml_lateralProfile = xml_road->FirstChildElement("lateralProfile");
	if (xml_lateralProfile == nullptr) return;
	tinyxml2::XMLElement* xml_shape = xml_lateralProfile->FirstChildElement("shape");
	while (xml_shape != nullptr) {
		double s,t, a, b, c, d;
		if (xml_shape->QueryDoubleAttribute("s", &s) != tinyxml2::XMLError::XML_SUCCESS) throw "Something wrong with superelevation";
		if (xml_shape->QueryDoubleAttribute("t", &t) != tinyxml2::XMLError::XML_SUCCESS) throw "Something wrong with superelevation";
		if (xml_shape->QueryDoubleAttribute("a", &a) != tinyxml2::XMLError::XML_SUCCESS) throw "Something wrong with superelevation";
		if (xml_shape->QueryDoubleAttribute("b", &b) != tinyxml2::XMLError::XML_SUCCESS) throw "Something wrong with superelevation";
		if (xml_shape->QueryDoubleAttribute("c", &c) != tinyxml2::XMLError::XML_SUCCESS) throw "Something wrong with superelevation";
		if (xml_shape->QueryDoubleAttribute("d", &d) != tinyxml2::XMLError::XML_SUCCESS) throw "Something wrong with superelevation";
		Shape shape = Shape(s, t, a, b, c, d);
		shapes.push_back(shape);
		xml_shape = xml_shape->NextSiblingElement("shape");
	}
}
void OpenDriveDocument::populateElevations(tinyxml2::XMLElement* xml_road, std::vector<Elevation>& elevations)
{
	tinyxml2::XMLElement* xml_elevationProfile = xml_road->FirstChildElement("elevationProfile");
	if (xml_elevationProfile == nullptr) return;
	tinyxml2::XMLElement* xml_elevation = xml_elevationProfile->FirstChildElement("elevation");
	while (xml_elevation != nullptr) {
		double s, a, b, c, d;
		if (xml_elevation->QueryDoubleAttribute("s", &s) != tinyxml2::XMLError::XML_SUCCESS) throw "Something wrong with superelevation";
		if (xml_elevation->QueryDoubleAttribute("a", &a) != tinyxml2::XMLError::XML_SUCCESS) throw "Something wrong with superelevation";
		if (xml_elevation->QueryDoubleAttribute("b", &b) != tinyxml2::XMLError::XML_SUCCESS) throw "Something wrong with superelevation";
		if (xml_elevation->QueryDoubleAttribute("c", &c) != tinyxml2::XMLError::XML_SUCCESS) throw "Something wrong with superelevation";
		if (xml_elevation->QueryDoubleAttribute("d", &d) != tinyxml2::XMLError::XML_SUCCESS) throw "Something wrong with superelevation";
		Elevation elevation = Elevation(s, a, b, c, d);
		elevations.push_back(elevation);
		xml_elevation = xml_elevation->NextSiblingElement("elevation");
	}
}
void OpenDriveDocument::populateLaneSections(tinyxml2::XMLElement* xml_road, std::vector<LaneSection>& laneSections)
{
	tinyxml2::XMLElement* xml_lanes = xml_road->FirstChildElement("lanes");
	if (xml_lanes == nullptr) return;
	tinyxml2::XMLElement* xml_lanesection = xml_lanes->FirstChildElement("laneSection");
	while (xml_lanesection != nullptr) {
		double s;
		bool singleSide;
		if (xml_lanesection->QueryDoubleAttribute("s", &s) != tinyxml2::XMLError::XML_SUCCESS) throw "Something wrong with 's' in lane section";
		xml_lanesection->QueryBoolAttribute("singleSide", &singleSide);
		tinyxml2::XMLElement* xml_center = xml_lanesection->FirstChildElement("center");
		std::vector<Lanes::Lane> lanes;
		if (xml_center == nullptr) throw "No center lane in lane section";
		else {
			tinyxml2::XMLElement* xml_lane = xml_center->FirstChildElement("lane");
			
			while (xml_lane != nullptr) {
				Lanes::Lane lane;
				this->populateLane(xml_lane, lane);
				lanes.push_back(lane);
				xml_lane = xml_lane->NextSiblingElement("lane");
			}
			
		}
		Lanes::Center center = Lanes::Center(lanes);
		std::optional<Lanes::Right> _right;
		std::optional<Lanes::Left> _left;
		tinyxml2::XMLElement* xml_left = xml_lanesection->FirstChildElement("left");
		if(xml_left != nullptr){
			tinyxml2::XMLElement* xml_lane = xml_left->FirstChildElement("lane");
			lanes.clear();
			while (xml_lane != nullptr) {
				Lanes::Lane lane;
				const char* lane_type;
				xml_lane->QueryStringAttribute("type", &lane_type);
				this->populateLane(xml_lane, lane);
				lanes.push_back(lane);
				xml_lane = xml_lane->NextSiblingElement("lane");
			}
			Lanes::Left left = Lanes::Left(lanes);
			_left = std::optional<Lanes::Left>(left);
		}
		tinyxml2::XMLElement* xml_right = xml_lanesection->FirstChildElement("right");
		if(xml_right!=nullptr){
		tinyxml2::XMLElement* xml_lane = xml_right->FirstChildElement("lane");
		lanes.clear();
		while (xml_lane != nullptr) {
				Lanes::Lane lane;
				const char* lane_type;
				xml_lane->QueryStringAttribute("type", &lane_type);
				this->populateLane(xml_lane, lane);
				lanes.push_back(lane);
				xml_lane = xml_lane->NextSiblingElement("lane");
			}
			Lanes::Right right = Lanes::Right(lanes);
			_right = std::optional<Lanes::Right>(right);
		}
		if (_left.has_value()) {
			std::sort(_left.value().lane.begin(), _left.value().lane.end());
		}
		if (_right.has_value()) {
			std::sort(_right.value().lane.begin(), _right.value().lane.end());
			std::reverse(_right.value().lane.begin(), _right.value().lane.end());
		}
		LaneSection laneSection = LaneSection(s, std::optional<bool>(singleSide), center,_right,_left);
		laneSections.push_back(laneSection);
		xml_lanesection = xml_lanesection->NextSiblingElement("laneSection");
	}
}
void OpenDriveDocument::populateLane(tinyxml2::XMLElement* xml_lane, Lanes::Lane& lane)
{
	/*Lanes::Type type;
	Lanes::Height height;
	Lanes::LaneGeometry laneGeometry;*/
	bool level;
	int id;
	std::string type;
	xml_lane->QueryBoolAttribute("level", &level);
	xml_lane->QueryIntAttribute("id", &id);
	type = xml_lane->Attribute("type");
	lane.id = id;
	lane.level = std::optional<bool>(level);
	lane.type = type;
	//
	//if (xml_lane->QueryDoubleAttribute("s", &s) != tinyxml2::XMLError::XML_SUCCESS) throw "Something wrong with 's' in lane section";
	Lanes::Link link;
	std::vector<Lanes::Height> heights;
	tinyxml2::XMLElement* xml_width = xml_lane->FirstChildElement("width");
	tinyxml2::XMLElement* xml_roadMark = xml_lane->FirstChildElement("roadMark");
	std::vector<Lanes::Width> widths;
	std::vector<Lanes::Border> borders;
	while (xml_width != nullptr) { 
		Lanes::Width width;
		this->populateGenericPolynomialTag(xml_width, width);
		widths.push_back(width);
		
		xml_width = xml_width->NextSiblingElement("width");
	}
	Lanes::LaneGeometry laneGeometry = Lanes::LaneGeometry(borders, widths);
	
	lane.laneGeometry = laneGeometry;
	while (xml_roadMark != nullptr) {
		xml_roadMark = xml_roadMark->NextSiblingElement("roadMark");
	}
}
void OpenDriveDocument::switchLaneType(std::string s_type, Lanes::Type &type)
{
}
void OpenDriveDocument::populateGeometries(tinyxml2::XMLElement* xml_road, std::vector<Geometry*>& geometries)
{
	tinyxml2::XMLElement* geometry = xml_road->FirstChildElement("planView")->FirstChildElement("geometry");
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
Link OpenDriveDocument::createLink(tinyxml2::XMLElement* xml_road)
{
	tinyxml2::XMLElement* xml_link = xml_road->FirstChildElement("link");
	tinyxml2::XMLElement* xml_successor = xml_link->FirstChildElement("successor");
	tinyxml2::XMLElement* xml_predecessor = xml_link->FirstChildElement("predecessor");
	Successor successor;
	Predecessor predecessor;
	Link link;
	if (xml_successor != NULL) {
		this->populateCessor(xml_successor, successor);
		link.successor = successor;
	}
	if(xml_predecessor != NULL){
		this->populateCessor(xml_predecessor,predecessor);
		link.predecessor = predecessor;
	}
	return link;
}
template<class T>
void OpenDriveDocument::populateCessor(tinyxml2::XMLElement* xml_cessor, T& cessor)
{
	std::string sElementId, sElementType,sContactPoint, sElementS, sElementDir;

	sElementId = std::string(xml_cessor->Attribute("elementId"));
	sElementType = std::string(xml_cessor->Attribute("elementType"));
	if (xml_cessor->Attribute("contactPoint") != NULL) {
		sContactPoint = std::string(xml_cessor->Attribute("contactPoint"));
	}
	if (xml_cessor->Attribute("elementS") != NULL) {
		sElementS = std::string(xml_cessor->Attribute("elementS"));
	}
	if (xml_cessor->Attribute("elementDir") != NULL) {
		sElementDir = std::string(xml_cessor->Attribute("elementDir"));
	}

	if (sElementS != "" || sElementDir != "") {
		throw "Not implemented cessor members!";
	}
	
	cessor.elementId = sElementId;
	if (sElementType == "road") {
		cessor.elementType = ElementType::road;
	}
	else if (sElementType == "junction") {
		cessor.elementType = ElementType::junction;
	}
	else {
		throw "Unexpected value";
	}
	if(sContactPoint != ""){
		if (sContactPoint == "start") {
			cessor.contactPoint = ContactPoint::start;
		}
		else if (sContactPoint == "end") {
			cessor.contactPoint = ContactPoint::end;
		}
		else {
			throw "Unexpected value";
		}
	}
}

template<class T>
void OpenDriveDocument::populateGenericPolynomialTag(tinyxml2::XMLElement* xml_tag, T& polynomialStruct)
{
	double sOffset, a, b, c, d;
	if (xml_tag->QueryDoubleAttribute("sOffset", &sOffset) != tinyxml2::XMLError::XML_SUCCESS) throw "Something wrong in populateGenericPolynomialTag";
	if (xml_tag->QueryDoubleAttribute("a", &a) != tinyxml2::XMLError::XML_SUCCESS) throw "Something wrong in populateGenericPolynomialTag";
	if (xml_tag->QueryDoubleAttribute("b", &b) != tinyxml2::XMLError::XML_SUCCESS) throw "Something wrong in populateGenericPolynomialTag";
	if (xml_tag->QueryDoubleAttribute("c", &c) != tinyxml2::XMLError::XML_SUCCESS) throw "Something wrong in populateGenericPolynomialTag";
	if (xml_tag->QueryDoubleAttribute("d", &d) != tinyxml2::XMLError::XML_SUCCESS) throw "Something wrong in populateGenericPolynomialTag";

	polynomialStruct.sOffset = sOffset;
	polynomialStruct.a = a;
	polynomialStruct.b = b;
	polynomialStruct.c = c;
	polynomialStruct.d = d;
}

template<class T>
void OpenDriveDocument::populateGenericPolynomialTags(tinyxml2::XMLElement* xml_tag, std::string childTagName, std::vector<T>& polynomialStructs)
{
	tinyxml2::XMLElement* xml_child_tag = xml_tag->FirstChildElement(childTagName.c_str());
	while (xml_child_tag != nullptr) {
			T polynomialStruct;
			this->populateGenericPolynomialTag(xml_tag, polynomialStruct);
			polynomialStructs.push_back(polynomialStruct);
			xml_child_tag = xml_child_tag->NextSiblingElement(childTagName.c_str());
	}
}


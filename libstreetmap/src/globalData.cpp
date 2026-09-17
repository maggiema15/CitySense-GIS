#include <map>
#include "globalData.h"
#include "m1.h"
#include "StreetsDatabaseAPI.h"
#include "OSMDatabaseAPI.h"
#include "OSMID.h"
#include "m1Helpers.h"

// Initialize globals
AdditionalData GlobalData;
ColorHandler ColorPicker;
// ChunkManager Chunks;

void AdditionalData::loadSegmentsOfStreet(int num_streets, int num_street_segments) {
    segmentsOfStreet.resize(num_streets);
    for (StreetSegmentIdx seg = 0; seg < num_street_segments; seg++) {
        segmentsOfStreet[getStreetSegmentInfo(seg).streetID].push_back(seg);
    }
}
void AdditionalData::unloadSegmentsOfStreet() {
    for (StreetIdx street = segmentsOfStreet.size() - 1; street >= 0; street--) {
        while (!segmentsOfStreet[street].empty()) {
            segmentsOfStreet[street].pop_back();
        }
        segmentsOfStreet.pop_back();
    }
}


void AdditionalData::loadTravelTimeOfSegment(int num_street_segments) {
    for (StreetSegmentIdx seg = 0; seg < num_street_segments; seg++) {
        travelTimeOfSegment.push_back(findStreetSegmentLength(seg) 
                                        / getStreetSegmentInfo(seg).speedLimit);
    }
}
void AdditionalData::unloadTravelTimeOfSegment() {
    while (!travelTimeOfSegment.empty()) {
        travelTimeOfSegment.pop_back();
    }
}


void AdditionalData::loadOSMWayByOSMID(int num_ways) {
    for (int i = 0; i < num_ways; i++) {
        const OSMWay* way = getWayByIndex(i);
        OSMWayByOSMID.insert({way->id(), way});
    }
}
void AdditionalData::unloadOSMWayByOSMID() {
    OSMWayByOSMID.clear();
}


void AdditionalData::loadOSMNodeByOSMID(int num_nodes) {
    for (int i = 0; i < num_nodes; i++) {
        const OSMNode* node = getNodeByIndex(i);
        OSMNodeByOSMID.insert({node->id(), node});
    }
}
void AdditionalData::unloadOSMNodeByOSMID() {
    OSMNodeByOSMID.clear();
}


void AdditionalData::loadSegmentsOfIntersection(int num_intersections) {
    segmentsOfIntersection.resize(num_intersections);
    for (IntersectionIdx i = 0; i < num_intersections; i++) {
        for (int seg = 0; seg < getNumIntersectionStreetSegment(i); seg++) {
            segmentsOfIntersection[i].push_back(getIntersectionStreetSegment(i, seg));
        }
    }
}
void AdditionalData::unloadSegmentsOfIntersection() {
    for (IntersectionIdx i = segmentsOfIntersection.size() - 1; i >= 0; i--) {
        while (!segmentsOfIntersection[i].empty()) {
            segmentsOfIntersection[i].pop_back();
        }
        segmentsOfIntersection.pop_back();
    }
}


void AdditionalData::loadStreetsLookUpMap(int num_streets)
{
    for (StreetIdx str = 0; str < num_streets; str++)
    {
        streetsLookUpMap.insert({simplifyString(getStreetName(str)), str});
    }
}
void AdditionalData::unloadStreetsLookUpMap()
{
    streetsLookUpMap.clear();
}


void AdditionalData::loadStreetIntersections (int num_streets)
{
    streetIntersectionsLookUp.resize(num_streets);
    for (StreetIdx str = 0; str < num_streets; str++)
    {
        std::unordered_set<IntersectionIdx> intersectionsSet;
        std::vector<StreetSegmentIdx> segments = getSegmentsOfStreet(str); //get all segments
        for (auto& it: segments)
        {
            StreetSegmentInfo info = getStreetSegmentInfo(it);
            intersectionsSet.insert(info.from); 
            intersectionsSet.insert(info.to);
        }
        for (auto& it: intersectionsSet)
        {
            streetIntersectionsLookUp[str].push_back(it);
        }
    }
}
void AdditionalData::unloadStreetIntersections ()
{
    streetIntersectionsLookUp.clear();
}

void AdditionalData::loadClosestIntersection(int num_intersection) {
    for (int itn = 0; itn < num_intersection; itn++) {
        double intersectionLat = getIntersectionPosition(itn).latitude();
        double intersectionLon = getIntersectionPosition(itn).longitude();
        std::array<double, 2> intersectionLatLon = {intersectionLat, intersectionLon};
        closestIntersectionLookUp.Insert(intersectionLatLon, itn);
    }
}

void AdditionalData::unloadClosestIntersection() {
    closestIntersectionLookUp.Clear();
}

StreetIdx AdditionalData::getStreetID(std::string street_name) {
    std::vector<StreetIdx> streetIds = findStreetIdsFromPartialStreetName(street_name);
    if (streetIds.size() == 0) return -1;

    return streetIds[0];
}

// The average latitude of the entire loaded map
void AdditionalData::loadMapAvgLat(int num_intersections) {
    double minLat = getIntersectionPosition(0).latitude();
    double maxLat = minLat;
    for (IntersectionIdx i = 0; i < num_intersections; i++) {
        minLat = std::min(minLat, getIntersectionPosition(i).latitude());
        maxLat = std::max(maxLat, getIntersectionPosition(i).latitude());
    }
    mapAvgLat = (minLat + maxLat) / 2.0;
}
// The width of the loaded map in cartesian coordinates
void AdditionalData::loadMapWidth(double min, double max) {
    mapWidth = max - min;
}

// The value at index [street_id] is the highest speed limit along that street
void AdditionalData::loadLengthOfStreet() {
    for (StreetIdx i = 0; i < getNumStreets(); i++) {
        double length = 0;
        for (int seg = 0; seg < segmentsOfStreet[i].size(); seg++) {
            length += findStreetSegmentLength(segmentsOfStreet[i][seg]);
        }
        lengthOfStreet.push_back(length);
    }
}
void AdditionalData::unloadLengthOfStreet() {
    lengthOfStreet.clear();
}

// Find the min and max x and y values of the map
void AdditionalData::loadMapBounds(int num_intersections) {
    double max_lat = getIntersectionPosition(0).latitude();
    double min_lat = max_lat;
    double max_lon = getIntersectionPosition(0).longitude();
    double min_lon = max_lon;
    for (IntersectionIdx i = 0; i < num_intersections; i++) {
        max_lat = std::max(max_lat, getIntersectionPosition(i).latitude());
        min_lat = std::min(min_lat, getIntersectionPosition(i).latitude());
        max_lon = std::max(max_lon, getIntersectionPosition(i).longitude());
        min_lon = std::min(min_lon, getIntersectionPosition(i).longitude());
    }
    minX = lonToX(min_lon, mapAvgLat);
    maxX = lonToX(max_lon, mapAvgLat);
    minY = latToY(min_lat);
    maxY = latToY(max_lat);
}


const std::vector<StreetSegmentIdx>& AdditionalData::getSegmentsOfStreet(StreetIdx street_id) const {
    return segmentsOfStreet[street_id];
}
double AdditionalData::getTravelTimeOfSegment(StreetSegmentIdx street_segment_id) const {
    return travelTimeOfSegment[street_segment_id];
}
const OSMWay* AdditionalData::getOSMWayFromOSMID(OSMID id) const {
    return OSMWayByOSMID.at(id);
}
const OSMNode* AdditionalData::getOSMNodeFromOSMID(OSMID id) const {
    return OSMNodeByOSMID.at(id);
}
const std::vector<StreetSegmentIdx>& AdditionalData::getSegmentsOfIntersection(IntersectionIdx intersection_id) const {
    return segmentsOfIntersection[intersection_id];
}
const std::vector<IntersectionIdx> AdditionalData::getStreetIntersections (StreetIdx street_id)
{
    return streetIntersectionsLookUp[street_id];
}
double AdditionalData::getMapAvgLat() const {
    return mapAvgLat;
}
double AdditionalData::getMapWidth() const {
    return mapWidth;
}

int AdditionalData::getClosestIntersection(LatLon my_position) {
    std::array<double, 2> myPositioLatLonArray = {my_position.latitude(), my_position.longitude()};
    return closestIntersectionLookUp.FindClosest(myPositioLatLonArray);
} 

double AdditionalData::getMinX() const { return minX; }
double AdditionalData::getMaxX() const { return maxX; }
double AdditionalData::getMinY() const { return minY; }
double AdditionalData::getMaxY() const { return maxY; }
double AdditionalData::getLengthOfStreet(StreetIdx street_id) const {
    return lengthOfStreet[street_id];
}


ColorHandler::ColorHandler() { pickColorScheme(LIGHT); num_themes = 2; }

void ColorHandler::pickColorScheme(ColorScheme theme) {
    switch (theme) {
        case LIGHT:
            background_color   = ezgl::WHITE;
            street_color       = ezgl::color(0xB0, 0xB0, 0xB0);
            arrow_color        = ezgl::color(0x40, 0x40, 0x40);
            selection_color    = ezgl::RED;
            feature_colors[0]  = ezgl::WHITE;                        // UKNOWN
            feature_colors[1]  = ezgl::color(0xBB, 0xFF, 0xBB);      // PARK
            feature_colors[2]  = ezgl::color(0xFF, 0xFF, 0xCC);      // BEACH
            feature_colors[3]  = ezgl::color(0x99, 0xDD, 0xFF);      // LAKE
            feature_colors[4]  = ezgl::color(0x99, 0xDD, 0xFF);      // RIVER
            feature_colors[5]  = ezgl::WHITE;                        // ISLAND
            feature_colors[6]  = ezgl::color(0xD0, 0xD0, 0xD0);      // BUILDING
            feature_colors[7]  = ezgl::color(0xBB, 0xFF, 0xBB);      // GREENSPACE
            feature_colors[8]  = ezgl::color(0xBB, 0xFF, 0xBB);      // GOLFCOURSE
            feature_colors[9]  = ezgl::color(0x99, 0xDD, 0xFF);      // STREAM
            feature_colors[10] = ezgl::LIGHT_SKY_BLUE;               // GLACIER
            label_color        = ezgl::BLACK;
            poi_color          = ezgl::color(0x66, 0x33, 0x00);
            text_box_color     = ezgl::WHITE;

            current_theme = LIGHT;
            current_theme_name = "Light";
            break;
        
        case DARK:
            background_color   = ezgl::color(0x20, 0x20, 0x20);
            street_color       = ezgl::color(0x80, 0x80, 0x90);
            arrow_color        = ezgl::color(0x40, 0x40, 0x40);
            selection_color    = ezgl::RED;
            feature_colors[0]  = ezgl::color(0x20, 0x20, 0x20);      // UKNOWN
            feature_colors[1]  = ezgl::color(0x62, 0xB5, 0x6B);      // PARK
            feature_colors[2]  = ezgl::color(0xA9, 0xA9, 0x68);      // BEACH
            feature_colors[3]  = ezgl::color(0x72, 0x9C, 0x9F);      // LAKE
            feature_colors[4]  = ezgl::color(0x72, 0x9C, 0x9F);      // RIVER
            feature_colors[5]  = ezgl::color(0x20, 0x20, 0x20);      // ISLAND
            feature_colors[6]  = ezgl::color(0x60, 0x60, 0x70);      // BUILDING
            feature_colors[7]  = ezgl::color(0x62, 0xB5, 0x6B);      // GREENSPACE
            feature_colors[8]  = ezgl::color(0x62, 0xB5, 0x6B);      // GOLFCOURSE
            feature_colors[9]  = ezgl::color(0x72, 0x9C, 0x9F);      // STREAM
            feature_colors[10] = ezgl::LIGHT_SKY_BLUE;               // GLACIER
            label_color        = ezgl::WHITE;
            poi_color          = ezgl::color(0xCC, 0xFF, 0xFF);
            text_box_color     = ezgl::WHITE;

            current_theme = DARK;
            current_theme_name = "Dark";
            break;
        
        case CONTRAST_LIGHT:
            background_color   = ezgl::color(0xF0, 0xF0, 0xF0);
            street_color       = ezgl::color(0x70, 0x70, 0x70);
            arrow_color        = ezgl::WHITE;
            selection_color    = ezgl::RED;
            feature_colors[0]  = ezgl::color(0x10, 0x10, 0x10);      // UKNOWN
            feature_colors[1]  = ezgl::color(0x00, 0xCC, 0x00);      // PARK
            feature_colors[2]  = ezgl::color(0xA9, 0xA9, 0x68);      // BEACH
            feature_colors[3]  = ezgl::color(0x00, 0x80, 0xFF);      // LAKE
            feature_colors[4]  = ezgl::color(0x00, 0x80, 0xFF);      // RIVER
            feature_colors[5]  = ezgl::color(0xF0, 0xF0, 0xF0);      // ISLAND
            feature_colors[6]  = ezgl::color(0xA0, 0xA0, 0xA0);      // BUILDING
            feature_colors[7]  = ezgl::color(0x00, 0xCC, 0x00);      // GREENSPACE
            feature_colors[8]  = ezgl::color(0x00, 0xCC, 0x00);      // GOLFCOURSE
            feature_colors[9]  = ezgl::color(0x00, 0x80, 0xFF);      // STREAM
            feature_colors[10] = ezgl::LIGHT_SKY_BLUE;               // GLACIER
            label_color        = ezgl::BLACK;
            poi_color          = ezgl::color(0x66, 0x00, 0x00);
            text_box_color     = ezgl::WHITE;

            current_theme = CONTRAST_LIGHT;
            current_theme_name = "Contrast Light";
        break;

        case CONTRAST_DARK:
            background_color   = ezgl::BLACK;
            street_color       = ezgl::color(0xE0, 0xE0, 0xE0);
            arrow_color        = ezgl::BLACK;
            selection_color    = ezgl::RED;
            feature_colors[0]  = ezgl::BLACK;                        // UKNOWN
            feature_colors[1]  = ezgl::color(0x66, 0xFF, 0x66);      // PARK
            feature_colors[2]  = ezgl::color(0xA9, 0xA9, 0x68);      // BEACH
            feature_colors[3]  = ezgl::color(0x00, 0x80, 0xFF);      // LAKE
            feature_colors[4]  = ezgl::color(0x00, 0x80, 0xFF);      // RIVER
            feature_colors[5]  = ezgl::BLACK;                        // ISLAND
            feature_colors[6]  = ezgl::color(0xA0, 0xA0, 0xA0);      // BUILDING
            feature_colors[7]  = ezgl::color(0x66, 0xFF, 0x66);      // GREENSPACE
            feature_colors[8]  = ezgl::color(0x66, 0xFF, 0x66);      // GOLFCOURSE
            feature_colors[9]  = ezgl::color(0x00, 0x80, 0xFF);      // STREAM
            feature_colors[10] = ezgl::LIGHT_SKY_BLUE;               // GLACIER
            label_color        = ezgl::WHITE;
            poi_color          = ezgl::color(0xFF, 0xDF, 0xDF);
            text_box_color     = ezgl::BLACK;

            current_theme = CONTRAST_DARK;
            current_theme_name = "Contrast Dark";
            break;

        default:
            background_color   = ezgl::WHITE;
            street_color       = ezgl::color(0xB0, 0xB0, 0xB0);
            arrow_color        = ezgl::color(0x40, 0x40, 0x40);
            selection_color    = ezgl::RED;
            feature_colors[0]  = ezgl::WHITE;                        // UKNOWN
            feature_colors[1]  = ezgl::color(0xBB, 0xFF, 0xBB);      // PARK
            feature_colors[2]  = ezgl::color(0xFF, 0xFF, 0xCC);      // BEACH
            feature_colors[3]  = ezgl::color(0x99, 0xDD, 0xFF);      // LAKE
            feature_colors[4]  = ezgl::color(0x99, 0xDD, 0xFF);      // RIVER
            feature_colors[5]  = ezgl::WHITE;                        // ISLAND
            feature_colors[6]  = ezgl::color(0xD0, 0xD0, 0xD0);      // BUILDING
            feature_colors[7]  = ezgl::color(0xBB, 0xFF, 0xBB);      // GREENSPACE
            feature_colors[8]  = ezgl::color(0xBB, 0xFF, 0xBB);      // GOLFCOURSE
            feature_colors[9]  = ezgl::color(0x99, 0xDD, 0xFF);      // STREAM
            feature_colors[10] = ezgl::LIGHT_SKY_BLUE;               // GLACIER
            label_color        = ezgl::BLACK;
            poi_color          = ezgl::color(0x66, 0x33, 0x00);
            text_box_color     = ezgl::WHITE;

            current_theme = LIGHT;
            current_theme_name = "Light";
    }
}

int ColorHandler::getNumThemes() { return num_themes; }
ColorScheme ColorHandler::getCurrentTheme() { return current_theme; }
std::string ColorHandler::getCurrentThemeName() { return current_theme_name; }
ezgl::color ColorHandler::getBackgroundColor() { return background_color; }
ezgl::color ColorHandler::getStreetColor() { return street_color; }
ezgl::color ColorHandler::getArrowColor() { return arrow_color; }
ezgl::color ColorHandler::getSelectionColor() { return selection_color; }
ezgl::color ColorHandler::getFeatureColor(FeatureType type) { return feature_colors[type]; }
ezgl::color ColorHandler::getLabelColor() { return label_color; }
ezgl::color ColorHandler::getPOIColor() { return poi_color; }
ezgl::color ColorHandler::getTextBoxColor() { return text_box_color; }


/*
// Find the coordinates where a chunk divison lies
void ChunkManager::loadChunkDivisions(int num_divisions) {
    double step_x = (GlobalData.getMaxX() - GlobalData.getMinX()) / num_divisions;
    double step_y = (GlobalData.getMaxY() - GlobalData.getMinY()) / num_divisions;
    for (int i = 0; i < num_divisions; i++) {
        x_divisions.push_back(GlobalData.getMinX() + (i * step_x));
        y_divisions.push_back(GlobalData.getMinY() + (i * step_y));
    }
}

std::pair<int, int> ChunkManager::findChunk(std::pair<double, double> point) {
    // Binary search implementation
    int low = 0, high = x_divisions.size() - 1;
    while (high - low > 1) {
        int mid = low + (high - low) / 2;
        if (x_divisions[mid] > point.first) {
            high = mid - 1;
        } else { low = mid; }
    }
    int chunk_x = low;
    low = 0, high = y_divisions.size() - 1;
    while (high - low > 1) {
        int mid = low + (high - low) / 2;
        if (y_divisions[mid] > point.second) {
            high = mid - 1;
        } else { low = mid; }
    }
    int chunk_y = low;
    return {chunk_x, chunk_y};
}
*/